"""Budgeted teacher/debug-only MCAP windows. Never writes/decompresses a bag to disk.

Indexed plain MCAP uses rosbags' index reader, but not its unbounded decompressor.
File-zstd MCAP is scanned forward with explicit source/expanded byte accounting.
Only allowlisted non-sensor messages are deserialized. Shared chunks may contain
sensor bytes; these bytes are counted and discarded, never exported as sensors.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from io import BytesIO
import math
from pathlib import Path
import struct
import time
from typing import Any, BinaryIO, Iterator, Sequence


TOPICS = (
    "/localization/kinematic_state", "/vehicle/status/velocity_status",
    "/nominal_control_cmd", "/control/command/control_cmd", "/clock",
    "/vehicle/status/gear_status", "/vehicle/status/control_mode", "/awsim/state",
    "/safety_reason", "/plan_diagnostics", "/overtake/race_armed",
    "/autostart/initialization_ready",
)


class BudgetExceeded(RuntimeError):
    pass


@dataclass(frozen=True)
class ReadBudget:
    max_source_bytes: int = 256 * 1024**2
    max_expanded_bytes: int = 1024 * 1024**2
    max_messages: int = 50000
    max_seconds: float = 180.0
    max_temporary_bytes: int = 0
    max_record_bytes: int = 64 * 1024**2

    def validate(self) -> None:
        if any(v <= 0 for k, v in asdict(self).items() if k != "max_temporary_bytes"):
            raise ValueError("reader budgets must be positive")
        if not math.isfinite(self.max_seconds) or self.max_temporary_bytes < 0:
            raise ValueError("invalid time/temporary budget")


class ReadMeter:
    def __init__(self, config: ReadBudget) -> None:
        config.validate()
        self.config = config
        self.started = time.monotonic()
        self.source_bytes = 0
        self.expanded_bytes = 0
        self.decoded_messages = 0
        self.temporary_bytes = 0

    def check(self) -> None:
        if time.monotonic() - self.started > self.config.max_seconds:
            raise BudgetExceeded("max_seconds")

    def charge(self, name: str, amount: int) -> None:
        self.check()
        limits = {"source_bytes": self.config.max_source_bytes,
                  "expanded_bytes": self.config.max_expanded_bytes,
                  "decoded_messages": self.config.max_messages}
        if amount < 0 or getattr(self, name) + amount > limits[name]:
            raise BudgetExceeded(name)
        setattr(self, name, getattr(self, name) + amount)

    def snapshot(self) -> dict[str, Any]:
        return {"source_bytes": self.source_bytes, "expanded_bytes": self.expanded_bytes,
                "decoded_messages": self.decoded_messages, "temporary_bytes": 0,
                "elapsed_seconds": time.monotonic() - self.started}


class MeteredStream:
    """Read-only wrapper; counts actual returned bytes, rejects oversized requests."""

    def __init__(self, stream: BinaryIO, meter: ReadMeter, counter: str) -> None:
        self.stream, self.meter, self.counter = stream, meter, counter

    def read(self, size: int = -1) -> bytes:
        self.meter.check()
        if size < 0:
            # rosbags index reader reads the known 37-byte footer to EOF.
            pos = self.stream.tell()
            self.stream.seek(0, 2)
            size = self.stream.tell() - pos
            self.stream.seek(pos)
        limit = (self.meter.config.max_source_bytes if self.counter == "source_bytes"
                 else self.meter.config.max_expanded_bytes)
        if size > limit - getattr(self.meter, self.counter):
            raise BudgetExceeded(self.counter)
        data = self.stream.read(size)
        self.meter.charge(self.counter, len(data))
        return data

    def readinto(self, buffer: Any) -> int:
        data = self.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)

    def seek(self, offset: int, whence: int = 0) -> int:
        self.meter.check()
        return self.stream.seek(offset, whence)

    def tell(self) -> int:
        return self.stream.tell()

    def close(self) -> None:
        self.stream.close()

    def readable(self) -> bool:
        return True

    def seekable(self) -> bool:
        return self.stream.seekable()


def _exact(stream: Any, count: int) -> bytes:
    data = stream.read(count)
    if len(data) != count:
        raise ValueError("truncated MCAP record")
    return data


def _u32(stream: Any) -> int:
    return struct.unpack("<I", _exact(stream, 4))[0]


def _string(stream: Any, maximum: int = 4 * 1024**2) -> str:
    size = _u32(stream)
    if size > maximum:
        raise BudgetExceeded("schema_or_string_size")
    return _exact(stream, size).decode("utf-8")


def _discard(stream: Any, count: int, *, seekable: bool = False) -> None:
    if seekable:
        stream.seek(count, 1)
        return
    while count:
        step = min(count, 65536)
        _exact(stream, step)
        count -= step


def _expand(data: bytes, compression: str, expected: int, meter: ReadMeter) -> bytes:
    if expected > meter.config.max_record_bytes:
        raise BudgetExceeded("max_record_bytes")
    if expected + 1 > meter.config.max_expanded_bytes - meter.expanded_bytes:
        raise BudgetExceeded("expanded_bytes")
    if compression == "":
        result = data
    elif compression == "zstd":
        import zstandard
        # Streaming output cap also defends against incorrect declared chunk size.
        with zstandard.ZstdDecompressor(max_window_size=65536).stream_reader(BytesIO(data)) as reader:
            result = reader.read(expected + 1)
    else:
        raise ValueError(f"unsupported bounded chunk compression: {compression}")
    meter.charge("expanded_bytes", len(result))
    if len(result) != expected:
        raise ValueError("MCAP expanded size mismatch")
    return result


def _intersects(start: int, end: int, windows: Sequence[tuple[int, int]]) -> bool:
    return any(start <= right and end >= left for left, right in windows)


def _plain(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if hasattr(value, "item"):
        return _plain(value.item())
    return str(value)


def decode_message(raw: bytes, typename: str, topic: str, bag_ns: int, store: Any,
                   meter: ReadMeter, source_id: str) -> dict[str, Any]:
    meter.charge("decoded_messages", 1)
    message = store.deserialize_cdr(raw, typename)
    from .mcap_converter_v2 import _stamp_ns, _yaw_from_quaternion
    stamp, stamp_source = _stamp_ns(message, bag_ns)
    value: dict[str, Any] = {}
    if typename == "nav_msgs/msg/Odometry":
        pose = message.pose.pose
        value = {"frame_id": message.header.frame_id, "child_frame_id": message.child_frame_id,
                 "x_m": float(pose.position.x), "y_m": float(pose.position.y),
                 "yaw_rad": _yaw_from_quaternion(pose.orientation)}
    elif hasattr(message, "longitudinal_velocity"):
        value = {"longitudinal_mps": float(message.longitudinal_velocity),
                 "lateral_mps": float(message.lateral_velocity), "yaw_rate_rps": float(message.heading_rate)}
    elif hasattr(message, "longitudinal") and hasattr(message, "lateral"):
        value = {"speed_mps": float(message.longitudinal.speed),
                 "acceleration_mps2": float(message.longitudinal.acceleration),
                 "steering_rad": float(message.lateral.steering_tire_angle)}
    elif hasattr(message, "clock"):
        value = {"clock_ns": int(message.clock.sec) * 10**9 + int(message.clock.nanosec)}
    else:
        for name in ("data", "mode", "report", "status", "state"):
            if hasattr(message, name):
                value[name] = _plain(getattr(message, name))
    return {"source_id": source_id, "topic": topic, "type": typename,
            "bag_stamp_ns": bag_ns, "semantic_stamp_ns": stamp, "timestamp_source": stamp_source,
            "payload_sha256": hashlib.sha256(raw).hexdigest(), "value": value}


def read_mcap_windows(path: Path, windows: Sequence[tuple[int, int]], *, meter: ReadMeter,
                      topics: Sequence[str] = TOPICS) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Windows are bag-record nanoseconds (provisional +/- header margin in plan).

    Source read time and expanded chunks are bounded globally across invocations.
    Partial records remain evidence, never imply complete window coverage.
    """
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    from .spatial_coverage_v4 import sha256_file
    store = get_typestore(Stores.ROS2_HUMBLE)
    schemas: dict[int, tuple[str, str]] = {}
    channels: dict[int, tuple[int, str]] = {}
    records: list[dict[str, Any]] = []
    before = meter.snapshot()
    file_stat = path.stat()
    meta_path = path.parent / "metadata.yaml"
    source_id = f"{path.name}:metadata:{sha256_file(meta_path) if meta_path.exists() else 'fixture'}"
    result: dict[str, Any] = {"path": str(path), "source_id": source_id,
        "size_bytes": file_stat.st_size, "windows_bag_ns": list(windows), "topics": list(topics),
        "status": "COMPLETE", "mode": "forward_stream" if path.suffix in {".zstd", ".zst"} else "indexed",
        "bag_header_time_relation": "must_be_checked_from_returned_records", "decode_errors": []}

    def register(sid: int, name: str, definition: str) -> None:
        schemas[sid] = (name, definition)

    def parse(stream: Any, *, top: bool, outer_seek: bool = False) -> None:
        while True:
            meter.check()
            opcode = stream.read(1)
            if not opcode:
                return
            op = opcode[0]
            if op == 0x89:  # trailing MCAP magic
                return
            size = struct.unpack("<Q", _exact(stream, 8))[0]
            if op in (2, 15):
                _discard(stream, size, seekable=outer_seek)
                return
            if op == 6:
                start, end, expanded, _ = struct.unpack("<QQQI", _exact(stream, 28))
                if top and start > max(right for _, right in windows) and channels:
                    result["scan_stop"] = "past_requested_bag_window_monotonic_log_time_assumption"
                    return
                compression = _string(stream)
                compressed = struct.unpack("<Q", _exact(stream, 8))[0]
                if 40 + len(compression.encode()) + compressed != size:
                    raise ValueError("invalid MCAP chunk length")
                if _intersects(start, end, windows) or not channels:
                    if compressed > meter.config.max_record_bytes:
                        raise BudgetExceeded("max_record_bytes")
                    parse(BytesIO(_expand(_exact(stream, compressed), compression, expanded, meter)), top=False)
                else:
                    _discard(stream, compressed, seekable=outer_seek)
                continue
            if op not in (3, 4, 5):
                _discard(stream, size, seekable=outer_seek)
                continue
            if size > meter.config.max_record_bytes:
                raise BudgetExceeded("max_record_bytes")
            if op == 5:
                channel, _, stamp, _ = struct.unpack("<HIQQ", _exact(stream, 22))
                sid, topic = channels.get(channel, (-1, "unknown"))
                if topic not in topics or not _intersects(stamp, stamp, windows):
                    _discard(stream, size - 22, seekable=outer_seek)
                    continue
                raw = _exact(stream, size - 22)
                name, definition = schemas[sid]
                try:
                    if name not in store.types:
                        store.register(get_types_from_msg(definition, name))
                    records.append(decode_message(raw, name, topic, stamp, store, meter, source_id))
                except BudgetExceeded:
                    raise
                except (ValueError, KeyError, AttributeError, AssertionError) as error:
                    result["decode_errors"].append({"topic": topic, "stamp_ns": stamp, "error": str(error)})
                continue
            rec = BytesIO(_exact(stream, size))
            sid = struct.unpack("<H", _exact(rec, 2))[0]
            if op == 3:
                name, encoding, definition = _string(rec), _string(rec), _string(rec)
                if encoding != "ros2msg":
                    raise ValueError(f"unsupported schema encoding {encoding}")
                register(sid, name, definition)
            else:
                schema = struct.unpack("<H", _exact(rec, 2))[0]
                topic, encoding = _string(rec), _string(rec)
                if encoding != "cdr":
                    raise ValueError("unsupported channel encoding")
                channels[sid] = (schema, topic)

    try:
        with path.open("rb") as file:
            counted = MeteredStream(file, meter, "source_bytes")
            if path.suffix in {".zstd", ".zst"}:
                import zstandard
                with zstandard.ZstdDecompressor(max_window_size=65536).stream_reader(counted, closefd=False) as expanded:
                    stream = MeteredStream(expanded, meter, "expanded_bytes")
                    if _exact(stream, 8) != b"\x89MCAP0\r\n":
                        raise ValueError("invalid MCAP magic")
                    parse(stream, top=True)
            else:
                from rosbags.rosbag2.storage_mcap import McapReader

                class IndexOnly(McapReader):
                    def meta_scan(self) -> None:
                        raise ValueError("MCAP index unavailable; bounded scan not silently substituted")

                class ReadPath:
                    def open(self, mode: str) -> Any:
                        if mode != "rb":
                            raise ValueError("read only")
                        return counted

                reader = IndexOnly(ReadPath())
                reader.open()
                for sid, schema in reader.schemas.items():
                    register(sid, schema.name, schema.data)
                names = {name: sid for sid, (name, _) in schemas.items()}
                channels.update({cid: (names[ch.schema], ch.topic) for cid, ch in reader.channels.items()})
                result["indexed_chunks"] = len(reader.chunks)
                wanted = [c for c in reader.chunks if _intersects(c.message_start_time, c.message_end_time, windows)]
                result["selected_chunks"] = len(wanted)
                for chunk in wanted:
                    meter.check()
                    if chunk.compressed_size > meter.config.max_record_bytes:
                        raise BudgetExceeded("max_record_bytes")
                    counted.seek(chunk.chunk_start_offset + 9 + 40 + len(chunk.compression))
                    data = _expand(_exact(counted, chunk.compressed_size), chunk.compression, chunk.uncompressed_size, meter)
                    parse(BytesIO(data), top=False)
                if not reader.chunks:
                    raise ValueError("no indexed chunks")
        if result["decode_errors"]:
            result["status"] = "PARTIAL"
    except BudgetExceeded as error:
        result.update(status="BUDGET_EXCEEDED", reason=str(error))
    except (OSError, ValueError, KeyError, AssertionError) as error:
        result.update(status="BLOCKED", reason=f"{type(error).__name__}: {error}")
    result["actual"] = {key: value - before[key] for key, value in meter.snapshot().items()}
    result["source_stat_unchanged"] = (path.stat().st_size, path.stat().st_mtime_ns) == (file_stat.st_size, file_stat.st_mtime_ns)
    result["returned_messages"] = len(records)
    result["logical_payload_identity"] = hashlib.sha256("\n".join(r["payload_sha256"] for r in records).encode()).hexdigest()
    return records, result
