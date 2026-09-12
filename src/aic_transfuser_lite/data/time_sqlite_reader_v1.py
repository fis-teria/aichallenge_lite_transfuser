"""Bounded-memory rosbag2 SQLite index for the time-path dataset.

The index pass validates one sensor message at a time and retains no camera
pixels or LiDAR arrays. Sensor rows remain :class:`RawMessageRef` values and
are decoded again by ``load_event`` when a selected anchor is assembled.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import sqlite3
from typing import Any

import numpy as np

from .clock_segments import ClockEpoch, ClockSample, segment_clock_epochs
from .mcap_converter_v2 import TimedCommand, TimedPose, TimedSteering, TimedVelocity, TimedImage, TimedLidar, _stamp_ns, _yaw_from_quaternion
from .time_history_v1 import TimeEvent
from .topic_contract_v2 import TOPIC_BY_NAME


@dataclass(frozen=True)
class RawMessageRef:
    db_relative_path: str
    row_id: int
    msgtype: str
    topic: str
    bag_timestamp_ns: int
    timestamp_ns: int
    timestamp_source: str
    shape: tuple[int, ...] = ()
    angle_min_rad: float | None = None
    angle_increment_rad: float | None = None
    range_min_m: float | None = None
    range_max_m: float | None = None


@dataclass(frozen=True)
class SQLiteRunIndex:
    run_id: str
    db_relative_path: str
    events: tuple[TimeEvent, ...]
    epochs: tuple[ClockEpoch, ...]
    topic_counts: dict[str, int]
    sensor_metadata: dict[str, dict[str, Any]]
    fallback_counts: dict[str, int]
    integrity: dict[str, Any]


def _db_path(run_dir: Path) -> Path:
    candidates = sorted(run_dir.glob("bag/*.db3"))
    if len(candidates) != 1:
        raise ValueError(f"expected one SQLite db3 in {run_dir}, got {len(candidates)}")
    return candidates[0]


@lru_cache(maxsize=2)
def _store(run_dir: Path) -> Any:
    from rosbags.typesys import Stores, get_typestore, get_types_from_idl
    store = get_typestore(Stores.ROS2_HUMBLE)
    definitions = {}
    for path in sorted((run_dir / "types").rglob("*.idl")):
        definitions.update(get_types_from_idl(path.read_text(encoding="utf-8")))
    if definitions:
        store.register(definitions)
    return store


def _topics(conn: sqlite3.Connection) -> dict[int, tuple[str, str]]:
    return {int(i): (str(name), str(typ)) for i, name, typ in conn.execute("SELECT id,name,type FROM topics ORDER BY id")}


def _clock_epochs(rows: list[tuple[int, int]], max_jump_ns: int) -> tuple[ClockEpoch, ...]:
    epochs = segment_clock_epochs(tuple(ClockSample(b, s) for b, s in rows), max_forward_jump_ns=max_jump_ns)
    if not epochs:
        raise ValueError("no /clock samples")
    return epochs


def read_time_sqlite_run(run_dir: Path, run_id: str, *, max_clock_jump_ns: int = 5_000_000_000) -> SQLiteRunIndex:
    """Create an event index with one SQLite blob read at a time.

    Camera/LiDAR payload bytes are decoded once for header/geometry validation,
    then discarded. Their semantic stamp is retained only when a valid header
    stamp exists; the returned payload is a lazy RawMessageRef.
    """
    run_dir = run_dir.resolve()
    db = _db_path(run_dir).resolve()
    uri = f"file:{db.as_posix()}?mode=ro&immutable=1"
    store = _store(run_dir)
    with sqlite3.connect(uri, uri=True) as conn:
        check = str(conn.execute("PRAGMA quick_check").fetchone()[0])
        if check != "ok":
            raise ValueError(f"SQLite quick_check failed: {check}")
        topics = _topics(conn)
        clock_topic = next((i for i, (name, _) in topics.items() if name == "/clock"), None)
        if clock_topic is None:
            raise ValueError("SQLite bag has no /clock topic")
        clock_rows: list[tuple[int, int]] = []
        for bag_ns, blob in conn.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (clock_topic,)):
            msg = store.deserialize_cdr(bytes(blob), topics[clock_topic][1])
            sim_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
            if sim_ns < 0:
                continue
            clock_rows.append((int(bag_ns), sim_ns))
        epochs = _clock_epochs(clock_rows, max_clock_jump_ns)
        all_bounds = conn.execute("SELECT MIN(timestamp),MAX(timestamp) FROM messages").fetchone()
        min_bag, max_bag = int(all_bounds[0]), int(all_bounds[1])
        # Extend epoch receipt intervals so every bag row belongs to one epoch.
        expanded: list[ClockEpoch] = []
        for i, e in enumerate(epochs):
            first = min_bag if i == 0 else e.first_bag_stamp_ns
            last = max_bag if i == len(epochs)-1 else epochs[i + 1].first_bag_stamp_ns - 1
            expanded.append(ClockEpoch(e.epoch_id, e.first_index, e.last_index, first, last,
                e.first_sim_stamp_ns, e.last_sim_stamp_ns, e.reset_reason))
        epochs = tuple(expanded)
        events: list[TimeEvent] = []
        counts: dict[str, int] = {}
        fallback: dict[str, int] = {}
        metadata: dict[str, dict[str, Any]] = {}
        wanted = {i: TOPIC_BY_NAME[name] for i, (name, typ) in topics.items() if name in TOPIC_BY_NAME}
        for i, contract in wanted.items():
            if topics[i][1] != contract.message_type:
                raise ValueError(f"unsupported topic type for {topics[i][0]}: {topics[i][1]!r}")
        query = "SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ({}) ORDER BY timestamp,id".format(",".join("?" * len(wanted)))
        for row_id, topic_id, bag_ns, blob in conn.execute(query, tuple(wanted)):
            role = wanted[int(topic_id)].role
            bag_ns = int(bag_ns)
            epoch_matches = [e for e in epochs if e.first_bag_stamp_ns <= bag_ns <= e.last_bag_stamp_ns]
            if len(epoch_matches) != 1:
                continue
            epoch = epoch_matches[0]
            typ = topics[int(topic_id)][1]
            counts[role] = counts.get(role, 0) + 1
            # Decode only compact state. Large payloads become references.
            payload: Any
            stamp = bag_ns
            source = "bag_timestamp_proxy"
            if role in {"camera", "lidar"}:
                # Decode once to obtain semantic timestamp and geometry, then
                # discard the message immediately. Pixels/ranges never enter
                # the index.
                msg = store.deserialize_cdr(bytes(blob), typ)
                stamp, source = _stamp_ns(msg, bag_ns)
                if source == "bag_timestamp_fallback" or stamp is None:
                    fallback[role] = fallback.get(role, 0) + 1
                    continue
                if role == "camera":
                    from .mcap_converter import message_image_to_rgb
                    rgb = message_image_to_rgb(msg)
                    shape = tuple(int(v) for v in rgb.shape)
                    if len(shape) != 3 or shape[2] != 3:
                        raise ValueError("invalid camera RGB shape")
                    metadata.setdefault(role, {"encodings": set(), "shapes": set()})["encodings"].add(str(msg.encoding))
                    metadata[role]["shapes"].add(shape)
                else:
                    ranges = np.asarray(msg.ranges, dtype=np.float32)
                    if ranges.ndim != 1 or ranges.size < 2 or not np.isfinite([msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max]).all() or float(msg.angle_increment) <= 0 or float(msg.range_max) <= float(msg.range_min):
                        raise ValueError("invalid LiDAR geometry")
                    shape = (int(ranges.size),)
                    metadata.setdefault(role, {"shapes": set(), "frames": set()})["shapes"].add(shape)
                    metadata[role]["frames"].add(str(msg.header.frame_id))
                payload = RawMessageRef(db.relative_to(run_dir).as_posix(), int(row_id), typ, topics[int(topic_id)][0], bag_ns, int(stamp), source, shape=shape)
            else:
                msg = store.deserialize_cdr(bytes(blob), typ)
                stamp, source = _stamp_ns(msg, bag_ns)
                if source == "bag_timestamp_fallback" or stamp is None:
                    fallback[role] = fallback.get(role, 0) + 1
                    continue
                if role == "pose":
                    p = msg.pose.pose
                    payload = TimedPose(stamp, float(p.position.x), float(p.position.y), _yaw_from_quaternion(p.orientation), str(msg.header.frame_id), str(msg.child_frame_id), bag_ns, source)
                    metadata.setdefault(role, {"frames": set()})["frames"].add((str(msg.header.frame_id), str(msg.child_frame_id)))
                elif role == "velocity":
                    payload = TimedVelocity(stamp, float(msg.longitudinal_velocity), float(msg.lateral_velocity), float(msg.heading_rate), bag_ns, source)
                elif role in {"final_command", "nominal_command"}:
                    payload = TimedCommand(stamp, float(msg.longitudinal.speed), float(msg.longitudinal.acceleration), float(msg.lateral.steering_tire_angle), bag_ns, source)
                elif role == "actual_steering":
                    payload = TimedSteering(stamp, float(msg.steering_tire_angle), bag_ns, source)
                else:
                    continue
            events.append(TimeEvent(role, run_id, epoch.epoch_id, "sim", "bag_receipt", int(stamp), bag_ns, int(row_id), payload, "bag_receipt_proxy"))
        for value in metadata.values():
            for key, item in list(value.items()):
                if isinstance(item, set): value[key] = sorted(item)
        return SQLiteRunIndex(run_id, db.relative_to(run_dir).as_posix(), tuple(events), epochs, counts, metadata, fallback, {"quick_check": check})


def load_event(run_dir: Path, event: TimeEvent) -> TimeEvent:
    """Resolve one RawMessageRef; large data is decoded only for this event."""
    ref = event.payload
    if not isinstance(ref, RawMessageRef):
        return event
    from rosbags.typesys import Stores, get_typestore
    store = _store(run_dir)
    db = (run_dir / ref.db_relative_path).resolve()
    uri = f"file:{db.as_posix()}?mode=ro&immutable=1"
    with sqlite3.connect(uri, uri=True) as conn:
        row = conn.execute("SELECT data FROM messages WHERE id=?", (ref.row_id,)).fetchone()
    if row is None:
        raise KeyError(f"message row not found: {ref.row_id}")
    msg = store.deserialize_cdr(bytes(row[0]), ref.msgtype)
    if event.role == "camera":
        from .mcap_converter import message_image_to_rgb
        payload = TimedImage(ref.timestamp_ns, message_image_to_rgb(msg), ref.bag_timestamp_ns, ref.timestamp_source)
    elif event.role == "lidar":
        payload = TimedLidar(ref.timestamp_ns, np.asarray(msg.ranges, dtype=np.float32), float(msg.angle_min), float(msg.angle_increment), float(msg.range_min), float(msg.range_max), str(msg.header.frame_id), ref.bag_timestamp_ns, ref.timestamp_source)
    else:
        raise ValueError(f"RawMessageRef role is not loadable: {event.role}")
    return TimeEvent(event.role, event.run, event.epoch, event.capture_clock, event.available_clock, event.capture_ns, event.available_ns, event.sequence, payload, event.availability_source)
