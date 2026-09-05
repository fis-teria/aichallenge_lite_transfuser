"""Offline S1 MCAP index core. No path adapter, decoder, decompressor or raw CLI.

All offsets/sizes are integer bytes, timestamps uint64 ns. Inputs are synthetic
random-access sources only. Returned descriptors are never actual-source results.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import io
import json
import struct
import time
from typing import Any, Callable, Mapping, Protocol
import zlib

VERSION = "spatial_s1_index_v4_offline_v2_read_layout"
PLAN_ID = "937cbbd09efa8e7fe729a0e58687a39157ca7ec1582c49b231f1a6fe82411597"
PRIOR_ID = "394292c7c268715a5efc4cd408f0e9634835d5d4cf016729a01c9f4786dde71d"
PLAN_COMMIT = "29c11b1c04e5cb40b237e4bfc4a747fc0b0658d1"
SOURCE = "/home/thistle/e2e_autonomous/datasets/raw/d1log_0902_all_v3/20260902-131505/rosbag2_autoware_0.mcap"
METADATA_HASH = "087f45eb23b6e8aa846822b155d603bf7f40157d999a3396a55a9383751c4b05"
RUN = "20260902-131505"
TOPIC = "/localization/kinematic_state"
MAGIC = b"\x89MCAP0\r\n"
U64 = 2**64 - 1
WINDOWS = {
    "8fbd120c37e872d2bc51ab58bc95813636caa7de04a335b560e9337c6993f12b": (5939999861, 6439999861),
    "208cfcac87744ded9ef39f3c85ac2ae6d3f545255e81239e33e8819e1947b20c": (256009994272, 256509994272),
    "353301a96a1f493e253091d78bef0c43c2b90818097aa892e392a5756250811d": (5199999878, 5699999878),
    "ffe514be1a0f7756c75006e4066566bcaf025610e726962ed1ecb9c93fca65f7": (689999978, 1189999978),
}
ENVELOPE = {"source_bytes": 67108864, "expanded_bytes": 134217728, "messages": 5000,
            "seconds": 60, "temporary_disk_bytes": 0, "single_record_bytes": 16777216, "chunks": 8}
FLAGS = {"raw_execution_authorized": False, "approval_gate": "PENDING_EXPLICIT_AUTHORIZATION",
         "deployment_or_training_approved": False, "actual_raw_reads_performed": 0,
         "actual_dataset_body_reads_performed": 0}


class S1Error(ValueError):
    def __init__(self, status: str, detail: str):
        self.status = status
        super().__init__(detail)


def check(ok: bool, detail: str, status: str = "INVALID_S1_STRUCTURE") -> None:
    if not ok:
        raise S1Error(status, detail)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def identity(value: Any) -> str:
    return sha(json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


def uint(value: Any, maximum: int = U64) -> bool:
    return type(value) is int and 0 <= value <= maximum


def exclusive_stop(start_ns: int, end_ns: int, consumer_max: int = U64) -> tuple[int, int]:
    check(uint(start_ns) and uint(end_ns) and start_ns <= end_ns and uint(consumer_max), "invalid inclusive timestamp range")
    check(end_ns < min(U64, consumer_max) and start_ns <= consumer_max, "exclusive stop overflow")
    return start_ns, end_ns + 1


@dataclass(frozen=True)
class Probe:
    group_id: str
    window_ns: tuple[int, int]
    records: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class Contract:
    plan_identity: str
    source_locator: str
    metadata_sha256: str
    probes: tuple[Probe, ...]
    # These are separate stop predicates, not allocations of extra envelopes.
    individual_chunk_cap: int = 8
    union_chunk_cap: int = 8
    source_run: str = RUN
    source_id: str = "rosbag2_autoware_0.mcap:metadata:" + METADATA_HASH


def validate_s1_contract(plan_mapping: Mapping[str, Any]) -> Contract:
    """Read mappings only. Recalculate new plan identity using its canonical recipe.

    The seven artifact mappings do not authorize or cause filesystem access.
    Original-nine artifacts are not read or reclassified by this module.
    """
    try:
        m = plan_mapping["execution_manifest.json"]
        p = plan_mapping["minimal_read_proposal.json"]
        a = plan_mapping["approval_request.json"]
        q = plan_mapping["claim_requirements.json"]
        check(m["status"] == "COMPLETE_PLAN_ONLY" and m["plan_commit"] == PLAN_COMMIT, "plan version", "BLOCKED_PLAN")
        entries = m["input_files"]
        hashes = {e["name"]: e["sha256"] for e in entries if not e["name"].startswith("prior/")}
        prior_hashes = {e["name"]: e["sha256"] for e in entries if e["name"].startswith("prior/")}
        check(len(hashes) == 9 and len(prior_hashes) == 6 and len(entries) == 15, "input inventory", "BLOCKED_PLAN")
        check(plan_mapping["input_manifest.json"]["files"] == entries, "input inventory mismatch", "BLOCKED_PLAN")
        calculated = identity({"policy": m["format"], "limits": m["limits"], "input_hashes": hashes,
            "prior_hashes": prior_hashes, "prior_logical_identity": PRIOR_ID, "code_hashes": m["code_hashes"],
            "result": {"proposal": p, "approval": a, "facts": m["facts"]}, "claims": q, "status": m["status"]})
        check(calculated == m["logical_plan_identity"] == PLAN_ID and
              m["prior_logical_identity_recomputed"] == PRIOR_ID, "canonical plan identity mismatch", "BLOCKED_PLAN")
        for mapping in (m, p, a):
            check(mapping["raw_execution_authorized"] is False and mapping["deployment_or_training_approved"] is False and
                  mapping["approval_gate"] == FLAGS["approval_gate"], "authorization mutated", "BLOCKED_PLAN")
        check(a["automatic_stage_transition"] is False and all(s["authorized"] is False for s in a["stages"].values()), "stages authorized", "BLOCKED_PLAN")
        check({k: v["proposed_limit"] for k, v in a["envelope"]["limits"].items()} == ENVELOPE, "envelope changed", "BLOCKED_PLAN")
        check(all(v["estimate"] is None for v in a["envelope"]["limits"].values()), "cost estimate mutated", "BLOCKED_PLAN")
        check(len(p["seeds"]) == len(p["closures"]) == 4 and len(p["legacy_claims"]) == 14, "claim cohort", "BLOCKED_PLAN")
        seeds = {s["group_id"]: s for s in p["seeds"]}
        check(set(seeds) == set(WINDOWS), "seed selection changed", "BLOCKED_PLAN")
        probes = []
        for c in p["closures"]:
            gid = c["group_id"]
            check(c["claim_id"] == "record_pair_binding:" + gid and c["claim_type"] == "B_RECORD_BINDING" and
                  c["closure_kind"] == "listed_record_binding", "legacy/replay scope injection", "BLOCKED_PLAN")
            check(tuple(c["search_window_log_time_ns_inclusive"]) == WINDOWS[gid], "window changed", "BLOCKED_PLAN")
            check(c["required_candidates"] == seeds[gid]["candidates"] and len(c["required_candidates"]) == 2 and
                  c["required_clock_records"] == c["required_anchor_velocity_endpoints"] == [], "record scope changed", "BLOCKED_PLAN")
            check(c["universe"]["source"]["absolute_path_opaque"] == SOURCE and
                  c["universe"]["source"]["metadata_sha256_reported"] == METADATA_HASH, "source locator mismatch", "BLOCKED_PLAN")
            for r in c["required_candidates"]:
                check(r["run_id"] == RUN and r["topic"] == TOPIC and
                      r["source_id"] == "rosbag2_autoware_0.mcap:metadata:" + METADATA_HASH, "source record mismatch", "BLOCKED_PLAN")
            probes.append(Probe(gid, WINDOWS[gid], tuple((r["record_id"], r["payload_sha256"]) for r in c["required_candidates"])))
        check(len({r for probe in probes for r, _ in probe.records}) == 8, "union not eight records", "BLOCKED_PLAN")
        return Contract(calculated, SOURCE, METADATA_HASH, tuple(sorted(probes, key=lambda x: x.group_id)))
    except S1Error:
        raise
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError) as error:
        raise S1Error("BLOCKED_PLAN", "invalid plan mapping: " + str(error)) from error


class ByteSource(Protocol):
    synthetic: bool
    @property
    def size(self) -> int: ...
    @property
    def revision(self) -> object: ...
    def read_at(self, offset: int, length: int) -> bytes: ...


class MemorySource:
    """Synthetic bytes only. Deliberately no constructor accepting paths or streams."""
    synthetic = True

    def __init__(self, data: bytes):
        check(type(data) is bytes, "memory source needs bytes", "UNSUPPORTED_S1_INPUT")
        self._stream = io.BytesIO(data)
        self._size = len(data)

    @property
    def size(self) -> int:
        return self._size

    @property
    def revision(self) -> object:
        return 0

    def read_at(self, offset: int, length: int) -> bytes:
        check(uint(offset) and uint(length) and offset + length <= self.size, "memory read range")
        self._stream.seek(offset)
        return self._stream.read(length)


@dataclass(frozen=True)
class MetadataInput:
    synthetic_bytes: bytes
    run_id: str
    source_id: str
    storage_format: str = "plain_mcap"


@dataclass(frozen=True)
class SyntheticReadLayout:
    """External fixture authority, NOT raw authorization or source-derived evidence.

    Ranges are sorted, non-overlapping [start, stop) byte intervals. The fixture
    builder supplies them independently of serialized Footer/Index declarations.
    Object identity binds this in-process layout, not a real-file identity proof.
    """
    synthetic_only: bool
    contract: Contract
    source: ByteSource
    source_size: int
    source_revision: object
    allowed_ranges: tuple[tuple[int, int], ...]
    provenance: str

    def validate(self, source: ByteSource, contract: Contract) -> None:
        check(self.synthetic_only is True and getattr(source, "synthetic", False) is True,
              "layout is synthetic only", "BLOCKED_READ_LAYOUT")
        check(self.contract == contract and self.source is source and
              uint(self.source_size) and self.source_size == source.size and
              self.source_revision == source.revision,
              "layout contract/source/size/revision mismatch", "BLOCKED_READ_LAYOUT_BINDING")
        check(type(self.provenance) is str and bool(self.provenance.strip()),
              "layout provenance missing", "BLOCKED_READ_LAYOUT")
        check(type(self.allowed_ranges) is tuple and bool(self.allowed_ranges),
              "layout ranges missing or mutable", "BLOCKED_READ_LAYOUT")
        previous_stop = 0
        for interval in self.allowed_ranges:
            check(type(interval) is tuple and len(interval) == 2,
                  "invalid layout interval shape", "BLOCKED_READ_LAYOUT")
            start, stop = interval
            check(uint(start) and uint(stop) and previous_stop <= start < stop <= self.source_size,
                  "layout interval outside uint64/source or overlapping/unsorted", "BLOCKED_READ_LAYOUT")
            previous_stop = stop

    def covers(self, offset: int, length: int) -> bool:
        if not (uint(offset) and uint(length) and offset + length <= self.source_size):
            return False
        stop, cursor = offset + length, offset
        for start, end in self.allowed_ranges:
            if end < cursor:
                continue
            if start > cursor:
                return False
            cursor = max(cursor, end)
            if cursor >= stop:
                return True
        return False


@dataclass
class Ledger:
    """One synthetic envelope across attempts. Human waiting time is not active time."""
    limits: dict[str, int] = field(default_factory=lambda: dict(ENVELOPE))
    clock: Callable[[], float] = time.monotonic
    returned_source_bytes: int = 0
    metadata_bytes_hashed: int = 0
    parsed_index_records: int = 0
    parsed_index_entries: int = 0
    read_calls: int = 0
    range_rejections: list[dict[str, int]] = field(default_factory=list)
    attempts: int = 0
    active_seconds: float = 0.0
    accounting_known: bool = True
    selected_chunks: set[int] = field(default_factory=set)
    _started: float | None = None
    _binding: tuple | None = None
    _source_stamp: tuple | None = None

    def begin(self, contract: Contract) -> None:
        check(self._started is None and self.accounting_known, "ledger unavailable after unknown accounting", "UNKNOWN_ACCOUNTING")
        check(set(self.limits) == set(ENVELOPE) and all(uint(v) and v <= ENVELOPE[k] for k, v in self.limits.items()), "envelope may only tighten", "BLOCKED_CONTRACT")
        binding = (contract.plan_identity, contract.source_locator, contract.metadata_sha256, contract.probes)
        check(self._binding in (None, binding), "ledger cannot change source/contract", "BLOCKED_CONTRACT")
        self._binding = binding
        self.attempts += 1
        self._started = self.clock()
        self.check_time()

    def check_time(self) -> None:
        elapsed = self.active_seconds + (self.clock() - self._started if self._started is not None else 0)
        check(elapsed <= self.limits["seconds"], "active-time envelope exceeded", "PARTIAL_BUDGET")

    def finish(self) -> None:
        if self._started is not None:
            self.active_seconds += self.clock() - self._started
            self._started = None

    def snapshot(self) -> dict:
        return {"returned_source_bytes": self.returned_source_bytes, "metadata_bytes_hashed": self.metadata_bytes_hashed,
            "metadata_index_records": self.parsed_index_records, "index_entries": self.parsed_index_entries,
            "unique_intersecting_chunks_across_attempts": len(self.selected_chunks), "read_calls": self.read_calls,
            "core_pre_read_range_rejections": len(self.range_rejections),
            "rejected_ranges": [dict(item) for item in self.range_rejections],
            "attempts": self.attempts, "active_seconds": self.active_seconds, "accounting_known": self.accounting_known,
            "payload_acquisition_attempts": 0, "payload_expansion_attempts": 0, "decoded_messages": 0,
            "expanded_bytes": 0, "temporary_disk_bytes": 0}


class Cursor:
    def __init__(self, data: bytes):
        self.data, self.pos = data, 0

    def take(self, n: int) -> bytes:
        check(uint(n) and self.pos + n <= len(self.data), "truncated field")
        value = self.data[self.pos:self.pos + n]
        self.pos += n
        return value

    def number(self, fmt: str) -> int:
        return struct.unpack("<" + fmt, self.take(struct.calcsize("<" + fmt)))[0]

    def string(self) -> str:
        try:
            return self.take(self.number("I")).decode("utf-8", errors="strict")
        except UnicodeError as error:
            raise S1Error("INVALID_S1_STRUCTURE", "invalid UTF-8") from error

    def done(self) -> None:
        check(self.pos == len(self.data), "record extension not supported by this core", "UNSUPPORTED_S1_EXTENSION")


class Reader:
    def __init__(self, source: ByteSource, ledger: Ledger, contract: Contract,
                 layout: SyntheticReadLayout | None = None):
        check(isinstance(layout, SyntheticReadLayout), "independent synthetic layout required", "BLOCKED_READ_LAYOUT")
        layout.validate(source, contract)
        self.layout = layout
        self.source, self.ledger = source, ledger
        self.size, self.revision = source.size, source.revision
        check(uint(self.size), "source size")
        self.forbidden: list[tuple[int, int]] = []

    def read(self, offset: int, length: int) -> bytes:
        self.ledger.check_time()
        check(self.source.size == self.size and self.source.revision == self.revision,
              "layout source changed before read", "BLOCKED_READ_LAYOUT_BINDING")
        if not self.layout.covers(offset, length):
            self.ledger.range_rejections.append({"offset_bytes": offset, "length_bytes": length})
            raise S1Error("BLOCKED_READ_RANGE", "candidate read not fully covered by independent synthetic layout")
        check(length <= self.ledger.limits["single_record_bytes"], "allocation/read size exceeds cap", "PARTIAL_BUDGET")
        check(self.ledger.returned_source_bytes + length <= self.ledger.limits["source_bytes"], "source-byte envelope exceeded before read", "PARTIAL_BUDGET")
        check(not any(offset < end and offset + length > start for start, end in self.forbidden), "attempt to read declared chunk region", "BLOCKED_PAYLOAD_BOUNDARY")
        self.ledger.read_calls += 1
        try:
            data = self.source.read_at(offset, length)
        except Exception as error:
            self.ledger.accounting_known = False
            raise S1Error("UNKNOWN_ACCOUNTING", "source exception; consumed bytes unknown, retry prohibited") from error
        if type(data) is not bytes:
            self.ledger.accounting_known = False
            raise S1Error("UNKNOWN_ACCOUNTING", "source returned non-bytes")
        self.ledger.returned_source_bytes += len(data)  # charge BEFORE timeout/truncation/change checks
        self.ledger.check_time()
        check(self.source.revision == self.revision and self.source.size == self.size, "synthetic source changed", "SOURCE_CHANGED")
        check(len(data) == length, "short or excess read; bytes retained in ledger", "PARTIAL_READ")
        return data

    def record(self, offset: int, end: int, opcode: int) -> tuple[bytes, int]:
        check(self.ledger.parsed_index_records < 10000, "metadata/index record analysis cap", "PARTIAL_BUDGET")
        check(offset + 9 <= end, "truncated record header")
        head = self.read(offset, 9)
        op, length = struct.unpack("<BQ", head)
        check(op == opcode, "unexpected record opcode", "UNSUPPORTED_S1_STRUCTURE")
        check(offset + 9 + length <= end, "record length exceeds enclosing range")
        body = self.read(offset + 9, length)
        self.ledger.parsed_index_records += 1
        return body, offset + 9 + length


def records(data: bytes, start: int, ledger: Ledger) -> list[tuple[int, bytes, int, int]]:
    cursor, result = Cursor(data), []
    while cursor.pos < len(data):
        ledger.check_time()
        check(ledger.parsed_index_records < 10000, "metadata/index record analysis cap", "PARTIAL_BUDGET")
        position = cursor.pos
        op, length = cursor.number("B"), cursor.number("Q")
        check(length <= ledger.limits["single_record_bytes"], "record allocation cap", "PARTIAL_BUDGET")
        body = cursor.take(length)
        ledger.parsed_index_records += 1
        result.append((op, body, start + position, 9 + length))
    return result


def chunk_index(body: bytes) -> dict:
    c = Cursor(body)
    start, end, offset, length = (c.number("Q") for _ in range(4))
    map_bytes = Cursor(c.take(c.number("I")))
    indexes = {}
    while map_bytes.pos < len(map_bytes.data):
        channel, pos = map_bytes.number("H"), map_bytes.number("Q")
        check(channel not in indexes, "duplicate channel in chunk-index map")
        indexes[channel] = pos
    index_length = c.number("Q")
    compression = c.string()
    compressed, expanded = c.number("Q"), c.number("Q")
    c.done()
    check(start <= end and length >= 9 and compressed <= length and expanded > 0, "invalid chunk descriptor")
    return {"log_start_ns": start, "log_end_ns": end, "chunk_start_offset": offset, "chunk_length": length,
            "message_index_offsets": indexes, "message_index_length": index_length, "compression": compression,
            "declared_compressed_bytes": compressed, "declared_uncompressed_bytes": expanded}


def parse_summary(rows: list, result: dict, ledger: Ledger) -> None:
    schemas, channels, chunks = result["schemas"], result["channels"], result["all_chunk_descriptors"]
    for op, body, _, _ in rows:
        ledger.check_time()
        c = Cursor(body)
        if op == 3:
            sid, name, encoding = c.number("H"), c.string(), c.string()
            definition = c.take(c.number("I"))
            c.done()
            check(sid != 0, "schema zero ID")
            row = {"id": sid, "name": name, "encoding": encoding, "definition_sha256": sha(definition), "definition_bytes": len(definition)}
            check(sid not in schemas or schemas[sid] == row, "same schema ID has conflicting definition")
            schemas[sid] = row
        elif op == 4:
            cid, sid, topic, encoding = c.number("H"), c.number("H"), c.string(), c.string()
            metadata = Cursor(c.take(c.number("I")))
            entries = {}
            while metadata.pos < len(metadata.data):
                k, v = metadata.string(), metadata.string()
                check(k not in entries, "duplicate channel metadata key")
                entries[k] = v
            c.done()
            row = {"id": cid, "schema_id": sid, "topic": topic, "message_encoding": encoding, "metadata": entries}
            check(cid not in channels or channels[cid] == row, "same channel ID has conflicting definition")
            channels[cid] = row
        elif op == 8:
            chunks.append(chunk_index(body))
        else:
            raise S1Error("UNSUPPORTED_S1_STRUCTURE", f"summary opcode {op} outside minimal supported subset")
    check(bool(chunks), "no chunk index; no scan fallback", "UNSUPPORTED_S1_NO_INDEX")
    check(bool(channels) and bool(schemas), "summary definitions missing", "BLOCKED_MISSING_DEFINITION")
    for row in channels.values():
        check(row["schema_id"] in schemas, "channel schema missing from summary", "BLOCKED_MISSING_DEFINITION")
        schema = schemas[row["schema_id"]]
        check(bool(schema["encoding"]) and schema["definition_bytes"] > 0 and bool(row["message_encoding"]), "required encoding/definition missing", "BLOCKED_MISSING_DEFINITION")
    names = {}
    for sid, row in schemas.items():
        names.setdefault(row["name"], []).append(sid)
    result["same_name_different_schema"] = {name: ids for name, ids in names.items() if len({schemas[i]["definition_sha256"] for i in ids}) > 1}


def inspect_s1_index(byte_source: ByteSource, metadata_input: MetadataInput, contract: Contract, ledger: Ledger,
                     layout: SyntheticReadLayout | None = None) -> dict:
    """Only synthetic random-access I/O; S1 readiness is never B/C/D/E success."""
    result: dict[str, Any] = {**FLAGS, "format": VERSION, "mode": "OFFLINE_SYNTHETIC_ONLY", "status": "NOT_EXECUTED",
        "schemas": {}, "channels": {}, "all_chunk_descriptors": [], "selected_chunk_descriptors": [],
        "message_indexes": [], "crc": {"summary": "NOT_INSPECTED", "chunk_and_data_crc": "NOT_INSPECTED_PAYLOAD_OUT_OF_SCOPE"},
        "s2_authorized": False, "s2_candidates_within_caps": False,
        "read_boundary_basis": "INDEPENDENT_SYNTHETIC_LAYOUT_REQUIRED",
        "read_layout_verified": False,
        "claims": {name: "NOT_EXECUTED" for name in ("B_RECORD_BINDING", "original_occurrence_uniqueness",
            "C_LISTED_PAIR_PROJECTION", "B_STREAM_REPLAY", "C_COMPLETE_CANDIDATE_INVARIANCE", "D_PHYSICAL_ACCURACY",
            "E_GEOMETRY_SUPERVISION", "E_STOP_LAUNCH_LABELS", "E_MOTION_PERMISSION_SAFETY", "E_CONTROLLER_ORACLE")},
        "all_record_header_candidate_completeness": "UNKNOWN", "actual_source_chunk_count": None,
        "declared_size_meaning": "reference only; not guaranteed actual expansion or required cost",
        "s2_retry_chunk_counting": "PENDING_EXPLICIT_POLICY_AND_REVIEW"}
    already_active = ledger._started is not None
    try:
        check(getattr(byte_source, "synthetic", False) is True, "only synthetic sources allowed", "UNSUPPORTED_S1_INPUT")
        check(metadata_input.storage_format == "plain_mcap", "file compression/other format unsupported, no fallback", "UNSUPPORTED_S1_INPUT")
        check(len(contract.probes) == 4 and len({r for p in contract.probes for r, _ in p.records}) == 8 and
              all(len(p.records) == 2 for p in contract.probes), "four probes/eight records required", "BLOCKED_CONTRACT")
        check(uint(contract.individual_chunk_cap) and uint(contract.union_chunk_cap), "invalid chunk caps", "BLOCKED_CONTRACT")
        for probe in contract.probes:
            exclusive_stop(*probe.window_ns)
        ledger.begin(contract)
        check(type(metadata_input.synthetic_bytes) is bytes and len(metadata_input.synthetic_bytes) <= ledger.limits["single_record_bytes"], "synthetic metadata size", "PARTIAL_BUDGET")
        ledger.metadata_bytes_hashed += len(metadata_input.synthetic_bytes)
        check(sha(metadata_input.synthetic_bytes) == contract.metadata_sha256 and metadata_input.run_id == contract.source_run and
              metadata_input.source_id == contract.source_id, "synthetic metadata/source binding mismatch", "SOURCE_BINDING_MISMATCH")
        r = Reader(byte_source, ledger, contract, layout)
        result["read_layout_verified"] = True
        result["read_layout_provenance"] = r.layout.provenance
        stamp = (r.size, r.revision)
        check(ledger._source_stamp in (None, stamp), "source changed between attempts", "SOURCE_CHANGED")
        ledger._source_stamp = stamp
        check(r.size >= 54, "truncated MCAP")
        check(r.read(0, 8) == MAGIC, "not plain MCAP v0", "UNSUPPORTED_S1_INPUT")
        check(r.read(r.size - 8, 8) == MAGIC, "trailing magic mismatch")
        footer_pos = r.size - 37
        footer, end = r.record(footer_pos, r.size - 8, 2)
        check(len(footer) == 20 and end == r.size - 8, "footer length")
        ss, so, crc = struct.unpack("<QQI", footer)
        check(ss != 0, "no summary/index; no fallback", "UNSUPPORTED_S1_NO_INDEX")
        check(8 < ss <= (so or footer_pos) <= footer_pos, "summary/footer ranges")
        header, header_end = r.record(8, ss, 1)
        hc = Cursor(header)
        result["header"] = {"profile": hc.string(), "library": hc.string()}
        hc.done()
        check(header_end <= ss, "summary overlaps header")
        summary_bytes = r.read(ss, footer_pos - ss)
        actual_crc = zlib.crc32(summary_bytes + struct.pack("<BQ", 2, 20) + footer[:16]) & 0xffffffff
        result["crc"]["summary"] = "NOT_AVAILABLE" if crc == 0 else "MATCH" if crc == actual_crc else "MISMATCH"
        check(crc == 0 or crc == actual_crc, "summary CRC mismatch", "CRC_MISMATCH")
        summary_end = so or footer_pos
        rows = records(summary_bytes[:summary_end - ss], ss, ledger)
        groups = {}
        last = None
        for op, _, pos, length in rows:
            if op != last:
                check(op not in groups, "summary opcodes not grouped")
                groups[op] = [pos, 0]
                last = op
            groups[op][1] += length
        if so:
            offsets = records(summary_bytes[so - ss:], so, ledger)
            found = {}
            for op, body, _, _ in offsets:
                check(op == 14 and len(body) == 17, "invalid summary-offset record")
                group_op, group_start, group_length = struct.unpack("<BQQ", body)
                check(group_op not in found and groups.get(group_op) == [group_start, group_length], "summary-offset group boundary mismatch")
                found[group_op] = [group_start, group_length]
            check(found == groups, "incomplete summary-offset table")
        result["summary_offsets"] = "VERIFIED" if so else "NOT_AVAILABLE"
        parse_summary(rows, result, ledger)
        chunks = result["all_chunk_descriptors"]
        ranges = []
        for chunk in chunks:
            ledger.check_time()
            start = chunk["chunk_start_offset"]
            end = start + chunk["chunk_length"]
            index_end = end + chunk["message_index_length"]
            check(header_end <= start < end <= index_end <= ss, "chunk/index descriptor out of bounds")
            check(bool(chunk["message_index_offsets"]), "message index absent; no chunk fallback", "UNSUPPORTED_S1_NO_INDEX")
            check(len(set(chunk["message_index_offsets"].values())) == len(chunk["message_index_offsets"]), "aliased message-index offset")
            for cid, offset in chunk["message_index_offsets"].items():
                check(cid in result["channels"], "index channel missing", "BLOCKED_MISSING_DEFINITION")
                check(end <= offset < index_end, "message index offset outside own index region")
            ranges.append((start, index_end))
            r.forbidden.append((start, end))  # Whole Chunk, not merely compressed payload.
        ordered = sorted(ranges)
        check(all(a[1] <= b[0] for a, b in zip(ordered, ordered[1:])), "overlapping/duplicate chunk or index ranges")
        counts = {p.group_id: 0 for p in contract.probes}
        for chunk in chunks:  # Complete bounded summary enumeration; never timestamp early stop.
            ledger.check_time()
            matched = [p.group_id for p in contract.probes if chunk["log_start_ns"] <= p.window_ns[1] and chunk["log_end_ns"] >= p.window_ns[0]]
            if matched:
                result["selected_chunk_descriptors"].append({**chunk, "probe_group_ids": matched})
                ledger.selected_chunks.add(chunk["chunk_start_offset"])
                for gid in matched:
                    counts[gid] += 1
        selected = result["selected_chunk_descriptors"]
        result["chunk_caps"] = {"per_probe_counts": counts, "union_count": len(selected),
            "individual_pass": all(v <= contract.individual_chunk_cap for v in counts.values()),
            "union_pass": len(selected) <= contract.union_chunk_cap,
            "physical_envelope_pass": len(ledger.selected_chunks) <= ledger.limits["chunks"]}
        check(all(result["chunk_caps"][k] for k in ("individual_pass", "union_pass", "physical_envelope_pass")),
              "intersecting chunk cap exceeded; descriptors retained, not truncated", "S1_INDEX_CAP_BLOCKED")
        target_channels = {cid for cid, channel in result["channels"].items() if channel["topic"] == TOPIC}
        check(bool(target_channels), "required topic channel missing", "BLOCKED_MISSING_DEFINITION")
        target_counts = {p.group_id: 0 for p in contract.probes}
        result["target_window_index_entries"] = target_counts
        for chunk in selected:
            ledger.check_time()
            # Read index headers/bodies only, never Chunk records or Message.data.
            starts = sorted(chunk["message_index_offsets"].items(), key=lambda pair: pair[1])
            region_start = chunk["chunk_start_offset"] + chunk["chunk_length"]
            region_end = region_start + chunk["message_index_length"]
            check(starts[0][1] == region_start, "index region leading gap unsupported", "UNSUPPORTED_S1_STRUCTURE")
            for i, (cid, pos) in enumerate(starts):
                boundary = starts[i + 1][1] if i + 1 < len(starts) else region_end
                if cid not in target_channels:
                    continue  # No clock/velocity record acquisition added.
                body, after = r.record(pos, boundary, 7)
                check(after == boundary, "message index record boundary mismatch")
                mc = Cursor(body)
                check(mc.number("H") == cid, "message-index channel mismatch")
                entries = Cursor(mc.take(mc.number("I")))
                mc.done()
                row = {"chunk_start_offset": chunk["chunk_start_offset"], "channel_id": cid, "entries": []}
                result["message_indexes"].append(row)
                while entries.pos < len(entries.data):
                    ledger.check_time()
                    check(ledger.parsed_index_entries < 100000, "index entry analysis cap", "PARTIAL_BUDGET")
                    timestamp, offset = entries.number("Q"), entries.number("Q")
                    ledger.parsed_index_entries += 1
                    check(chunk["log_start_ns"] <= timestamp <= chunk["log_end_ns"] and
                          offset + 9 <= chunk["declared_uncompressed_bytes"], "index timestamp/record offset out of descriptor bounds")
                    row["entries"].append({"log_time_ns": timestamp, "uncompressed_record_offset": offset})
                    for probe in contract.probes:
                        if probe.window_ns[0] <= timestamp <= probe.window_ns[1]:
                            target_counts[probe.group_id] += 1
        check(all(target_counts.values()), "required topic/window index entry not acquired", "PARTIAL_SCOPE")
        result["declared_size_sums"] = {k: sum(c[k] for c in selected) for k in ("declared_compressed_bytes", "declared_uncompressed_bytes")}
        check(byte_source.revision == r.revision and byte_source.size == r.size, "synthetic source changed", "SOURCE_CHANGED")
        ledger.check_time()
        result["s2_candidates_within_caps"] = True
        result["status"] = "S1_SYNTHETIC_INDEX_INSPECTED"
    except S1Error as error:
        result.update(status=error.status, detail=str(error))
    except Exception as error:
        result.update(status="BLOCKED_CORE_EXCEPTION", detail=type(error).__name__ + ":" + str(error))
    finally:
        if not already_active and ledger._started is not None:
            ledger.finish()
        result["synthetic_io"] = ledger.snapshot()
    return result
