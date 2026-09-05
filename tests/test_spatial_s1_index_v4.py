"""Hand-packed synthetic structures only: no bag-derived fixture or disk source."""
from __future__ import annotations

import ast
import builtins
from copy import deepcopy
from dataclasses import replace
import inspect
import io
import socket
import struct
import zlib

import pytest

from aic_transfuser_lite.data import spatial_s1_index_v4 as s1


def number(fmt: str, *values: int) -> bytes:
    return struct.pack("<" + fmt, *values)


def string(value: str) -> bytes:
    data = value.encode()
    return number("I", len(data)) + data


def record(op: int, body: bytes) -> bytes:
    return number("BQ", op, len(body)) + body


def schema(sid: int = 1, definition: bytes = b"uint64 synthetic_value", name: str = "synthetic/State") -> bytes:
    return record(3, number("H", sid) + string(name) + string("ros2msg") + number("I", len(definition)) + definition)


def channel(cid: int = 1, sid: int = 1, topic: str = s1.TOPIC) -> bytes:
    return record(4, number("HH", cid, sid) + string(topic) + string("cdr") + number("I", 0))


def contract() -> s1.Contract:
    # IDs/hashes here label synthetic records only, never saved-source candidates.
    probes = tuple(s1.Probe(gid, window, tuple((f"synthetic-{i}-{j}", s1.sha(f"fake-{i}-{j}".encode()))
                   for j in range(2))) for i, (gid, window) in enumerate(s1.WINDOWS.items()))
    return s1.Contract("synthetic-plan", "opaque-synthetic-source", s1.sha(b"synthetic metadata"), probes,
                       source_run="synthetic-run", source_id="synthetic-source-id")


def metadata() -> s1.MetadataInput:
    return s1.MetadataInput(b"synthetic metadata", "synthetic-run", "synthetic-source-id")


class GuardedSource(s1.MemorySource):
    def __init__(self, data: bytes, forbidden: list[tuple[int, int]],
                 allowed_ranges: tuple[tuple[int, int], ...] = ()):
        super().__init__(data)
        self.forbidden = forbidden
        self.requests: list[tuple[int, int]] = []
        self.current_revision = 0
        self.allowed_ranges = allowed_ranges
        self.sentinel_rejections = 0

    @property
    def revision(self) -> object:
        return self.current_revision

    def read_at(self, offset: int, length: int) -> bytes:
        self.requests.append((offset, length))  # Record issuance, even if sentinel rejects.
        assert length >= 0
        if any(offset < end and offset + length > start for start, end in self.forbidden):
            self.sentinel_rejections += 1
            raise AssertionError("payload sentinel touched")
        return super().read_at(offset, length)


def fixture(*, spans: list[tuple[int, int]] | None = None, order: list[int] | None = None,
            schemas: bytes | None = None, channels: bytes | None = None, crc: str = "absent",
            offsets: bool = True, no_index: bool = False, extra_summary: bytes = b"",
            index_mutation: str = "", header: bytes | None = None, empty_entries: bool = False) -> GuardedSource:
    """The Chunk's sentinel bytes are deliberately opaque, not decodable messages."""
    spans = spans if spans is not None else [(0, 257_000_000_000)]
    data = s1.MAGIC + (header if header is not None else record(1, string("ros2") + string("synthetic-only")))
    allowed = [(0, len(data))]  # Builder-owned positions, not parsed source declarations.
    descriptors, forbidden = [], []
    for start, end in spans:
        chunk_start = len(data)
        payload = b"SYNTHETIC_PAYLOAD_DO_NOT_READ" * 4
        chunk = record(6, number("QQQI", start, end, 1024, 0) + string("") + number("Q", len(payload)) + payload)
        data += chunk
        forbidden.append((chunk_start, len(data)))
        index_start = len(data)
        times = [lo for lo, hi in s1.WINDOWS.values() if start <= lo <= end]
        times.reverse()  # Message Index need not be sorted either.
        entries = b"" if empty_entries else b"".join(number("QQ", t, 0) for t in times)
        mi_body = number("H", 2 if index_mutation == "channel" else 1) + number("I", len(entries)) + entries
        mi = record(7, mi_body)
        data += mi
        allowed.append((index_start, len(data)))
        mapped_offset = chunk_start if index_mutation == "payload_offset" else index_start
        map_data = number("HQ", 1, mapped_offset)
        if index_mutation == "duplicate_channel":
            map_data += map_data
        body = number("QQQQ", start, end, chunk_start, len(chunk)) + number("I", len(map_data)) + map_data
        body += number("Q", len(mi)) + string("") + number("QQ", len(payload), 1024)
        if index_mutation == "out_of_bounds":
            body = body[:16] + number("Q", s1.U64) + body[24:]
        descriptors.append(record(8, body))
    # DataEnd is deliberately not read; its CRC covers payload, outside S1.
    data += record(15, number("I", 0))
    summary_start = len(data)
    groups = [schemas if schemas is not None else schema(), channels if channels is not None else channel()]
    if not no_index:
        groups.append(b"".join(descriptors[i] for i in (order if order is not None else range(len(descriptors)))))
    if extra_summary:
        groups.append(extra_summary)
    summary, positions = b"", []
    for group in groups:
        if group:
            positions.append((group[0], summary_start + len(summary), len(group)))
            summary += group
    summary_offset_start = summary_start + len(summary) if offsets else 0
    table = b"".join(record(14, number("BQQ", *p)) for p in positions) if offsets else b""
    footer_prefix = number("QQ", summary_start, summary_offset_start)
    checksum = zlib.crc32(summary + table + number("BQ", 2, 20) + footer_prefix) & 0xffffffff
    checksum = 0 if crc == "absent" else checksum if crc == "match" else checksum ^ 1
    data += summary + table + record(2, footer_prefix + number("I", checksum)) + s1.MAGIC
    allowed.append((summary_start, len(data)))
    return GuardedSource(data, forbidden, tuple(allowed))


def layout_for(source: GuardedSource, c: s1.Contract | None = None) -> s1.SyntheticReadLayout:
    return s1.SyntheticReadLayout(True, c or contract(), source, source.size, source.revision,
                                  source.allowed_ranges, "hand-built fixture positions before serialization")


def run(source: s1.ByteSource | None = None, *, c: s1.Contract | None = None,
        ledger: s1.Ledger | None = None, meta: s1.MetadataInput | None = None) -> dict:
    source, c = source or fixture(), c or contract()
    return s1.inspect_s1_index(source, meta or metadata(), c, ledger or s1.Ledger(), layout_for(source, c))


def assert_closed(result: dict) -> None:
    for key, value in s1.FLAGS.items():
        assert result[key] == value
    assert not result["s2_authorized"]
    assert set(result["claims"].values()) == {"NOT_EXECUTED"}
    assert result["all_record_header_candidate_completeness"] == "UNKNOWN"
    assert result["actual_source_chunk_count"] is None
    for key in ("payload_acquisition_attempts", "payload_expansion_attempts", "decoded_messages", "expanded_bytes", "temporary_disk_bytes"):
        assert result["synthetic_io"][key] == 0


def test_normal_shared_chunk_payload_sentinel_and_nonpromotion() -> None:
    source = fixture(crc="match")
    result = run(source)
    assert result["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    assert result["summary_offsets"] == "VERIFIED"
    assert result["crc"]["summary"] == "MATCH"
    assert result["schemas"][1]["definition_sha256"] == s1.sha(b"uint64 synthetic_value")
    assert result["chunk_caps"]["union_count"] == 1
    assert list(result["chunk_caps"]["per_probe_counts"].values()) == [1] * 4
    assert result["synthetic_io"]["returned_source_bytes"] == sum(n for _, n in source.requests)
    assert len(result["message_indexes"][0]["entries"]) == 4
    assert_closed(result)


def test_unsorted_four_windows_no_early_complete() -> None:
    spans = list(s1.WINDOWS.values()) + [(999_000_000_000, 999_000_000_001)]
    result = run(fixture(spans=spans, order=[4, 1, 3, 0, 2]))
    assert result["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    assert len(result["all_chunk_descriptors"]) == 5
    assert len(result["selected_chunk_descriptors"]) == 4
    assert all(result["target_window_index_entries"].values())


@pytest.mark.parametrize("kind,expected", [("absent", "NOT_AVAILABLE"), ("match", "MATCH"), ("bad", "MISMATCH")])
def test_crc(kind: str, expected: str) -> None:
    result = run(fixture(crc=kind))
    assert result["crc"]["summary"] == expected
    assert result["status"] == ("CRC_MISMATCH" if kind == "bad" else "S1_SYNTHETIC_INDEX_INSPECTED")


def test_optional_summary_offsets() -> None:
    result = run(fixture(offsets=False))
    assert result["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    assert result["summary_offsets"] == "NOT_AVAILABLE"


@pytest.mark.parametrize("kind,expected", [
    ("same_name", "S1_SYNTHETIC_INDEX_INSPECTED"), ("same_id", "INVALID_S1_STRUCTURE"),
    ("same_id_identical", "S1_SYNTHETIC_INDEX_INSPECTED"), ("unknown_schema", "BLOCKED_MISSING_DEFINITION"),
    ("missing_schema", "BLOCKED_MISSING_DEFINITION"), ("missing_channel", "BLOCKED_MISSING_DEFINITION"),
    ("empty_definition", "BLOCKED_MISSING_DEFINITION"), ("channel_conflict", "INVALID_S1_STRUCTURE"),
    ("no_topic", "BLOCKED_MISSING_DEFINITION"), ("no_index", "UNSUPPORTED_S1_NO_INDEX"),
    ("extra", "UNSUPPORTED_S1_STRUCTURE"), ("empty_entries", "PARTIAL_SCOPE")])
def test_definitions_and_unsupported(kind: str, expected: str) -> None:
    options = {
        "same_name": {"schemas": schema() + schema(2, b"different")},
        "same_id": {"schemas": schema() + schema(1, b"different")},
        "same_id_identical": {"schemas": schema() * 2},
        "unknown_schema": {"channels": channel(sid=2)}, "missing_schema": {"schemas": b""},
        "missing_channel": {"channels": b""}, "empty_definition": {"schemas": schema(definition=b"")},
        "channel_conflict": {"channels": channel() + channel(topic="different")},
        "no_topic": {"channels": channel(topic="other")}, "no_index": {"no_index": True},
        "extra": {"extra_summary": record(11, b"\0" * 46)}, "empty_entries": {"empty_entries": True}}
    result = run(fixture(**options[kind]))
    assert result["status"] == expected
    if kind == "same_name":
        assert result["same_name_different_schema"] == {"synthetic/State": [1, 2]}
    assert_closed(result)


@pytest.mark.parametrize("mutation", ["payload_offset", "out_of_bounds", "channel", "duplicate_channel"])
def test_index_corruption_before_payload_read(mutation: str) -> None:
    source = fixture(index_mutation=mutation)
    result = run(source)
    assert result["status"] == "INVALID_S1_STRUCTURE"
    assert result["synthetic_io"]["accounting_known"]


@pytest.mark.parametrize("kind", ["short", "magic", "footer", "utf8", "oversized", "prefix", "summary_range"])
def test_malformed_lengths_and_bounds(kind: str) -> None:
    source = fixture()
    data = source._stream.getvalue()
    if kind == "short":
        data = data[:20]
    elif kind == "magic":
        data = b"INVALID!" + data[8:]
    elif kind == "footer":
        data = data[:-37] + number("BQ", 2, 21) + data[-28:]
    elif kind == "utf8":
        source = fixture(header=record(1, number("I", 1) + b"\xff" + string("library")))
        data = source._stream.getvalue()
    elif kind == "oversized":
        data = data[:9] + number("Q", s1.U64) + data[17:]
    elif kind == "prefix":
        source = fixture(header=record(1, number("I", 999999)))
        data = source._stream.getvalue()
    else:
        data = data[:-28] + number("Q", s1.U64) + data[-20:]
    result = run(GuardedSource(data, source.forbidden, source.allowed_ranges))
    assert result["status"] in {"INVALID_S1_STRUCTURE", "UNSUPPORTED_S1_INPUT", "BLOCKED_READ_LAYOUT"}
    assert_closed(result)


@pytest.mark.parametrize("start,end,maximum", [(0, s1.U64, s1.U64), (0, 2**63 - 1, 2**63 - 1),
                                               (-1, 1, s1.U64), (2, 1, s1.U64), (False, 1, s1.U64)])
def test_exclusive_overflow(start: int, end: int, maximum: int) -> None:
    with pytest.raises(s1.S1Error):
        s1.exclusive_stop(start, end, maximum)


def test_exclusive_boundaries() -> None:
    assert s1.exclusive_stop(0, 0) == (0, 1)
    assert s1.exclusive_stop(s1.U64 - 1, s1.U64 - 1) == (s1.U64 - 1, s1.U64)


@pytest.mark.parametrize("predicate", ["individual", "union", "physical"])
def test_separate_chunk_caps(predicate: str) -> None:
    c, ledger = contract(), s1.Ledger()
    if predicate == "individual":
        c = replace(c, individual_chunk_cap=0)
    elif predicate == "union":
        c = replace(c, union_chunk_cap=0)
    else:
        ledger.limits["chunks"] = 0
    result = run(c=c, ledger=ledger)
    assert result["status"] == "S1_INDEX_CAP_BLOCKED"
    key = "physical_envelope_pass" if predicate == "physical" else predicate + "_pass"
    assert not result["chunk_caps"][key]
    assert sum(not result["chunk_caps"][k] for k in ("individual_pass", "union_pass", "physical_envelope_pass")) == 1


def test_nine_chunks_retained_not_truncated() -> None:
    result = run(fixture(spans=[(0, 257_000_000_000)] * 9))
    assert result["status"] == "S1_INDEX_CAP_BLOCKED"
    assert len(result["selected_chunk_descriptors"]) == 9
    assert result["synthetic_io"]["unique_intersecting_chunks_across_attempts"] == 9
    assert not result["s2_candidates_within_caps"]
    assert result["message_indexes"] == []


@pytest.mark.parametrize("limit", ["source_bytes", "single_record_bytes"])
def test_oversized_request_rejected_before_read(limit: str) -> None:
    source, ledger = fixture(), s1.Ledger()
    ledger.limits[limit] = 7
    result = run(source, ledger=ledger)
    assert result["status"] == "PARTIAL_BUDGET"
    assert source.requests == []


def test_post_read_timeout_debits_bytes() -> None:
    source = fixture()
    now = [0.0]
    original = source.read_at
    def slow(offset: int, length: int) -> bytes:
        data = original(offset, length)
        now[0] += 61.0
        return data
    source.read_at = slow
    result = run(source, ledger=s1.Ledger(clock=lambda: now[0]))
    assert result["status"] == "PARTIAL_BUDGET"
    assert result["synthetic_io"]["returned_source_bytes"] == 8
    assert result["synthetic_io"]["active_seconds"] == 61


def test_partial_read_retry_retains_debit() -> None:
    source, ledger = fixture(), s1.Ledger()
    original = source.read_at
    source.read_at = lambda offset, length: original(offset, length)[:3]
    first = run(source, ledger=ledger)
    assert first["status"] == "PARTIAL_READ"
    assert first["synthetic_io"]["returned_source_bytes"] == 3
    source.read_at = original
    second = run(source, ledger=ledger)
    assert second["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    assert second["synthetic_io"]["attempts"] == 2
    assert ledger.returned_source_bytes == 3 + sum(n for _, n in source.requests[1:])


def test_unknown_accounting_is_sticky_retry_forbidden() -> None:
    source, ledger = fixture(), s1.Ledger()
    original = source.read_at
    def fail(offset: int, length: int) -> bytes:
        original(offset, length)
        raise OSError("synthetic consumed amount unavailable")
    source.read_at = fail
    first = run(source, ledger=ledger)
    assert first["status"] == "UNKNOWN_ACCOUNTING"
    source.read_at = original
    second = run(source, ledger=ledger)
    assert second["status"] == "UNKNOWN_ACCOUNTING"
    assert ledger.attempts == 1 and len(source.requests) == 1
    assert not second["synthetic_io"]["accounting_known"]


def test_retry_rereads_cumulative_budget_and_wait_excluded() -> None:
    source = fixture()
    now = [0.0]
    ledger = s1.Ledger(clock=lambda: now[0])
    assert run(source, ledger=ledger)["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    consumed = ledger.returned_source_bytes
    now[0] += 99999  # idle human approval wait, not an active attempt
    assert run(source, ledger=ledger)["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    assert ledger.returned_source_bytes == 2 * consumed
    assert ledger.active_seconds == 0 and len(ledger.selected_chunks) == 1
    ledger.limits["source_bytes"] = ledger.returned_source_bytes
    assert run(source, ledger=ledger)["status"] == "PARTIAL_BUDGET"
    assert ledger.returned_source_bytes == 2 * consumed


@pytest.mark.parametrize("when", ["during", "between"])
def test_source_change(when: str) -> None:
    source, ledger = fixture(), s1.Ledger()
    if when == "between":
        assert run(source, ledger=ledger)["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
        source.current_revision += 1
    else:
        original = source.read_at
        def changed(offset: int, length: int) -> bytes:
            data = original(offset, length)
            source.current_revision += 1
            return data
        source.read_at = changed
    assert run(source, ledger=ledger)["status"] == "SOURCE_CHANGED"


@pytest.mark.parametrize("kind", ["format", "metadata", "run", "source", "real_source"])
def test_binding_and_non_synthetic_rejected_without_io(kind: str) -> None:
    source, meta = fixture(), metadata()
    if kind == "format":
        meta = replace(meta, storage_format="file-zstd")
    elif kind == "metadata":
        meta = replace(meta, synthetic_bytes=b"different")
    elif kind in ("run", "source"):
        meta = replace(meta, **{"run_id" if kind == "run" else "source_id": "different"})
    else:
        source.synthetic = False
    result = run(source, meta=meta)
    assert result["status"] in {"SOURCE_BINDING_MISMATCH", "UNSUPPORTED_S1_INPUT"}
    assert source.requests == []


def synthetic_plan(monkeypatch: pytest.MonkeyPatch) -> dict:
    """Test the canonical binding with a synthetic hash, never overwrite production PLAN_ID."""
    entries = [{"name": f"original/{i}", "sha256": s1.sha(str(i).encode())} for i in range(9)]
    entries += [{"name": f"prior/{i}", "sha256": s1.sha(str(i).encode())} for i in range(6)]
    m = {**s1.FLAGS, "status": "COMPLETE_PLAN_ONLY", "plan_commit": s1.PLAN_COMMIT, "input_files": entries,
         "format": "synthetic-plan", "limits": {}, "code_hashes": {}, "facts": {},
         "prior_logical_identity_recomputed": s1.PRIOR_ID}
    p = {**s1.FLAGS, "seeds": [], "closures": [], "legacy_claims": list(range(14))}
    for i, (gid, window) in enumerate(s1.WINDOWS.items()):
        candidates = [{"record_id": f"synthetic-{i}-{j}", "payload_sha256": s1.sha(f"synthetic-{i}-{j}".encode()),
                       "run_id": s1.RUN, "topic": s1.TOPIC, "source_id": "rosbag2_autoware_0.mcap:metadata:" + s1.METADATA_HASH} for j in range(2)]
        p["seeds"].append({"group_id": gid, "candidates": candidates})
        p["closures"].append({"group_id": gid, "claim_id": "record_pair_binding:" + gid, "claim_type": "B_RECORD_BINDING",
            "closure_kind": "listed_record_binding", "search_window_log_time_ns_inclusive": list(window),
            "required_candidates": deepcopy(candidates), "required_clock_records": [], "required_anchor_velocity_endpoints": [],
            "universe": {"source": {"absolute_path_opaque": s1.SOURCE, "metadata_sha256_reported": s1.METADATA_HASH}}})
    a = {**s1.FLAGS, "automatic_stage_transition": False, "stages": {"S1": {"authorized": False}, "S2": {"authorized": False}},
         "envelope": {"limits": {k: {"proposed_limit": v, "estimate": None} for k, v in s1.ENVELOPE.items()}}}
    q = {"synthetic": True}
    digest = s1.identity({"policy": m["format"], "limits": m["limits"],
        "input_hashes": {e["name"]: e["sha256"] for e in entries[:9]}, "prior_hashes": {e["name"]: e["sha256"] for e in entries[9:]},
        "prior_logical_identity": s1.PRIOR_ID, "code_hashes": m["code_hashes"], "result": {"proposal": p, "approval": a, "facts": m["facts"]},
        "claims": q, "status": m["status"]})
    m["logical_plan_identity"] = digest
    monkeypatch.setattr(s1, "PLAN_ID", digest)
    return {"execution_manifest.json": m, "minimal_read_proposal.json": p, "approval_request.json": a,
            "claim_requirements.json": q, "input_manifest.json": {"files": entries},
            "unresolved_and_unrecoverable.json": {}, "report_ja.md": "synthetic only"}


def test_plan_contract_record_hashes_from_mapping_only(monkeypatch: pytest.MonkeyPatch) -> None:
    plan = synthetic_plan(monkeypatch)
    before = deepcopy(plan)
    c = s1.validate_s1_contract(plan)
    assert len(c.probes) == 4 and sum(len(p.records) for p in c.probes) == 8
    assert plan == before


@pytest.mark.parametrize("mutation", ["window", "legacy", "record", "metadata", "identity", "authorize", "inventory"])
def test_plan_scope_and_identity_mutation_rejected(monkeypatch: pytest.MonkeyPatch, mutation: str) -> None:
    plan = synthetic_plan(monkeypatch)
    closure = plan["minimal_read_proposal.json"]["closures"][0]
    if mutation == "window":
        closure["search_window_log_time_ns_inclusive"][1] += 1
    elif mutation == "legacy":
        closure["closure_kind"] = "full_prefix"
    elif mutation == "record":
        closure["required_candidates"].append(deepcopy(closure["required_candidates"][0]))
    elif mutation == "metadata":
        closure["universe"]["source"]["metadata_sha256_reported"] = "0" * 64
    elif mutation == "identity":
        plan["execution_manifest.json"]["logical_plan_identity"] = "0" * 64
    elif mutation == "authorize":
        plan["approval_request.json"]["stages"]["S1"]["authorized"] = True
    else:
        plan["input_manifest.json"] = {"files": []}
    with pytest.raises(s1.S1Error, match="identity|inventory") as error:
        s1.validate_s1_contract(plan)
    assert error.value.status == "BLOCKED_PLAN"


def test_no_external_io_or_forbidden_import(monkeypatch: pytest.MonkeyPatch) -> None:
    source = fixture()
    tree = ast.parse(inspect.getsource(s1))
    imports = {node.module.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    imports |= {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    assert imports <= {"__future__", "dataclasses", "hashlib", "io", "json", "struct", "time", "typing", "zlib"}
    def forbidden(*args: object, **kwargs: object) -> None:
        raise AssertionError("external I/O forbidden")
    with monkeypatch.context() as patch:
        patch.setattr(builtins, "open", forbidden)
        patch.setattr(io, "open", forbidden)
        patch.setattr(socket, "socket", forbidden)
        result = run(source)
    assert result["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"


def test_active_ledger_not_stolen() -> None:
    ledger = s1.Ledger()
    ledger.begin(contract())
    started = ledger._started
    assert run(ledger=ledger)["status"] == "UNKNOWN_ACCOUNTING"
    assert ledger._started == started
    ledger.finish()


@pytest.mark.parametrize("failure", ["exception", "partial", "timeout"])
def test_late_failure_preserves_parsed_diagnostics(failure: str) -> None:
    source = fixture()
    now = [0.0]
    ledger = s1.Ledger(clock=lambda: now[0])
    original = source.read_at
    index_start = source.forbidden[0][1]
    def interrupted(offset: int, length: int) -> bytes:
        data = original(offset, length)
        if offset == index_start + 9:
            if failure == "exception":
                raise OSError("synthetic late failure")
            if failure == "partial":
                return data[:2]
            now[0] += 61
        return data
    source.read_at = interrupted
    result = run(source, ledger=ledger)
    assert result["status"] == {"exception": "UNKNOWN_ACCOUNTING", "partial": "PARTIAL_READ", "timeout": "PARTIAL_BUDGET"}[failure]
    assert result["schemas"] and result["channels"] and result["selected_chunk_descriptors"]
    expected_bytes = sum(n for _, n in source.requests[:-1])
    if failure == "partial":
        expected_bytes += 2
    elif failure == "timeout":
        expected_bytes += source.requests[-1][1]
    assert ledger.returned_source_bytes == expected_bytes
    assert_closed(result)


@pytest.mark.parametrize("failure", ["summary_offset", "entry_time", "entry_offset", "entry_length", "index_missing"])
def test_index_and_summary_field_corruption(failure: str) -> None:
    source = fixture()
    data = bytearray(source._stream.getvalue())
    index_start = source.forbidden[0][1]
    if failure == "summary_offset":
        summary_offset = struct.unpack_from("<Q", data, len(data) - 20)[0]
        struct.pack_into("<Q", data, summary_offset + 10, s1.U64)
    elif failure == "entry_time":
        struct.pack_into("<Q", data, index_start + 15, s1.U64)
    elif failure == "entry_offset":
        struct.pack_into("<Q", data, index_start + 23, s1.U64)
    elif failure == "entry_length":
        struct.pack_into("<I", data, index_start + 11, 9999)
    else:
        data[index_start] = 5  # Message opcode is rejected before its body is read.
    result = run(GuardedSource(bytes(data), source.forbidden, source.allowed_ranges))
    assert result["status"] in {"INVALID_S1_STRUCTURE", "UNSUPPORTED_S1_STRUCTURE"}
    assert_closed(result)


def test_direct_reader_limits_and_forbidden_region() -> None:
    source, ledger = fixture(), s1.Ledger()
    reader = s1.Reader(source, ledger, contract(), layout_for(source))
    reader.forbidden = source.forbidden
    with pytest.raises(s1.S1Error) as error:
        reader.read(source.forbidden[0][0], 1)
    assert error.value.status == "BLOCKED_READ_RANGE"
    with pytest.raises(s1.S1Error):
        reader.read(0, -1)
    assert source.requests == []


def test_envelope_cannot_be_increased_or_rebound() -> None:
    ledger = s1.Ledger()
    ledger.limits["source_bytes"] += 1
    assert run(ledger=ledger)["status"] == "BLOCKED_CONTRACT"
    ledger = s1.Ledger()
    assert run(ledger=ledger)["status"] == "S1_SYNTHETIC_INDEX_INSPECTED"
    before = ledger.returned_source_bytes
    assert run(ledger=ledger, c=replace(contract(), source_locator="other"))["status"] == "BLOCKED_CONTRACT"
    assert ledger.returned_source_bytes == before


def inspect_with(source: GuardedSource, layout: s1.SyntheticReadLayout | None,
                 c: s1.Contract | None = None) -> dict:
    return s1.inspect_s1_index(source, metadata(), c or contract(), s1.Ledger(), layout)


def test_layout_required_without_read_calls() -> None:
    source = fixture()
    result = inspect_with(source, None)
    assert result["status"] == "BLOCKED_READ_LAYOUT"
    assert source.requests == []
    assert result["synthetic_io"]["read_calls"] == 0
    assert_closed(result)


@pytest.mark.parametrize("inside,crc_match", [(False, False), (True, False), (False, True), (True, True)])
def test_forged_footer_blocked_in_core_before_source(inside: bool, crc_match: bool) -> None:
    source = fixture()
    layout = layout_for(source)  # Freeze BEFORE corrupting serialized declarations.
    data = bytearray(source._stream.getvalue())
    ss = source.forbidden[0][0] + (60 if inside else 0)
    footer_pos = len(data) - 37
    prefix = number("QQ", ss, 0)
    crc = zlib.crc32(data[ss:footer_pos] + number("BQ", 2, 20) + prefix) & 0xffffffff if crc_match else 0
    data[footer_pos:] = record(2, prefix + number("I", crc)) + s1.MAGIC
    source._stream = io.BytesIO(bytes(data))
    result = inspect_with(source, layout)
    assert result["status"] == "BLOCKED_READ_RANGE"
    assert result["synthetic_io"]["core_pre_read_range_rejections"] == 1
    assert result["synthetic_io"]["rejected_ranges"] == [{"offset_bytes": ss, "length_bytes": footer_pos - ss}]
    assert source.sentinel_rejections == 0
    assert not any(start == ss for start, _ in source.requests)
    assert result["synthetic_io"]["returned_source_bytes"] == sum(n for _, n in source.requests)
    assert result["synthetic_io"]["read_calls"] == len(source.requests)
    assert result["synthetic_io"]["accounting_known"]
    assert_closed(result)


def test_forged_header_length_blocked_before_source() -> None:
    source = fixture()
    layout = layout_for(source)
    data = bytearray(source._stream.getvalue())
    old_length = struct.unpack_from("<Q", data, 9)[0]
    struct.pack_into("<Q", data, 9, old_length + 32)
    source._stream = io.BytesIO(bytes(data))
    result = inspect_with(source, layout)
    assert result["status"] == "BLOCKED_READ_RANGE"
    assert (17, old_length + 32) not in source.requests
    assert source.sentinel_rejections == 0
    assert result["synthetic_io"]["core_pre_read_range_rejections"] == 1


def test_forged_message_index_unallowed_dataend_region() -> None:
    source = fixture()
    layout = layout_for(source)
    data = bytearray(source._stream.getvalue())
    ss = struct.unpack_from("<Q", data, len(data) - 28)[0]
    descriptor = ss + len(schema()) + len(channel()) + 9
    index_start = source.forbidden[0][1]
    index_length = 9 + struct.unpack_from("<Q", data, index_start + 1)[0]
    dataend_start = index_start + index_length
    # Forge mutually consistent Chunk Index claims pointing at the unallowed DataEnd.
    struct.pack_into("<Q", data, descriptor + 24, dataend_start - source.forbidden[0][0])
    struct.pack_into("<Q", data, descriptor + 38, dataend_start)
    struct.pack_into("<Q", data, descriptor + 46, 13)
    source._stream = io.BytesIO(bytes(data))
    result = inspect_with(source, layout)
    assert result["status"] == "BLOCKED_READ_RANGE"
    assert (dataend_start, 9) not in source.requests
    assert source.sentinel_rejections == 0
    assert result["schemas"] and result["selected_chunk_descriptors"]
    assert result["synthetic_io"]["core_pre_read_range_rejections"] == 1


@pytest.mark.parametrize("ranges", [((-1, 1),), ((False, 1),), ((0, True),), ((2, 1),), ((1, 1),),
                                     ((0, s1.U64 + 1),), ((0, s1.U64),), ((0, 10), (9, 11)),
                                     ((10, 11), (0, 1)), (), ([0, 1],)])
def test_invalid_layout_ranges_no_source_calls(ranges: tuple) -> None:
    source = fixture()
    result = inspect_with(source, replace(layout_for(source), allowed_ranges=ranges))
    assert result["status"] == "BLOCKED_READ_LAYOUT"
    assert source.requests == []


@pytest.mark.parametrize("kind", ["source", "contract", "probe", "size", "revision", "synthetic", "provenance"])
def test_layout_binding_no_source_calls(kind: str) -> None:
    source = fixture()
    layout = layout_for(source)
    if kind == "source":
        layout = replace(layout, source=fixture())  # Same size/revision, different object.
    elif kind == "contract":
        layout = replace(layout, contract=replace(contract(), plan_identity="different"))
    elif kind == "probe":
        layout = replace(layout, contract=replace(contract(), probes=contract().probes[::-1]))
    elif kind == "size":
        layout = replace(layout, source_size=source.size + 1)
    elif kind == "revision":
        layout = replace(layout, source_revision=1)
    elif kind == "synthetic":
        layout = replace(layout, synthetic_only=False)
    else:
        layout = replace(layout, provenance="")
    result = inspect_with(source, layout)
    assert result["status"].startswith("BLOCKED_READ_LAYOUT")
    assert source.requests == []


@pytest.mark.parametrize("offset,length", [(0, 4), (-1, 1), (False, 1), (0, True), (0, -1),
                                          (s1.U64, 1), (0, s1.U64 + 1)])
def test_candidate_range_gap_or_invalid_rejected_before_call(offset: int, length: int) -> None:
    source, ledger = fixture(), s1.Ledger()
    layout = replace(layout_for(source), allowed_ranges=((0, 2), (3, 8)))
    reader = s1.Reader(source, ledger, contract(), layout)
    with pytest.raises(s1.S1Error) as error:
        reader.read(offset, length)
    assert error.value.status == "BLOCKED_READ_RANGE"
    assert source.requests == [] and ledger.read_calls == ledger.returned_source_bytes == 0
    assert len(ledger.range_rejections) == 1


def test_adjacent_ranges_and_source_side_guard_are_distinct() -> None:
    source, ledger = fixture(), s1.Ledger()
    layout = replace(layout_for(source), allowed_ranges=((0, 4), (4, 8)))
    reader = s1.Reader(source, ledger, contract(), layout)
    assert reader.read(0, 8) == s1.MAGIC
    assert ledger.read_calls == 1 and not ledger.range_rejections
    assert source.sentinel_rejections == 0
    # Test-only direct call proves the source guard would log and reject issuance.
    with pytest.raises(AssertionError, match="sentinel"):
        source.read_at(source.forbidden[0][0], 1)
    assert source.sentinel_rejections == 1 and len(source.requests) == 2
    assert ledger.read_calls == 1  # Deliberate test call is not a core read.
