from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools import prepare_awsim_camera_hz_override as camera_hz


def _supported_fixture(monkeypatch: pytest.MonkeyPatch) -> tuple[bytes, bytes]:
    original = bytearray(camera_hz.PATCH_OFFSET + 64)
    stop = camera_hz.PATCH_OFFSET + len(camera_hz.EXPECTED_YIELD_BLOCK)
    original[camera_hz.PATCH_OFFSET:stop] = camera_hz.EXPECTED_YIELD_BLOCK
    patched = bytearray(original)
    patched[camera_hz.PATCH_OFFSET:stop] = camera_hz.REPLACEMENT_NOPS
    original_bytes = bytes(original)
    patched_bytes = bytes(patched)
    monkeypatch.setattr(
        camera_hz, "EXPECTED_INPUT_SHA256", camera_hz.sha256_bytes(original_bytes)
    )
    monkeypatch.setattr(
        camera_hz, "EXPECTED_OUTPUT_SHA256", camera_hz.sha256_bytes(patched_bytes)
    )
    return original_bytes, patched_bytes


def test_patch_changes_only_the_verified_trailing_yield(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original, expected = _supported_fixture(monkeypatch)

    patched, manifest = camera_hz.patch_assembly_bytes(original)

    assert patched == expected
    assert len(patched) == len(original)
    assert manifest["semantic_change"] == "remove trailing WaitForFixedUpdate yield"
    assert manifest["patch_offset_hex"] == "0x1178E"
    assert manifest["output_sha256"] == camera_hz.sha256_bytes(expected)
    assert manifest["changed_offsets"] == [
        camera_hz.PATCH_OFFSET + offset
        for offset, value in enumerate(camera_hz.EXPECTED_YIELD_BLOCK)
        if value != 0
    ]


def test_patch_rejects_an_unknown_assembly() -> None:
    with pytest.raises(ValueError, match="unsupported AWSIM assembly SHA-256"):
        camera_hz.patch_assembly_bytes(b"not the audited AWSIM assembly")


def test_patch_rejects_unexpected_bytes_even_with_matching_hash(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original = bytes(camera_hz.PATCH_OFFSET + 64)
    monkeypatch.setattr(
        camera_hz, "EXPECTED_INPUT_SHA256", camera_hz.sha256_bytes(original)
    )
    with pytest.raises(ValueError, match="unexpected yield block"):
        camera_hz.patch_assembly_bytes(original)


def test_cli_writes_new_read_only_outputs_and_refuses_overwrite(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    original, expected = _supported_fixture(monkeypatch)
    input_path = tmp_path / "Assembly-CSharp.dll"
    output_path = tmp_path / "Assembly-CSharp.camera-hz.dll"
    manifest_path = tmp_path / "Assembly-CSharp.camera-hz.json"
    input_path.write_bytes(original)
    argv = [
        "--input",
        str(input_path),
        "--output",
        str(output_path),
        "--manifest",
        str(manifest_path),
    ]

    assert camera_hz.main(argv) == 0
    assert output_path.read_bytes() == expected
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["input"] == str(input_path.resolve())
    assert manifest["output"] == str(output_path.resolve())
    assert output_path.stat().st_mode & 0o222 == 0
    assert manifest_path.stat().st_mode & 0o222 == 0

    with pytest.raises(FileExistsError, match="refusing existing output"):
        camera_hz.main(argv)


def test_cli_forbids_in_place_output(tmp_path: Path) -> None:
    assembly = tmp_path / "Assembly-CSharp.dll"
    assembly.write_bytes(b"anything")
    with pytest.raises(ValueError, match="paths must be distinct"):
        camera_hz.main(
            [
                "--input",
                str(assembly),
                "--output",
                str(assembly),
                "--manifest",
                str(tmp_path / "manifest.json"),
            ]
        )
