#!/usr/bin/env python3
"""Prepare a verified AWSIM Camera publish-rate compatibility override.

The supported AI Challenge AWSIM build predates upstream commit 4dc00f7,
which removed a trailing ``WaitForFixedUpdate`` from
``CameraSensorHolder.FixedUpdateRoutine``.  This tool applies the equivalent
change to a new assembly copy.  It never edits or replaces the source file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Sequence


EXPECTED_INPUT_SHA256 = (
    "f8553d26dadc1316143ee22f6d9a75803753ff82433876da78f48f808d49ba28"
)
EXPECTED_OUTPUT_SHA256 = (
    "129192b7ab8783d7092bca92ae8c220b95055d350468c3cd2c513d701b19cb95"
)
PATCH_OFFSET = 0x1178E
EXPECTED_YIELD_BLOCK = bytes.fromhex(
    "02"          # ldarg.0
    "738002000a"  # newobj WaitForFixedUpdate::.ctor
    "7d0c030004"  # stfld <>2__current
    "02"          # ldarg.0
    "19"          # ldc.i4.3
    "7d0b030004"  # stfld <>1__state
    "17"          # ldc.i4.1
    "2a"          # ret
)
REPLACEMENT_NOPS = bytes(len(EXPECTED_YIELD_BLOCK))
UPSTREAM_COMMIT = (
    "https://github.com/autowarefoundation/AWSIM/commit/"
    "4dc00f768b84e1ebaf8d8b6372f5065e0a26622f"
)


def sha256_bytes(data: bytes) -> str:
    """Return the lowercase SHA-256 digest for *data*."""

    return hashlib.sha256(data).hexdigest()


def patch_assembly_bytes(original: bytes) -> tuple[bytes, dict[str, object]]:
    """Return a verified patched copy and deterministic provenance manifest.

    The input must be the one audited assembly build.  The function rejects a
    different build even when a similar byte sequence occurs at the same
    offset.  The 20-byte compiler-generated yield block is replaced with IL
    NOPs; all other bytes remain identical.
    """

    input_sha256 = sha256_bytes(original)
    if input_sha256 != EXPECTED_INPUT_SHA256:
        raise ValueError(
            "unsupported AWSIM assembly SHA-256: "
            f"expected={EXPECTED_INPUT_SHA256}, actual={input_sha256}"
        )
    stop = PATCH_OFFSET + len(EXPECTED_YIELD_BLOCK)
    actual_block = original[PATCH_OFFSET:stop]
    if actual_block != EXPECTED_YIELD_BLOCK:
        raise ValueError(
            f"unexpected yield block at 0x{PATCH_OFFSET:X}: "
            f"expected={EXPECTED_YIELD_BLOCK.hex()}, actual={actual_block.hex()}"
        )

    patched_buffer = bytearray(original)
    patched_buffer[PATCH_OFFSET:stop] = REPLACEMENT_NOPS
    patched = bytes(patched_buffer)
    changed_offsets = [
        index
        for index, (before, after) in enumerate(zip(original, patched, strict=True))
        if before != after
    ]
    expected_changed_offsets = [
        PATCH_OFFSET + relative_offset
        for relative_offset, value in enumerate(EXPECTED_YIELD_BLOCK)
        if value != 0
    ]
    if changed_offsets != expected_changed_offsets:
        raise AssertionError(f"unexpected changed offsets: {changed_offsets}")

    output_sha256 = sha256_bytes(patched)
    if output_sha256 != EXPECTED_OUTPUT_SHA256:
        raise AssertionError(
            "patched assembly SHA-256 mismatch: "
            f"expected={EXPECTED_OUTPUT_SHA256}, actual={output_sha256}"
        )
    manifest: dict[str, object] = {
        "schema_version": "awsim_camera_hz_override_v1",
        "upstream_commit": UPSTREAM_COMMIT,
        "semantic_change": "remove trailing WaitForFixedUpdate yield",
        "input_sha256": input_sha256,
        "output_sha256": output_sha256,
        "patch_offset_decimal": PATCH_OFFSET,
        "patch_offset_hex": f"0x{PATCH_OFFSET:X}",
        "block_size_bytes": len(EXPECTED_YIELD_BLOCK),
        "expected_yield_block_hex": EXPECTED_YIELD_BLOCK.hex(),
        "replacement_nops_hex": REPLACEMENT_NOPS.hex(),
        "changed_offsets": changed_offsets,
    }
    return patched, manifest


def _write_exclusive(path: Path, payload: bytes) -> None:
    with path.open("xb") as handle:
        handle.write(payload)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a hash-verified AWSIM Assembly-CSharp.dll copy carrying "
            "the official Camera topic-Hz scheduler fix."
        )
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    manifest_path = args.manifest.expanduser().resolve()
    if len({input_path, output_path, manifest_path}) != 3:
        raise ValueError("input, output, and manifest paths must be distinct")
    if not input_path.is_file():
        raise FileNotFoundError(f"input assembly does not exist: {input_path}")
    for destination in (output_path, manifest_path):
        if destination.exists():
            raise FileExistsError(f"refusing existing output: {destination}")
        if not destination.parent.is_dir():
            raise FileNotFoundError(
                f"output parent directory does not exist: {destination.parent}"
            )

    patched, manifest = patch_assembly_bytes(input_path.read_bytes())
    manifest.update(
        {
            "input": str(input_path),
            "output": str(output_path),
            "manifest": str(manifest_path),
        }
    )
    manifest_bytes = (
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")

    output_written = False
    manifest_written = False
    try:
        _write_exclusive(output_path, patched)
        output_written = True
        _write_exclusive(manifest_path, manifest_bytes)
        manifest_written = True
        if output_path.read_bytes() != patched:
            raise OSError("written assembly does not match verified patched bytes")
        if manifest_path.read_bytes() != manifest_bytes:
            raise OSError("written manifest does not match generated manifest")
        output_path.chmod(0o444)
        manifest_path.chmod(0o444)
    except BaseException:
        if manifest_written and manifest_path.is_file():
            current_manifest = manifest_path.read_bytes()
            if current_manifest == manifest_bytes:
                manifest_path.unlink()
        if output_written and output_path.is_file():
            current = output_path.read_bytes()
            if current == patched:
                output_path.unlink()
        raise

    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
