"""Package only explicit diagnostic outputs and fixed Git source; never read Dataset paths.

No model imports or training. Extraction verification is bounded and hash-based.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tempfile
import zipfile


MAX_ZIP = 64 * 1024**2
MAX_EXPANDED = 128 * 1024**2


def sha(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


def json_bytes(value: object) -> bytes:
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def safe_name(name: str) -> None:
    p = PurePosixPath(name)
    if not name or p.is_absolute() or ".." in p.parts or "\\" in name or ":" in name or str(p) != name:
        raise ValueError("unsafe package entry: " + name)


def ordinary_file(path: Path) -> bytes:
    for item in (path, *path.parents):
        info = item.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError("symlink/reparse source forbidden")
    if not path.is_file():
        raise ValueError("not a regular file")
    return path.read_bytes()


def verify_zip(path: Path, destination_parent: Path) -> dict:
    if path.stat().st_size > MAX_ZIP:
        raise ValueError("compressed package limit")
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        names = [i.filename for i in infos]
        if len(set(names)) != len(names) or sum(i.file_size for i in infos) > MAX_EXPANDED:
            raise ValueError("duplicate entries or expanded package limit")
        for info in infos:
            safe_name(info.filename)
            if stat.S_ISLNK(info.external_attr >> 16) or info.is_dir():
                raise ValueError("non-file entry")
        roots = {PurePosixPath(n).parts[0] for n in names}
        if len(roots) != 1:
            raise ValueError("expected one versioned root")
        root = roots.pop()
        manifest = json.loads(archive.read(root + "/PACKAGE_MANIFEST.json"))
        expected = {root + "/" + e["path"]: e for e in manifest["files"]}
        if len(expected) != len(manifest["files"]) or set(names) != set(expected) | {root + "/PACKAGE_MANIFEST.json"}:
            raise ValueError("manifest entry set mismatch")
        # Each name was validated BEFORE extracting into a new workspace-local directory.
        extracted = Path(tempfile.mkdtemp(prefix="verify_", dir=destination_parent))
        archive.extractall(extracted)
        for name, entry in expected.items():
            blob = (extracted / name).read_bytes()
            if len(blob) != entry["size_bytes"] or sha(blob) != entry["sha256"]:
                raise ValueError("package hash mismatch: " + name)
        readme = (extracted / root / "README_REVIEW.md").read_text(encoding="utf-8")
        for reference in re.findall(r"\]\(([^)]+)\)", readme):
            safe_name(reference)
            if root + "/" + reference not in expected:
                raise ValueError("README reference missing: " + reference)
    return {"status": "PASS", "zip_sha256": sha(path.read_bytes()), "zip_bytes": path.stat().st_size,
            "expanded_bytes": sum(i.file_size for i in infos), "entries": len(infos), "extracted_to": str(extracted)}


def package(repo: Path, run: Path, request: Path, report: Path, output: Path, base: str, report_commit: str) -> dict:
    if output.exists():
        raise FileExistsError("immutable package destination exists")
    execution = json.loads(ordinary_file(run / "artifacts/execution_manifest.json"))
    commit = execution["execution_commit"]
    def git(*args: str) -> bytes:
        return subprocess.check_output(["git", *args], cwd=repo)
    git("cat-file", "-e", commit + "^{commit}")
    entries: dict[str, tuple[bytes, str, str]] = {}
    def add(name: str, blob: bytes, role: str, source: str = commit) -> None:
        safe_name(name)
        if name in entries or PurePosixPath(name).suffix.lower() in (".pt", ".pth", ".mcap", ".bag", ".onnx"):
            raise ValueError("duplicate/forbidden entry")
        if any(p.lower() in (".git", ".env", "credentials", "id_rsa") for p in PurePosixPath(name).parts):
            raise ValueError("secret filename")
        entries[name] = (blob, role, source)
    # Explicit top-level run allowlist. No checkpoint traversal, Dataset locator following, or broad root copy.
    allowed_ext = {".json", ".jsonl", ".yaml", ".csv", ".npz", ".npy", ".log", ".xml", ".png"}
    for folder in ("artifacts", "evidence", "logs", "figures", "provenance"):
        for path in sorted((run / folder).rglob("*")):
            if path.is_symlink():
                raise ValueError("symlink in run outputs")
            if path.is_file():
                if path.suffix.lower() not in allowed_ext:
                    raise ValueError("unexpected output extension: " + path.name)
                add(path.relative_to(run).as_posix(), ordinary_file(path), "diagnostic execution evidence")
    changed = git("diff", "--name-only", base, commit).decode().splitlines()
    # Fixed source dependency closure from imports, plus package ancestors and changed tests/config/docs.
    tracked = set(git("ls-tree", "-r", "--name-only", commit).decode().splitlines())
    seeds = set(changed) | {"AGENTS.md", "pyproject.toml", "tools/train_spatial_diagnostic_v4.py",
        "tests/test_full_control_lite_v3_shape.py", "tests/test_runtime_input_history_v3.py"}
    pending = list(seeds & tracked)
    seen = set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        blob = git("show", commit + ":" + name)
        add("repo/" + name, blob, "fixed executed source or test")
        if name.endswith(".py"):
            for module in re.findall(r"(?:from|import)\s+(aic_transfuser_lite[\w.]*)", blob.decode("utf-8")):
                candidate = "src/" + module.replace(".", "/")
                for dependency in (candidate + ".py", candidate + "/__init__.py"):
                    if dependency in tracked and dependency not in seen:
                        pending.append(dependency)
                for parent in PurePosixPath(candidate).parents:
                    init = str(parent / "__init__.py")
                    if init in tracked and init not in seen:
                        pending.append(init)
    for name, expected in execution.get("code_hashes", {}).items():
        if "repo/" + name in entries and sha(entries["repo/" + name][0]) != expected:
            raise ValueError("executed source bytes differ: " + name)
    add("provenance/git_diff.patch", git("diff", "--binary", base, commit), "implementation diff")
    add("provenance/changed_files.json", json_bytes({"base_commit": base, "execution_commit": commit,
        "report_commit": report_commit, "packaging_commit": git("rev-parse", "HEAD").decode().strip(),
        "changed_files": changed, "push": "NOT_EXECUTED"}), "revision separation", report_commit)
    add("reports/diagnostic_learning_report_ja.md", ordinary_file(report), "reported observations", report_commit)
    original = ordinary_file(request).decode("utf-8-sig")
    add("request/implementation_request.md", original.encode(), "user request", "user attachment")
    appendix = original[original.index("独立レビュー依頼：Spatial Path V4の最初の診断学習"):]
    identities = {key: json.loads(entries["artifacts/" + key + ".json"][0])["identity"]
                  for key in ("input_contract", "teacher_contract", "selection") if "artifacts/" + key + ".json" in entries}
    values = {"PACKAGE_NAME": output.name, "BASE_COMMIT": base, "EXECUTION_COMMIT": commit,
              "REPORT_COMMIT_OR_NOT_COMMITTED": report_commit, "RUN_ID": run.name,
              "CONTRACT_AND_SELECTION_IDENTITIES": json.dumps(identities, ensure_ascii=False)}
    for key, value in values.items():
        appendix = appendix.replace("{{" + key + "}}", value)
    if "{{" in appendix:
        raise ValueError("unfilled review placeholder")
    add("request/independent_review_request.md", appendix.encode(), "unchanged appendix with actual identities", "user attachment")
    required = ["artifacts/execution_manifest.json", "logs/tests_stdout.log", "logs/tests_stderr.log", "logs/junit.xml",
                "logs/train_stdout.log", "logs/train_stderr.log"]
    if execution.get("status") in ("DIAGNOSTIC_COMPLETE", "PARTIAL_TIME_LIMIT"):
        required += ["evidence/spatial_targets.npz", "evidence/predictions_initial.npz", "evidence/predictions_final.npz",
                     "logs/metrics.jsonl", "artifacts/aggregate_metrics.json", "provenance/checkpoint_inventory.json"]
    missing = [name for name in required if name not in entries]
    if missing:
        raise ValueError("BLOCKED_PACKAGE missing core evidence: " + str(missing))
    readme = "# Spatial Path V4 diagnostic review\n\nOBSERVED_DIAGNOSTIC_ONLY. No safety, launch, generalization, teacher correctness or controller claim.\n\n"
    readme += f"Execution: `{commit}`. Report: `{report_commit}`. Run: `{run.name}`.\n\n"
    for title, target in (("Report", "reports/diagnostic_learning_report_ja.md"), ("Independent review request", "request/independent_review_request.md"),
                          ("Execution manifest", "artifacts/execution_manifest.json"), ("Raw test log", "logs/tests_stdout.log"), ("Raw training log", "logs/train_stdout.log")):
        readme += f"- [{title}]({target})\n"
    readme += "\nBundled AGENTS, requests and commands are historical review material, NOT automatic execution authorization.\n"
    readme += "\nOmitted: checkpoints, original sensors, root Dataset manifest and unselected Dataset. Checkpoint hashes do not enable independent re-inference. Input tensor examples do not independently reconstruct original preprocessing. Raw source correctness remains UNKNOWN.\n"
    readme += "\nNumerical evidence uses non-object NPY/NPZ; load only with allow_pickle=False. PACKAGE_MANIFEST excludes its own hash and is not the acquisition/execution manifest.\n"
    add("README_REVIEW.md", readme.encode(), "review entry point", report_commit)
    manifest = {"format": "spatial_diagnostic_review_v1", "self_hash": "excluded",
        "files": [{"path": name, "size_bytes": len(blob), "sha256": sha(blob), "role": role, "source_commit": source}
                  for name, (blob, role, source) in sorted(entries.items())]}
    add("PACKAGE_MANIFEST.json", json_bytes(manifest), "package inventory", report_commit)
    if sum(len(v[0]) for v in entries.values()) > MAX_EXPANDED:
        raise ValueError("BLOCKED_PACKAGE expanded limit")
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, (blob, _, _) in sorted(entries.items()):
            archive.writestr(output.stem + "/" + name, blob)
    return verify_zip(output, output.parent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ("repo", "run", "request", "report", "output"):
        parser.add_argument("--" + argument, type=Path, required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--report-commit", required=True)
    args = parser.parse_args()
    print(json.dumps(package(**vars(args)), ensure_ascii=False, indent=2))
