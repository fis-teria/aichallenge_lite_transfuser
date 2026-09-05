"""Read-only-output package: fixed source, raw logs/numerical evidence, no weights."""
import argparse
import ast
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import zipfile

from package_spatial_diagnostic_v4 import sha, json_bytes, safe_name, ordinary_file, verify_zip, MAX_EXPANDED


def package(repo: Path, run: Path, request: Path, report: Path, review: Path, output: Path, base: str) -> dict:
    if output.exists():
        raise FileExistsError("immutable ZIP exists")
    execution = json.loads(ordinary_file(run / "artifacts/execution_manifest.json"))
    commit = execution["execution_commit"]
    def git(*args: str) -> bytes:
        return subprocess.check_output(["git", *args], cwd=repo)
    report_commit = git("rev-parse", "HEAD").decode().strip()
    files = {}
    def add(name: str, blob: bytes, role: str, source: str = commit) -> None:
        safe_name(name)
        if name in files or PurePosixPath(name).suffix in (".pt", ".pth", ".mcap", ".bag", ".onnx"):
            raise ValueError("forbidden/duplicate entry")
        files[name] = (blob, role, source)
    for category in ("artifacts", "evidence", "logs", "provenance", "figures"):
        for path in sorted((run / category).rglob("*")):
            if path.is_symlink():
                raise ValueError("symlink in outputs")
            if path.is_file():
                if path.suffix not in (".json", ".yaml", ".npy", ".npz", ".csv", ".log", ".xml", ".png", ".jsonl"):
                    raise ValueError("unexpected output")
                add(path.relative_to(run).as_posix(), ordinary_file(path), "evaluation evidence")
    changed = git("diff", "--name-only", base, commit).decode().splitlines()
    tracked = set(git("ls-tree", "-r", "--name-only", commit).decode().splitlines())
    seeds = set(changed) | {"AGENTS.md", "pyproject.toml", "tests/test_spatial_diagnostic_geometry_v4.py",
        "tools/package_spatial_diagnostic_v4.py", "src/aic_transfuser_lite/training/spatial_diagnostic_v4.py"}
    todo, seen = list(seeds & tracked), set()
    while todo:
        name = todo.pop()
        if name in seen:
            continue
        seen.add(name)
        blob = git("show", commit + ":" + name)
        add("repo/" + name, blob, "fixed source; historical trainer is static eligibility reference ONLY")
        if name.endswith(".py"):
            modules = set(re.findall(r"(?:from|import)\s+(aic_transfuser_lite[\w.]*)", blob.decode()))
            for node in ast.walk(ast.parse(blob)):
                if isinstance(node, ast.ImportFrom) and node.level and name.startswith("src/"):
                    parent = list(PurePosixPath(name).parent.parts[1:])
                    modules.add(".".join(parent[:len(parent)-node.level+1] + ([node.module] if node.module else [])))
            for module in modules:
                path = "src/"+module.replace(".", "/")
                deps = [path+".py", path+"/__init__.py"] + [str(p/"__init__.py") for p in PurePosixPath(path).parents]
                todo.extend(d for d in deps if d in tracked and d not in seen)
    for name, expected in execution.get("code_hashes", {}).items():
        if "repo/"+name in files and sha(files["repo/"+name][0]) != expected:
            raise ValueError("executed source mismatch")
    add("provenance/git_diff.patch", git("diff", "--binary", base, commit), "implementation diff")
    add("provenance/changed_files.json", json_bytes({"base": base, "execution_commit": commit, "report_commit": report_commit,
        "packaging_commit": report_commit, "files": changed, "push": "NOT_EXECUTED"}), "revision split", report_commit)
    add("reports/limited_validation_report_ja.md", ordinary_file(report), "results report", report_commit)
    add("request/implementation_request.md", ordinary_file(request), "user request", "user attachment")
    add("request/independent_review_request.md", ordinary_file(review), "next independent review request", report_commit)
    required = ["logs/tests_stdout.log", "logs/tests_stderr.log", "logs/junit.xml", "logs/evaluation_stdout.log", "logs/evaluation_stderr.log"]
    if execution["status"] == "EVALUATION_COMPLETE":
        required += ["evidence/train_predictions.npz", "evidence/validation_predictions.npz", "evidence/validation_targets.npz",
                     "evidence/validation_baselines.npz", "provenance/state_before.json", "provenance/state_after.json"]
    if any(name not in files for name in required):
        raise ValueError("missing essential evidence")
    readme = f"# Fixed step500 limited validation\n\nExecution `{commit}`; report `{report_commit}`; run `{run.name}`.\n\n"
    readme += "[Report](reports/limited_validation_report_ja.md) | [Review request](request/independent_review_request.md) | [Execution manifest](artifacts/execution_manifest.json) | [Metrics](artifacts/metrics.json)\n\n"
    readme += "AGENTS, historical requests, trainer source and commands are reference material, NOT execution authorization. No optimizer/training permitted.\n\n"
    readme += "NPY/NPZ must be read with allow_pickle=False. Each entry is hashed; PACKAGE_MANIFEST excludes itself and is separate from execution provenance.\n\n"
    readme += "Omitted: weights, original sensors, root Dataset manifest, unselected Dataset/raw. No independent re-inference without weights; input tensors alone do not re-establish preprocessing from original sensor bytes. Source correctness/physical safety/permission/controller remain UNKNOWN.\n"
    add("README_REVIEW.md", readme.encode(), "entry point", report_commit)
    manifest = {"self_hash": "excluded", "files": [{"path": name, "size_bytes": len(blob), "sha256": sha(blob), "role": role, "source_commit": source}
        for name, (blob, role, source) in sorted(files.items())]}
    add("PACKAGE_MANIFEST.json", json_bytes(manifest), "package-only inventory", report_commit)
    if sum(len(v[0]) for v in files.values()) > MAX_EXPANDED:
        raise ValueError("BLOCKED_PACKAGE: expanded limit; no evidence removed")
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "x", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, (blob, _, _) in sorted(files.items()):
            archive.writestr(output.stem+"/"+name, blob)
    return verify_zip(output, output.parent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("repo", "run", "request", "report", "review", "output"):
        parser.add_argument("--"+key, required=True, type=Path)
    parser.add_argument("--base", required=True)
    print(json.dumps(package(**vars(parser.parse_args())), indent=2))
