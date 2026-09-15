"""Use the unchanged sync_to_wsl.ps1 remote script over native WSL transport."""
import argparse
import hashlib
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("mode", choices=("check", "sync"))
args = parser.parse_args()
repo = Path(r"E:\workspace\e2e_lite_transfuser")
def git(*argv: str) -> str:
    return subprocess.check_output(["git", "-C", str(repo), *argv], text=True).strip()

if git("status", "--porcelain=v1", "--untracked-files=all"):
    raise RuntimeError("Windows worktree is not clean")
branch = git("branch", "--show-current")
if not branch:
    raise RuntimeError("Windows must be on a named branch")
git("check-ref-format", "--branch", branch)
head = git("rev-parse", "HEAD")
source = (repo / "tools/sync_to_wsl.ps1").read_text(encoding="utf-8-sig")
script = source.split("$remoteScript = @'\n", 1)[1].split("\n'@", 1)[0] + "\n"
print("SYNC_SCRIPT_SHA256=" + hashlib.sha256(script.encode()).hexdigest(), flush=True)
subprocess.run([
    "wsl.exe", "-d", "Ubuntu-22.04-Recovered", "-u", "thistle", "--exec", "bash", "-s", "--",
    "/home/thistle/e2e_autonomous/e2e_lite_transfuser", "/mnt/e/workspace/e2e_lite_transfuser",
    branch, head, args.mode,
], input=script.encode("utf-8"), check=True)
