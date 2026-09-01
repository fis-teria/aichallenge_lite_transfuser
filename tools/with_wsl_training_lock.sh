#!/usr/bin/env bash
set -euo pipefail

if (($# == 0)); then
    printf 'Usage: %s <training-or-validation-command> [args...]\n' "$0" >&2
    exit 2
fi

repository_root=$(git rev-parse --show-toplevel)
lock_file="$repository_root/.git/codex-wsl-worktree.lock"

command -v flock >/dev/null 2>&1 || {
    printf 'flock is required for synchronized WSL worktree access.\n' >&2
    exit 1
}

exec 9>"$lock_file"
if ! flock -n 9; then
    printf 'The WSL worktree is being synchronized. Try again after sync completes.\n' >&2
    exit 1
fi

exec "$@"
