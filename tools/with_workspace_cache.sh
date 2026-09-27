#!/usr/bin/env bash
set -euo pipefail

if (($# == 0)); then
    printf 'Usage: %s <command> [args...]\n' "$0" >&2
    exit 2
fi

script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace_root=$(git -C "$script_directory/.." rev-parse --show-toplevel)
export XDG_CACHE_HOME="$workspace_root/.cache"
export TORCH_HOME="$workspace_root/.cache/torch"
export HF_HOME="$workspace_root/.cache/huggingface"
export PIP_CACHE_DIR="$workspace_root/.cache/pip"
export TORCHINDUCTOR_CACHE_DIR="$workspace_root/.cache/torchinductor"
export TRITON_CACHE_DIR="$workspace_root/.cache/triton"
export CUDA_CACHE_PATH="$workspace_root/.cache/cuda"
cd "$workspace_root"
exec "$@"
