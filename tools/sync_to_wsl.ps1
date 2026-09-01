[CmdletBinding()]
param(
    [string]$SshHost = "codex-wsl",
    [string]$WslRepository = "/home/thistle/e2e_autonomous/e2e_lite_transfuser",
    [string]$WindowsRepositoryInWsl = "/mnt/e/workspace/e2e_lite_transfuser",
    [switch]$CheckOnly
)

$ErrorActionPreference = "Stop"

if ($SshHost -notmatch "^[A-Za-z0-9._-]+$") {
    throw "SshHost contains unsupported characters: $SshHost"
}
foreach ($pathValue in @($WslRepository, $WindowsRepositoryInWsl)) {
    if ($pathValue -notmatch "^/[A-Za-z0-9._/-]+$") {
        throw "Repository paths must not contain spaces or shell metacharacters: $pathValue"
    }
}

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$gitTopLevel = (& git -C $repositoryRoot rev-parse --show-toplevel 2>$null).Trim()
if ($LASTEXITCODE -ne 0 -or -not $gitTopLevel) {
    throw "Windows repository is not a Git worktree: $repositoryRoot"
}

$localStatus = @(& git -C $repositoryRoot status --porcelain=v1 --untracked-files=all)
if ($LASTEXITCODE -ne 0) {
    throw "Failed to read the Windows repository status."
}
if ($localStatus.Count -ne 0) {
    $details = $localStatus -join [Environment]::NewLine
    throw "Commit or remove all Windows-side changes before syncing. Uncommitted files are never copied to WSL:`n$details"
}

$branch = (& git -C $repositoryRoot branch --show-current).Trim()
if ($LASTEXITCODE -ne 0 -or -not $branch) {
    throw "The Windows checkout must be on a named branch before syncing."
}
& git check-ref-format --branch $branch *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Invalid Windows branch name: $branch"
}
if ($branch -notmatch "^[A-Za-z0-9._/-]+$") {
    throw "Branch name contains characters that are unsafe for the WSL transport: $branch"
}

$expectedSha = (& git -C $repositoryRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $expectedSha -notmatch "^[0-9a-f]{40}$") {
    throw "Failed to resolve the Windows HEAD commit."
}

$mode = if ($CheckOnly) { "check" } else { "sync" }
$remoteCommand = "bash -s -- $WslRepository $WindowsRepositoryInWsl $branch $expectedSha $mode"

$remoteScript = @'
set -euo pipefail

wsl_repository=$1
windows_repository=$2
source_branch=$3
expected_sha=$4
mode=$5

fail() {
    printf 'SYNC_ERROR: %s\n' "$1" >&2
    exit 1
}

test -d "$wsl_repository/.git" || fail "WSL repository is missing: $wsl_repository"
test -d "$windows_repository/.git" || fail "Windows repository is not visible in WSL: $windows_repository"
command -v flock >/dev/null 2>&1 || fail "flock is required for synchronized WSL worktree access."

exec 9>"$wsl_repository/.git/codex-wsl-worktree.lock"
flock -n 9 || fail "WSL training or validation currently holds the worktree lock."

check_active_processes() {
    active_processes=$(ps -eo pid=,comm=,args= | awk '
        $2 ~ /^(python([0-9.]*)?|torchrun|accelerate|deepspeed|pytest|colcon|ros2)$/ &&
        $0 ~ /(e2e_lite_transfuser|training\.train|train(_v1)?\.py|torchrun|pytest|colcon|ros2)/ { print }
    ')
    if test -n "$active_processes"; then
        printf '%s\n' "$active_processes" >&2
        fail "A WSL training, validation, or ROS process may be using the checkout."
    fi
}

check_active_processes

dirty=$(git -C "$wsl_repository" status --porcelain=v1 --untracked-files=all)
if test -n "$dirty"; then
    printf '%s\n' "$dirty" >&2
    fail "WSL contains uncommitted non-ignored changes. Nothing was changed."
fi

test -x "$wsl_repository/.venv/bin/python" || fail "WSL virtual environment is missing."
test -d "$wsl_repository/datasets/processed/aic_real_dataset_v2" || fail "Dataset v2 is missing."

current_sha=$(git -C "$wsl_repository" rev-parse HEAD)
printf 'WINDOWS_BRANCH=%s\n' "$source_branch"
printf 'WINDOWS_HEAD=%s\n' "$expected_sha"
printf 'WSL_HEAD_BEFORE=%s\n' "$current_sha"

if test "$mode" = check; then
    if test "$current_sha" = "$expected_sha"; then
        printf 'SYNC_REQUIRED=no\n'
    else
        printf 'SYNC_REQUIRED=yes\n'
    fi
    printf 'CHECK_OK\n'
    exit 0
fi

git -C "$wsl_repository" fetch --no-tags "$windows_repository" "$source_branch"
fetched_sha=$(git -C "$wsl_repository" rev-parse FETCH_HEAD)
test "$fetched_sha" = "$expected_sha" || fail "Fetched commit does not match Windows HEAD."

protected_paths=$(git -C "$wsl_repository" ls-tree -r --name-only "$fetched_sha" | awk '
    /(^|\/)(\.venv|runs|checkpoints|weights)(\/|$)/ ||
    /^datasets\/(raw|processed)(\/|$)/ ||
    /^sample_train_data_[^/]*(\/|$)/ ||
    /^ros2_ws\/(build|install|log)(\/|$)/ ||
    /^docs\/transfuser_lite_v1_execution_state\.md$/ ||
    /\.(pt|pth|ckpt|safetensors|onnx|bag|mcap|db3|zstd)$/ { print }
')
if test -n "$protected_paths"; then
    printf '%s\n' "$protected_paths" >&2
    fail "The target commit tracks protected WSL training assets. Nothing was checked out."
fi

check_active_processes
git -C "$wsl_repository" switch --detach --no-overwrite-ignore "$fetched_sha"

synced_sha=$(git -C "$wsl_repository" rev-parse HEAD)
test "$synced_sha" = "$expected_sha" || fail "WSL HEAD does not match Windows HEAD after checkout."
test -z "$(git -C "$wsl_repository" status --porcelain=v1 --untracked-files=all)" || fail "WSL worktree is not clean after checkout."
test -x "$wsl_repository/.venv/bin/python" || fail "WSL virtual environment disappeared during sync."
test -d "$wsl_repository/datasets/processed/aic_real_dataset_v2" || fail "Dataset v2 disappeared during sync."

printf 'WSL_HEAD_AFTER=%s\n' "$synced_sha"
printf 'SYNC_OK\n'
'@

$remoteScript | & ssh -o BatchMode=yes -o ConnectTimeout=10 $SshHost $remoteCommand
if ($LASTEXITCODE -ne 0) {
    throw "WSL $mode failed with exit code $LASTEXITCODE."
}
