# Windows Codex / WSL training workflow

## Roles

- Canonical source and Codex project: `E:\workspace\e2e_lite_transfuser`
- Training and Linux/ROS validation: `/home/thistle/e2e_autonomous/e2e_lite_transfuser`
- GitHub origin: `https://github.com/fis-teria/aichallenge_lite_transfuser.git`

Edit and review source on Windows. Run training from the native WSL filesystem. Do not train from `/mnt/e`, and do not copy `.venv`, datasets, runs, checkpoints, rosbag files, or ROS build outputs into the Windows checkout.

## Source-of-truth rules

1. Create a `codex/<topic>` branch in the Windows checkout.
2. Make and review a small change, run available static/unit checks, and commit it on Windows.
3. Synchronize that exact commit to WSL with `tools/sync_to_wsl.ps1`.
4. Run Linux, CUDA, dataset, training, ROS, replay, and simulator validation in WSL through `tools/with_wsl_training_lock.sh`.
5. Push only from the Windows checkout after review. Never push from WSL or another SSH/experiment host.

The synchronization command fetches a committed Windows branch through WSL's `/mnt/e` mount, then checks out the fetched commit in detached-HEAD mode. It does not add or change a WSL remote, does not use a forced reset, and does not delete ignored files. Detached HEAD makes the exact training commit explicit and prevents accidental WSL-only development from becoming the source of truth.

The command stops before changing the WSL worktree or protected training assets when:

- Windows has staged, unstaged, or untracked non-ignored files;
- Windows is in detached-HEAD state;
- WSL has staged, unstaged, or untracked non-ignored files;
- a matching Python/torchrun training process is running;
- the WSL virtual environment or Dataset v2 is missing;
- the fetched commit differs from the Windows commit.
- the target commit attempts to track protected datasets, runs, checkpoints, weights, rosbags, or ROS build outputs.

`-CheckOnly` does not fetch and therefore does not inspect the target commit's protected-path denylist. A normal sync performs that check before checkout. A failed normal sync may update Git objects and `FETCH_HEAD`, but it does not force-reset the worktree.

## Synchronize

From PowerShell in the Windows checkout:

```powershell
# Preflight without fetch or checkout
.\tools\sync_to_wsl.ps1 -CheckOnly

# Fetch and check out the exact committed Windows HEAD in WSL
.\tools\sync_to_wsl.ps1
```

Expected completion marker:

```text
SYNC_OK
```

The script prints `WSL_HEAD_BEFORE` and `WSL_HEAD_AFTER`. The previous commit remains recoverable through the WSL Git reflog. If the preflight stops, inspect the reported state; do not bypass the check with a reset or cleanup command.

## Validate in WSL

```bash
ssh codex-wsl
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
source .venv/bin/activate

# Commit identity and preserved environment
git status --short --branch
git rev-parse HEAD
python --version
python -c 'import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))'
test -f datasets/processed/aic_real_dataset_v2/index.csv

# Repository validation under the shared worktree lock
tools/with_wsl_training_lock.sh pytest -q
tools/with_wsl_training_lock.sh python tools/smoke_test.py --config configs/transfuser_lite_v0.yaml
```

Run training through the same lock, for example:

```bash
tools/with_wsl_training_lock.sh python -m aic_transfuser_lite.training.train_v1 \
  --help
```

Replace `--help` with reviewed task-specific arguments. Use a task-specific output directory below `runs/`. Record the Git commit, config path, dataset index, seed, checkpoint, and validation results in the run metadata or report.

## Return results to Windows

Keep raw data and large artifacts in WSL. Copy back only selected small metrics, manifests, plots, or review reports after checking that they contain no credentials or private data. Model weights and datasets remain ignored and must not be added to Git.
