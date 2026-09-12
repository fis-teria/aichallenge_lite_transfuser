"""Freeze the approved SI26 simulator/controller inputs without editing either source tree."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

SOURCE = Path('/home/si26-pc008/git/autonomous_ai/aichallenge-racingkart')
LEGACY = Path('/home/si26-pc008/ga_cma_ws')
ROOT = Path('/home/si26-pc008/cma_mppi_20260912')
IMAGE = 'aichallenge-h2h-car1:ed978b5195cc-599bf86a79c3-223b3374b07c'
ARCHIVE_SHA = 'ed978b5195cc933d190ff04baf43989dcabd51fbb20a89dfc0bb60444190349b'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def manifest(root: Path) -> dict[str, str]:
    return {str(p.relative_to(root)): sha256(p) for p in sorted(root.rglob('*')) if p.is_file()}


def main() -> None:
    snapshot = ROOT / 'snapshot'
    snapshot.mkdir(parents=True, exist_ok=False)
    archive = SOURCE / 'head_to_head/submissions/local-worktree-car1.tar.gz'
    if sha256(archive) != ARCHIVE_SHA:
        raise RuntimeError('Approved controller archive changed; inspect before adopting it')
    image_id = subprocess.check_output(['docker', 'image', 'inspect', IMAGE, '--format', '{{.Id}}'], text=True).strip()
    original = manifest(LEGACY / 'aichallenge/simulator/AWSIM')
    (ROOT / 'legacy_awsim_manifest.json').write_text(json.dumps(original, indent=2))
    awsim = SOURCE / 'aichallenge/simulator/AWSIM'
    before = manifest(awsim)
    shutil.copytree(awsim, snapshot / 'AWSIM')
    if before != manifest(snapshot / 'AWSIM') or before != manifest(awsim):
        raise RuntimeError('Simulator changed while snapshotting')
    submit = SOURCE / 'aichallenge/workspace/src/aichallenge_submit'
    copies = {
        'submission.tar.gz': archive,
        'base_reference.csv': submit / 'multi_purpose_mpc_ros/env/final_ver3/in_corce_line_straight_smooth.csv',
        'mppi.yaml': submit / 'reference_space_mppi_planner/config/reference_space_mppi.param.yaml',
        'reference.launch.xml': submit / 'aichallenge_submit_launch/launch/reference.launch.xml',
        'footprint.yaml': submit / 'reference_space_mppi_planner/config/awsim_wall_map/footprint.param.yaml',
        'ot_lane_region.json': SOURCE / 'ai-work/runs/ot-lane-region-20260907/ot_lane_region.json',
        'previous_calibration.yaml': SOURCE / 'head_to_head/scenario_tool/state/calibration.yaml',
        'cyclonedds.xml': SOURCE / 'vehicle/cyclonedds.xml',
        'admin_watch.py': SOURCE / 'head_to_head/admin_watch.py',
    }
    for name, src in copies.items():
        shutil.copy2(src, snapshot / name)
    metadata = {'source_root': str(SOURCE), 'legacy_root': str(LEGACY),
                'controller_image_tag': IMAGE, 'controller_image_id': image_id,
                'source_head': subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip(),
                'source_diff_sha256': hashlib.sha256(subprocess.check_output(['git', '-C', str(SOURCE), 'diff', '--binary'])).hexdigest(),
                'conditions': [{'name': 'normal', 'target_mps': 10.0, 'handicap': False},
                               {'name': 'leader', 'target_mps': 7.5, 'handicap': True}],
                'awsim_files': before, 'inputs': {n: sha256(snapshot / n) for n in copies}}
    (ROOT / 'environment.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps({'root': str(ROOT), 'image_id': image_id, 'awsim_files': len(before),
                      'archive_sha256': ARCHIVE_SHA}), flush=True)


if __name__ == '__main__':
    main()
