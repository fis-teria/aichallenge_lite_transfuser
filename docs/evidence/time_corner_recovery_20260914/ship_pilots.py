"""Explicit closed-run shipping using the previous structural verification code.

pack: AWSIM host; verify: native WSL lock; cleanup: restricted /collection mount.
No original data is removed by pack or verify.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import tarfile
import time

REMOTE = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')
ANALYSIS = Path('/home/thistle/e2e_autonomous/runs/time_corner_recovery_20260914')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_corner_recovery_20260914')
ALLOWED = {'codex-time-recovery-cornerleft020-r27', 'codex-time-recovery-cornerleft020-r29',
           'codex-time-recovery-cornerright020-r28'}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('pack','verify','cleanup'))
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--runs', nargs='+', required=True)
    parser.add_argument('--snapshot-sha256')
    args = parser.parse_args()
    names = args.runs
    assert 1 <= len(names) <= 2 and len(set(names)) == len(names) and set(names) <= ALLOWED
    assert re.fullmatch(r'cornerpair0[12]_20260914', args.prefix)
    spec = importlib.util.spec_from_file_location('verified_previous_shipping',args.library)
    assert spec and spec.loader
    library = importlib.util.module_from_spec(spec); spec.loader.exec_module(library)
    library.RAW = RAW; library.ANALYSIS = ANALYSIS
    if args.mode != 'pack':
        assert re.fullmatch('[0-9a-f]{64}',args.snapshot_sha256 or '')
        if args.mode == 'verify':
            library.verify(ANALYSIS,args.prefix,names,args.snapshot_sha256)
        else:
            library.cleanup(Path('/collection'),args.prefix,names,args.snapshot_sha256)
        return
    assert REMOTE.resolve() == REMOTE
    assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
    ledger = json.loads((REMOTE/'corner_campaign_20260914.json').read_text())
    assert set(names) <= {r['run_id'] for r in ledger['attempts']}
    rows = []
    for name in names:
        run = REMOTE/name
        result = json.loads((run/'result.json').read_text())
        assert result['nodes']['closed_bag'] and not result['cleanup_errors']
        row = library.inventory(run)
        for rel, expected in json.loads((run/'transfer_manifest.json').read_text()).items():
            actual = row['files'][rel]
            if actual['kind'] == 'symlink':
                target = (run/rel).resolve(); assert target.is_relative_to(run)
                actual = row['files'][target.relative_to(run).as_posix()]
            assert actual['kind'] == 'file' and actual['bytes'] == expected['bytes'] and actual['sha256'] == expected['sha256']
        rows.append(row)
    snapshot = REMOTE/(args.prefix+'_snapshot.json')
    library.write(snapshot,dict(created_unix=time.time(),runs=rows,source_root=str(REMOTE),
                                campaign_sha256=library.sha(REMOTE/'corner_campaign_20260914.json')))
    archive = REMOTE/(args.prefix+'.tar.gz')
    with tarfile.open(archive,'x:gz',compresslevel=1) as tf:
        for row in rows:
            name = row['run_id']
            for rel in ['',*row['directories']]:
                info = tarfile.TarInfo(name+('/'+rel if rel else '')); info.type=tarfile.DIRTYPE; info.mode=0o755
                tf.addfile(info)
            for rel,record in row['files'].items():
                info=tarfile.TarInfo(name+'/'+rel); info.mode=0o644
                if record['kind']=='symlink':
                    info.type=tarfile.SYMTYPE; info.linkname=record['target']; tf.addfile(info)
                else:
                    info.size=record['bytes']
                    with (REMOTE/name/rel).open('rb') as stream:
                        tf.addfile(info,stream)
    receipt=dict(snapshot_sha256=library.sha(snapshot),archive_sha256=library.sha(archive),
                 archive_bytes=archive.stat().st_size,run_ids=names,
                 source_regular_file_bytes=sum(r['regular_file_bytes'] for r in rows))
    library.write(REMOTE/(args.prefix+'_shipping.json'),receipt)
    print(json.dumps(receipt),flush=True)


if __name__ == '__main__':
    main()
