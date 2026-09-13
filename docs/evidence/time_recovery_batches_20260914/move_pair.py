"""Bounded campaign shipping: pack closed runs, verify in WSL, then remove originals.

Run pack on the simulator host, verify under the native WSL worktree lock,
and cleanup in a restricted container mounting only the collection root.
The caller supplies the exact two run IDs and the snapshot SHA for each stage.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import subprocess
import tarfile
import time

REMOTE = Path('/home/graneple/e2e_autonomous/time_recovery_collection_20260913')
ANALYSIS = Path('/home/thistle/e2e_autonomous/runs/time_recovery_batches_20260914')
RAW = Path('/home/thistle/e2e_autonomous/raw/time_recovery_batches_20260914')


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024**2), b''):
            h.update(block)
    return h.hexdigest()


def write(path: Path, value: dict) -> None:
    with path.open('x') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


def inventory(run: Path) -> dict:
    assert run.is_dir() and not run.is_symlink()
    files: dict = {}
    directories = []
    for p in sorted(run.rglob('*')):
        relative = p.relative_to(run).as_posix()
        assert p.resolve().is_relative_to(run.resolve()), str(p)
        if p.is_symlink():
            assert not p.readlink().is_absolute()
            files[relative] = dict(kind='symlink', target=str(p.readlink()))
        elif p.is_dir():
            directories.append(relative)
        else:
            assert p.is_file()
            files[relative] = dict(kind='file', bytes=p.stat().st_size, sha256=sha(p))
    return dict(run_id=run.name, files=files, directories=directories,
                regular_file_bytes=sum(x.get('bytes', 0) for x in files.values()))


def pack(root: Path, prefix: str, names: list[str]) -> None:
    assert root == REMOTE and not subprocess.check_output(['docker', 'ps', '-q'], text=True).strip()
    ledger = json.loads((root/'campaign_20260914.json').read_text())
    resident = [p['run_id'] for p in ledger['attempts'] if p['state'] != 'WSL_MOVED']
    assert sorted(resident) == sorted(names) and len(names) == ledger['batch_size'] == 2
    rows = []
    for name in names:
        run = root/name
        result = json.loads((run/'result.json').read_text())
        assert result['nodes']['closed_bag'] and not result.get('cleanup_errors')
        row = inventory(run)
        manifest = json.loads((run/'transfer_manifest.json').read_text())
        for rel, record in manifest.items():
            actual = row['files'][rel]
            assert actual['kind'] == 'file' and actual['bytes'] == record['bytes']
            assert actual['sha256'] == record['sha256']
        rows.append(row)
    snapshot = root/(prefix+'_snapshot.json')
    write(snapshot, dict(created_unix=time.time(), runs=rows, source_root=str(root),
                         campaign_sha256=sha(root/'campaign_20260914.json')))
    archive = root/(prefix+'.tar.gz')
    assert not archive.exists()
    with tarfile.open(archive, 'x:gz', compresslevel=1) as tf:
        for row in rows:
            name = row['run_id']
            for rel in ['', *row['directories']]:
                info = tarfile.TarInfo(name+('/'+rel if rel else ''))
                info.type = tarfile.DIRTYPE; info.mode = 0o755
                tf.addfile(info)
            for rel, record in row['files'].items():
                path = root/name/rel
                info = tarfile.TarInfo(name+'/'+rel); info.mode = 0o644
                if record['kind'] == 'symlink':
                    info.type = tarfile.SYMTYPE; info.linkname = record['target']
                    tf.addfile(info)
                else:
                    info.size = record['bytes']
                    with path.open('rb') as stream:
                        tf.addfile(info, stream)
    receipt = dict(snapshot_sha256=sha(snapshot), archive_sha256=sha(archive),
                   archive_bytes=archive.stat().st_size, run_ids=names,
                   source_regular_file_bytes=sum(x['regular_file_bytes'] for x in rows))
    write(root/(prefix+'_shipping.json'), receipt)
    print(json.dumps(receipt), flush=True)


def verify(root: Path, prefix: str, names: list[str], expected: str) -> None:
    assert root == ANALYSIS
    snapshot = root/(prefix+'_snapshot.json')
    assert sha(snapshot) == expected
    snap = json.loads(snapshot.read_text())
    shipping = json.loads((root/(prefix+'_shipping.json')).read_text())
    assert shipping['snapshot_sha256'] == expected and shipping['run_ids'] == names
    assert [row['run_id'] for row in snap['runs']] == names
    archive = root/(prefix+'.tar.gz')
    assert archive.stat().st_size == shipping['archive_bytes']
    assert sha(archive) == shipping['archive_sha256']
    RAW.mkdir(exist_ok=True)
    assert RAW.resolve() == RAW and all(not (RAW/n).exists() for n in names)
    expected_members = {}
    for row in snap['runs']:
        n = row['run_id']; expected_members[n] = dict(kind='dir')
        expected_members.update({n+'/'+d: dict(kind='dir') for d in row['directories']})
        expected_members.update({n+'/'+f: r for f, r in row['files'].items()})
    with tarfile.open(archive, 'r:gz') as tf:
        members = tf.getmembers()
        assert len(members) == len(expected_members)
        assert {m.name for m in members} == set(expected_members)
        for m in members:
            target = RAW/m.name
            assert target.resolve().is_relative_to(RAW) and not Path(m.name).is_absolute()
            record = expected_members[m.name]
            if record['kind'] == 'dir':
                assert m.isdir()
                target.mkdir(mode=0o755)
            elif record['kind'] == 'file':
                assert m.isfile() and m.size == record['bytes']
                source = tf.extractfile(m); assert source is not None
                with source, target.open('xb') as output:
                    shutil.copyfileobj(source, output, length=8*1024**2)
            else:
                assert m.issym() and m.linkname == record['target'] and not Path(m.linkname).is_absolute()
                assert (target.parent/m.linkname).resolve().is_relative_to(RAW/m.name.split('/')[0])
                target.symlink_to(m.linkname)
    checked = []
    for row in snap['runs']:
        run = RAW/row['run_id']
        assert inventory(run) == row, row['run_id']
        dbs = list((run/'bag').glob('*.db3')); assert len(dbs) == 1
        with sqlite3.connect(dbs[0].as_uri()+'?mode=ro', uri=True) as connection:
            assert connection.execute('PRAGMA quick_check').fetchall() == [('ok',)]
        checked.append(dict(run_id=run.name, raw_path=str(run), file_entries=len(row['files']),
                            regular_file_bytes=row['regular_file_bytes']))
    result = dict(snapshot_sha256=expected, archive_sha256=sha(archive), runs=checked,
                  all_files_and_directory_structure_identical=True,
                  all_sqlite_quick_checks_passed=True, verified_unix=time.time())
    write(root/(prefix+'_verified.json'), result)
    print(json.dumps(result), flush=True)


def cleanup(root: Path, prefix: str, names: list[str], expected: str) -> None:
    assert root == Path('/collection') and os.geteuid() == 0
    snapshot = root/(prefix+'_snapshot.json')
    assert sha(snapshot) == expected
    snap = json.loads(snapshot.read_text())
    verified = json.loads((root/(prefix+'_verified.json')).read_text())
    shipping = json.loads((root/(prefix+'_shipping.json')).read_text())
    assert verified['snapshot_sha256'] == shipping['snapshot_sha256'] == expected
    assert verified['all_files_and_directory_structure_identical'] and verified['all_sqlite_quick_checks_passed']
    assert [r['run_id'] for r in snap['runs']] == [r['run_id'] for r in verified['runs']] == names
    marker_root = root/(prefix+'_markers')
    assert marker_root.resolve().parent == root and not marker_root.is_symlink()
    for row in snap['runs']:
        run = root/row['run_id']; assert run.resolve().parent == root
        assert inventory(run) == row
        marker = marker_root/run.name
        assert marker.resolve().parent == marker_root and not marker.is_symlink()
        note = json.loads((marker/'MOVED_TO_WSL.json').read_text())
        assert note['snapshot_sha256'] == expected and note['raw_path'] == str(RAW/run.name)
        print('SOURCE_RECHECK', run.name, flush=True)
    archive = root/(prefix+'.tar.gz')
    assert archive.resolve().parent == root and not archive.is_symlink()
    assert archive.stat().st_size == shipping['archive_bytes'] and sha(archive) == shipping['archive_sha256']
    result_path = root/(prefix+'_cleanup.json')
    result = json.loads(result_path.read_text()); assert result['state'] == 'PREPARED'
    for name in names:
        shutil.rmtree(root/name)
        (marker_root/name).rename(root/name)
        result['removed_runs'].append(name)
        result.update(state='SOURCE_CLEANUP_IN_PROGRESS')
        result_path.write_text(json.dumps(result, indent=2)+'\n')
    archive.unlink()
    result.update(state='COMPLETE', after_free_bytes=shutil.disk_usage(root).free,
                  completed_unix=time.time(), temporary_transfer_archive_removed=True)
    result_path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=('pack', 'verify', 'cleanup'))
    ap.add_argument('--pair', type=int, choices=(1, 2, 3, 4), required=True)
    ap.add_argument('--runs', nargs=2, required=True)
    ap.add_argument('--snapshot-sha256')
    args = ap.parse_args()
    assert len(set(args.runs)) == 2
    assert all(re.fullmatch(r'codex-time-recovery-(left|right)(020|040)-r2[1-8]', n) for n in args.runs)
    prefix = f'pair{args.pair:02d}_20260914'
    if args.mode == 'pack':
        pack(REMOTE, prefix, args.runs)
    else:
        assert re.fullmatch('[0-9a-f]{64}', args.snapshot_sha256 or '')
        if args.mode == 'verify':
            verify(ANALYSIS, prefix, args.runs, args.snapshot_sha256)
        else:
            cleanup(Path('/collection'), prefix, args.runs, args.snapshot_sha256)


if __name__ == '__main__':
    main()
