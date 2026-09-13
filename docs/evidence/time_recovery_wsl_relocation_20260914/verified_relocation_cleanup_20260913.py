import pathlib,json,hashlib,shutil,datetime
r=pathlib.Path('/collection').resolve()
assert str(r)=='/collection'
snapshot=r/'wsl_relocation_snapshot_20260913.json'
expected='06f36a125f7645856a1ec3ef6770aacdaaadde10dfc22d92aa39ead9754c30d0'
assert hashlib.sha256(snapshot.read_bytes()).hexdigest()==expected
s=json.loads(snapshot.read_text());v=json.loads((r/'wsl_relocation_verified_20260913.json').read_text())
assert v['snapshot_sha256']==expected and v['run_count']==20 and v['file_entries']==1069
assert v['all_files_and_directory_structure_identical'] and v['all_sqlite_quick_checks_passed']
allowed={f'codex-time-recovery-left020-r{i:02d}' for i in list(range(1,10))+list(range(11,19))+[20]}|{'codex-time-recovery-preflight-r10','codex-time-recovery-right020-r19'}
assert {x['run_id'] for x in s['runs']}==allowed=={x['run_id'] for x in v['runs']}
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
 return h.hexdigest()
# Recheck every target before deleting any source data.
for row in s['runs']:
 p=r/row['run_id'];assert p.resolve().parent==r and p.is_dir() and not p.is_symlink()
 found={};dirs=[]
 for q in sorted(p.rglob('*')):
  rel=str(q.relative_to(p));assert q.resolve().is_relative_to(p)
  if q.is_symlink():found[rel]={'kind':'symlink','target':str(q.readlink())}
  elif q.is_dir():dirs.append(rel)
  else:
   assert q.is_file();found[rel]={'kind':'file','bytes':q.stat().st_size,'sha256':sha(q)}
 assert found==row['files'] and dirs==row['directories'],row['run_id']
 marker=r/'relocation_markers_20260913'/row['run_id']
 assert marker.resolve().parent==r/'relocation_markers_20260913' and not marker.is_symlink()
 m=json.loads((marker/'MOVED_TO_WSL.json').read_text());assert m['snapshot_sha256']==expected
 print('SOURCE_RECHECK',row['run_id'],flush=True)
a=r/'remaining_diagnostics_for_wsl_20260913.tar.gz'
assert a.resolve().parent==r and not a.is_symlink() and a.stat().st_size==367885829
assert sha(a)=='2c632a5a1866e862a2afa4b1d503b52e141477ec0b7015eae7b94ff49acf2af8'
result_path=r/'wsl_relocation_result_20260913.json';result=json.loads(result_path.read_text());assert result['state']=='PREPARED'
for row in s['runs']:
 p=r/row['run_id'];marker=r/'relocation_markers_20260913'/row['run_id']
 assert p.resolve().parent==r and not p.is_symlink() and marker.resolve().parent==r/'relocation_markers_20260913'
 shutil.rmtree(p)
 marker.rename(p)
 result['removed_runs'].append({'run_id':row['run_id'],'source_payload_removed':True,'marker_retained':True,'regular_file_bytes':row['regular_file_bytes']})
 result['state']='SOURCE_CLEANUP_IN_PROGRESS';result_path.write_text(json.dumps(result,indent=2)+'\n')
 print('MOVED',row['run_id'],flush=True)
a.unlink()
result.update(state='COMPLETE',completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),after_free_bytes=shutil.disk_usage(r).free,removed_run_count=len(result['removed_runs']),source_regular_file_bytes_removed=sum(x['regular_file_bytes'] for x in result['removed_runs']),temporary_transfer_archive_removed=True)
result_path.write_text(json.dumps(result,indent=2)+'\n')
print('RELOCATION_COMPLETE',json.dumps({k:v for k,v in result.items() if k!='removed_runs'}),flush=True)
