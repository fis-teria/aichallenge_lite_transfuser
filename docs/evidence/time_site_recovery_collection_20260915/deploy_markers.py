"""Apply only tested marker runtime files to the idle owned collection root."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

from manage import REPO, HERE, ROOT, UNC, remote, copy_to_host

old = '945f241cf150c48ac507e3fbedcf94c2dfef59c9'
head = subprocess.check_output(['git','rev-parse','HEAD'], cwd=REPO,text=True).strip()
assert not subprocess.check_output(['git','status','--porcelain'], cwd=REPO,text=True).strip()
gate = json.loads((UNC.parent/f'time_site_recovery_gate_{head[:7]}.json').read_text())
assert gate['commit'] == head and gate['full_exit'] == 0
assert hashlib.sha256((UNC.parent/Path(gate['log']).name).read_bytes()).hexdigest() == gate['log_sha256']
paths = subprocess.check_output(['git','diff','--name-only',old,head,'--','src','tools','configs'],cwd=REPO,text=True).splitlines()
assert len(paths) == 4
archive = HERE/'marker_source_update.tar.gz'
with tarfile.open(archive,'x:gz') as tf:
    for rel in paths:
        data = subprocess.check_output(['git','show',head+':'+rel],cwd=REPO)
        info = tarfile.TarInfo('source/'+rel); info.size = len(data); info.mode = 0o644
        tf.addfile(info, io.BytesIO(data))
prep = dict(old_commit=old, commit=head, files=['source/'+r for r in paths], gate=gate,
            archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
prep_file = HERE/'marker_update.json'; prep_file.write_text(json.dumps(prep,indent=2))
stage = ROOT+'/marker_update_'+head[:7]
remote(f"from pathlib import Path;Path({stage!r}).mkdir(exist_ok=False)")
copy_to_host([archive,prep_file,HERE/'marker_ros_smoke.py'],stage)
remote(rf'''
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
root=Path({ROOT!r});stage=Path({stage!r});prep=json.loads((stage/'marker_update.json').read_text())
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
deployment=json.loads((root/'deployment.json').read_text())
assert deployment['source_commit']==prep['old_commit']==(root/'deployed_commit.txt').read_text().strip()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for rel,digest in deployment['files'].items():assert sha(root/rel)==digest,rel
assert sha(stage/'marker_source_update.tar.gz')==prep['archive_sha256']
backup=stage/'before';backup.mkdir()
for name in ('deployment.json','deployed_commit.txt','test_gate.json'):shutil.copy2(root/name,backup/name)
with tarfile.open(stage/'marker_source_update.tar.gz') as tf:
 members=tf.getmembers();assert sorted(m.name for m in members)==sorted(prep['files'])
 for m in members:
  dst=root/m.name;assert m.isfile() and dst.resolve().is_relative_to(root/'source')
  if dst.exists():
   save=backup/m.name;save.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dst,save)
  pending=dst.with_suffix(dst.suffix+'.marker_pending')
  with pending.open('xb') as f:f.write(tf.extractfile(m).read())
  pending.replace(dst);deployment['files'][m.name]=sha(dst)
(stage/'smoke_dds.xml').write_text('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
cmd=['docker','run','--rm','--network','none','--name','codex-recovery-marker-smoke',
 '-e','ROS_DOMAIN_ID=95','-v',str(root)+':/capture:ro',
 '-v','/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge:/aichallenge:ro',
 '--entrypoint','bash','codex-cartographer-v4-build:20260910','-lc',
 'source /aichallenge/workspace/install/setup.bash && export PYTHONPATH=/capture/source/src:${{PYTHONPATH:-}} && export CYCLONEDDS_URI=file:///capture/'+stage.name+'/smoke_dds.xml && python3 /capture/'+stage.name+'/marker_ros_smoke.py']
result=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
(stage/'official_ros_smoke.log').write_text(result.stdout+result.stderr)
assert result.returncode==0,(result.stdout+result.stderr)[-6000:]
lines=[json.loads(s) for s in result.stdout.splitlines() if s.startswith('{{')]
assert lines[-1]['passed'];(root/'marker_official_image_smoke.json').write_text(json.dumps(lines[-1],indent=2))
deployment['source_commit']=prep['commit']
(root/'deployment.json').write_text(json.dumps(deployment,indent=2))
(root/'test_gate.json').write_text(json.dumps(prep['gate'],indent=2))
(root/'deployed_commit.txt').write_text(prep['commit']+'\n')
(stage/'applied.json').write_text(json.dumps(dict(commit=prep['commit'],runtime_files=len(deployment['files']),changed_files=prep['files'],smoke=lines[-1]),indent=2))
print((stage/'applied.json').read_text())
''',timeout=90)
for name in ('marker_official_image_smoke.json',):
    subprocess.run(['scp','graneple@192.168.3.10:'+ROOT+'/'+name,str(UNC/name)],check=True)
