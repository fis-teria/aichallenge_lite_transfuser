from manage import remote, ROOT, UNC, HOST
import subprocess

remote(rf'''
from pathlib import Path
import hashlib,json,subprocess,tarfile
root=Path({ROOT!r});stage=root/'marker_update_30da4c7';prep=json.loads((stage/'marker_update.json').read_text())
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
deployment=json.loads((root/'deployment.json').read_text());assert deployment['source_commit']==prep['old_commit']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for rel,digest in deployment['files'].items():
 if rel not in prep['files']:assert sha(root/rel)==digest,rel
with tarfile.open(stage/'marker_source_update.tar.gz') as tf:
 for m in tf.getmembers():
  assert m.isfile() and m.name in prep['files']
  digest=hashlib.sha256(tf.extractfile(m).read()).hexdigest()
  assert sha(root/m.name)==digest;deployment['files'][m.name]=digest
oldlog=stage/'official_ros_smoke.log';oldlog.rename(stage/'official_ros_smoke_second_dds_failure.log')
(stage/'smoke_dds.xml').write_text('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
cmd=['docker','run','--rm','--network','none','--name','codex-recovery-marker-smoke',
 '-e','ROS_DOMAIN_ID=95','-v',str(root)+':/capture:ro',
 '-v','/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge:/aichallenge:ro',
 '--entrypoint','bash','codex-cartographer-v4-build:20260910','-lc',
 'source /aichallenge/workspace/install/setup.bash && export PYTHONPATH=/capture/source/src:${{PYTHONPATH:-}} && export CYCLONEDDS_URI=file:///capture/'+stage.name+'/smoke_dds.xml && python3 /capture/'+stage.name+'/marker_ros_smoke.py']
r=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
(stage/'official_ros_smoke.log').write_text(r.stdout+r.stderr)
assert r.returncode==0,(r.stdout+r.stderr)[-6000:]
rows=[json.loads(s) for s in r.stdout.splitlines() if s.startswith('{{')];smoke=rows[-1];assert smoke['passed']
(root/'marker_official_image_smoke.json').write_text(json.dumps(smoke,indent=2))
deployment['source_commit']=prep['commit']
(root/'deployment.json').write_text(json.dumps(deployment,indent=2))
(root/'test_gate.json').write_text(json.dumps(prep['gate'],indent=2))
(root/'deployed_commit.txt').write_text(prep['commit']+'\n')
(stage/'applied.json').write_text(json.dumps(dict(commit=prep['commit'],runtime_files=len(deployment['files']),changed_files=prep['files'],smoke=smoke),indent=2))
print((stage/'applied.json').read_text())
''',timeout=90)
subprocess.run(['scp',HOST+':'+ROOT+'/marker_official_image_smoke.json',str(UNC/'marker_official_image_smoke.json')],check=True)
