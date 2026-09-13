"""Prepare a new, source-verified deployment; never modifies prior deployments."""
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile,time
root=Path('/home/graneple/e2e_autonomous/time_segment_following_20260914')
assert root.resolve()==root and root.is_dir()
prior=root.parent/'time_recovery_model_candidate_20260914'
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
manifest=json.loads((root/'source_manifest.json').read_text())
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def text_run(cmd):
 return subprocess.run(cmd,cwd=repo,text=True,capture_output=True,check=True,timeout=20).stdout.strip()
def write(name,value):
 with (root/name).open('x') as f:json.dump(value,f,indent=2)
assert not text_run(['docker','ps','-q'])
rviz=repo/'aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz'
write('pre_environment.json',{'repo_head':text_run(['git','rev-parse','HEAD']),
 'repo_status':text_run(['git','status','--porcelain']),
 'repo_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=repo)).hexdigest(),
 'containers':text_run(['docker','ps','-a','--format','{{json .}}']),
 'compose':text_run(['docker','compose','ls','--all','--format','json']),
 'rviz_sha256':sha(rviz),'free_bytes':shutil.disk_usage(root).free})
archive=root/manifest['archive_file']
assert sha(archive)==manifest['archive_sha256']
source=root/manifest['source_dir'];assert not source.exists()
with tarfile.open(archive) as tar:
 for member in tar.getmembers():
  target=(root/member.name).resolve()
  assert (target==source or source in target.parents) and (member.isdir() or member.isfile()),member.name
 tar.extractall(root)
verified=[]
for entry in manifest['entries']:
 p=source/entry['path'];data=p.read_bytes()
 assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==entry['git_blob'],entry['path']
 verified.append({'path':entry['path'],'sha256':hashlib.sha256(data).hexdigest()})
with (prior/'command_off_best.pt').open('rb') as src, (root/'command_off_best.pt').open('xb') as dst:shutil.copyfileobj(src,dst)
assert sha(root/'command_off_best.pt')==manifest['checkpoint_sha256']
with (root/'smoke_dds.xml').open('xb') as f:f.write((prior/'smoke_dds.xml').read_bytes())
write('source_verification.json',{'status':'PASS','source_commit':manifest['source_commit'],
 'archive_sha256':sha(archive),'checkpoint_sha256':sha(root/'command_off_best.pt'),'source_files':verified})
image='codex-cartographer-v4-build:20260910'
base=['docker','run','--rm','--network','none','-v',str(root)+':/time','-v',str(repo/'aichallenge')+':/aichallenge:ro','--entrypoint','bash',image,'-lc']
def run(name,cmd,seconds):
 write(name+'_command.json',cmd);began=time.monotonic();print('START '+name,flush=True)
 with (root/(name+'.log')).open('x') as f:
  p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=seconds,cwd=repo)
 receipt={'exit':p.returncode,'seconds':time.monotonic()-began};write(name+'_exit.json',receipt);print(json.dumps({name:receipt}),flush=True)
 if p.returncode:raise RuntimeError(name+' failed; see '+str(root/(name+'.log')))
build=base[:3]+['--user','1000:1000']+base[3:]+['source /aichallenge/workspace/install/setup.bash && cd /time/'+source.name+'/ros2_ws && colcon --log-base /time/build_log build --packages-select aic_e2e_runtime --build-base /time/build --install-base /time/install']
run('build',build,180)
checked=[]
for p in (source/'src/aic_transfuser_lite').rglob('*.py'):
 rel=p.relative_to(source/'src');installed=root/'install/aic_e2e_runtime/share/aic_e2e_runtime/python_src'/rel
 assert sha(p)==sha(installed),str(rel);checked.append({'path':str(rel),'sha256':sha(installed)})
for p in (source/'ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime').rglob('*.py'):
 rel=p.relative_to(source/'ros2_ws/src/aic_e2e_runtime');installed=root/'install/aic_e2e_runtime/lib/python3.10/site-packages'/rel
 assert sha(p)==sha(installed),str(rel);checked.append({'path':str(rel),'sha256':sha(installed)})
write('install_verification.json',{'status':'PASS','files':checked})
smoke=base[:3]+['-e','ROS_DOMAIN_ID=93','-e','CYCLONEDDS_URI=file:///time/smoke_dds.xml']+base[3:]+['source /aichallenge/workspace/install/setup.bash && source /time/install/setup.bash && timeout --signal=TERM --kill-after=10s 100s python3 /time/'+source.name+'/tools/check_time_ros_connection.py --checkpoint /time/command_off_best.pt --checkpoint-sha256 '+manifest['checkpoint_sha256']+' --trial-config /time/'+source.name+'/'+manifest['config']+' --output /time/smoke']
run('smoke',smoke,120)
assert json.loads((root/'smoke/summary.json').read_text())['status']=='PASS'
print(json.dumps({'status':'READY_FOR_BOUNDED_AWSIM','source_commit':manifest['source_commit'],'verified_install_files':len(checked)}),flush=True)
