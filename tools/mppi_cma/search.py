"""Finite, resumable CMA-ES raceline search. Only owned isolated episodes are started."""
from __future__ import annotations
import fcntl
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from analyze_episode import analyze
from geometry import generate_reference
from run_episode import ROOT, run_episode
from parallel_dispatch import dispatch


def save(path: Path, value: object) -> None:
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--workers',type=int,choices=range(1,5),default=1)
    args=parser.parse_args()
    lock=(ROOT/'search.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    study=ROOT/'search';study.mkdir(exist_ok=True)
    state_file=study/'state.json'
    if state_file.exists():state=json.loads(state_file.read_text())
    else:
        image=subprocess.check_output(['docker','image','inspect','aichallenge-2025-dev-ga','--format','{{.Id}}'],text=True).strip()
        state={'started_unix_s':time.time(),'deadline_unix_s':time.time()+7200,
               'maximum_new_episodes':43,'new_episodes_started':0,'completed':False,
               'optimizer_image_id':image,'conditions':{},'phase':'preflight'}
        save(state_file,state)
    if state['completed']:
        print('Search already completed',flush=True);return
    state['parallel_workers']=args.workers
    state['active_episodes']=[]
    state.pop('active_episode',None)
    save(state_file,state)

    def budget() -> None:
        if time.time()>state['deadline_unix_s'] or state['new_episodes_started']>=state['maximum_new_episodes']:
            raise RuntimeError('Finite search budget reached; no additional episode started')

    def evaluate(name: str, condition: str, path: Path) -> dict:
        output=ROOT/'episodes'/name
        if not (output/'runtime_result.json').exists():
            if output.exists():raise RuntimeError('Incomplete episode needs inspection: '+name)
            budget();state['new_episodes_started']+=1;state['active_episode']=name;save(state_file,state)
            state['active_episodes']=[name];save(state_file,state)
            run_episode(name,handicap=condition=='leader',reference=path,target_mps=7.5 if condition=='leader' else 10.)
        result=analyze(output)
        state['active_episodes']=[];state.pop('active_episode',None);save(state_file,state)
        print(json.dumps(result),flush=True)
        return result

    def batch(jobs: list[dict], record) -> list[dict]:
        results={}
        pending=[]
        for job in jobs:
            output=ROOT/'episodes'/job['name']
            if (output/'runtime_result.json').exists():
                result=analyze(output);results[job['name']]=result;record(job,result)
            elif output.exists():
                raise RuntimeError('Incomplete episode needs inspection: '+job['name'])
            else:pending.append(job)
        save(state_file,state)

        def started(_index,job):
            budget()
            state['new_episodes_started']+=1
            state['active_episodes'].append(job['name'])
            state['active_episode']=state['active_episodes'][0]
            save(state_file,state)

        def run(job):
            run_episode(job['name'],handicap=job['condition']=='leader',reference=job['path'],
                        target_mps=7.5 if job['condition']=='leader' else 10.)
            return analyze(ROOT/'episodes'/job['name'])

        def finished(_index,job,result):
            state['active_episodes'].remove(job['name'])
            if state['active_episodes']:state['active_episode']=state['active_episodes'][0]
            else:state.pop('active_episode',None)
            if isinstance(result,Exception):
                state['last_error']={'episode':job['name'],'error':str(result)}
            else:
                results[job['name']]=result;record(job,result)
                print(json.dumps(result),flush=True)
            save(state_file,state)

        dispatch(pending,run,started,finished,args.workers)
        return [results[job['name']] for job in jobs]

    def cma_step(directory: Path, command: str) -> None:
        subprocess.run(['docker','run','--rm','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
                        '-v',str(ROOT)+':/work','--entrypoint','python3',state['optimizer_image_id'],
                        '/work/tools/cma_step.py','/work/'+str(directory.relative_to(ROOT)),command],check=True,timeout=45)

    for condition,baseline in [('normal','baseline-normal02'),('leader','baseline-leader01')]:
        directory=study/condition;directory.mkdir(exist_ok=True)
        baseline_metrics=analyze(ROOT/'episodes'/baseline)
        seed_path=ROOT/'candidates/outside24/reference.csv'
        seed=evaluate('seed-'+condition+'-outside24',condition,seed_path)
        if not seed['completed'] or sum(seed['penalties'][k] for k in ['crash','wall','over']):
            raise RuntimeError('Seed path did not pass completion/contact check; inspect before broad search')
        if condition not in state['conditions']:
            seed_anchors=json.loads((seed_path.parent/'anchors.json').read_text())
            save(directory/'seed_anchors.json',seed_anchors)
            state['conditions'][condition]={'next_generation':0,'evaluations':[
                {'metrics':baseline_metrics,'anchors_m':[0.]*16,'reference':str(ROOT/'snapshot/base_reference.csv')},
                {'metrics':seed,'anchors_m':seed_anchors,'reference':str(seed_path)}]}
            save(state_file,state)
        progress=state['conditions'][condition]
        if not (directory/'optimizer.pkl').exists():cma_step(directory,'init')
        for generation in range(progress['next_generation'],3):
            generation_file=directory/f'generation_{generation:02d}.json'
            if generation_file.exists():proposals=json.loads(generation_file.read_text())
            else:
                if generation>0:cma_step(directory,'ask')
                proposals=json.loads((directory/'proposals.json').read_text());save(generation_file,proposals)
            jobs=[]
            for index,anchors in enumerate(proposals):
                name=f'{condition}-g{generation:02d}-c{index:02d}'
                candidate=directory/name/'reference.csv'
                if not candidate.exists():generate_reference(ROOT/'snapshot/base_reference.csv',candidate,anchors)
                jobs.append({'name':name,'condition':condition,'path':candidate,'anchors_m':anchors})
            state['phase']='search';save(state_file,state)
            def record_candidate(job,metrics):
                record={'metrics':metrics,'anchors_m':job['anchors_m'],'reference':str(job['path'])}
                if not any(x['metrics']['episode']==job['name'] for x in progress['evaluations']):progress['evaluations'].append(record)
            measured=batch(jobs,record_candidate)
            objectives=[metrics['objective'] for metrics in measured]
            save(directory/'evaluated.json',{'candidates':proposals,'objectives':objectives})
            cma_step(directory,'tell')
            progress['next_generation']=generation+1;save(state_file,state)
        best=min(progress['evaluations'],key=lambda x:(x['metrics']['objective'],x['metrics']['episode']))
        progress['best']=best;state['phase']='validation';save(state_file,state)
        validation_jobs=[{'name':f'{condition}-best-repeat{i+1}','condition':condition,'path':Path(best['reference'])} for i in range(2)]
        validation_jobs.append({'name':condition+'-baseline-repeat1','condition':condition,'path':ROOT/'snapshot/base_reference.csv'})
        validation=batch(validation_jobs,lambda _job,_metrics:None)
        repeats=validation[:2];repeated_baseline=validation[2]
        progress['best_repeats']=repeats;progress['baseline_repeat']=repeated_baseline
        progress['validated_preferred_feasible']=all(x['preferred_feasible'] for x in [best['metrics'],*repeats])
        save(state_file,state)
    state['completed']=True;state['phase']='complete';state.pop('active_episode',None)
    save(state_file,state)
    print('BOUNDED SEARCH COMPLETE',flush=True)


if __name__=='__main__':main()
