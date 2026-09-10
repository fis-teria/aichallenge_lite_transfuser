"""Coverage inventory only: saved audit ledger, no sensor/model/raw reads."""
import argparse
import csv
import hashlib
import json
import math
from collections import defaultdict, Counter
from pathlib import Path


def summarize(rows):
    groups=defaultdict(list)
    for row in rows:
        groups[(row.get('split','UNKNOWN'),row.get('normal_recovery','UNKNOWN'))].append(row)
    result=[]
    for (split,kind),items in sorted(groups.items()):
        lengths=[];unknown=0
        for row in items:
            try: value=float(row.get('h30_noise_filtered_arc_m_provisional',''))
            except (TypeError,ValueError):unknown+=1;continue
            if not math.isfinite(value) or value<0:unknown+=1;continue
            lengths.append(value)
        result.append(dict(split=split,kind=kind,total=len(items),known=len(lengths),unknown=unknown,
            maximum_saved_prefix_m=max(lengths,default=None),
            potential_support={str(d):sum(v>=d for v in lengths) for d in (2,5,10,20)},
            run_count=len({r['run_id'] for r in items}),
            eligibility_counts=dict(Counter(r.get('h30_path_loss_eligibility','') for r in items))))
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--ledger',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();data=a.ledger.read_bytes()
    result=dict(source=str(a.ledger),sha256=hashlib.sha256(data).hexdigest(),
        meaning='PROVISIONAL_SAVED_PREFIX_SUPPORT_NOT_TRAINING_ELIGIBILITY',
        note='Uninspected rows remain UNKNOWN. No stitching across anchors, runs or epochs.',
        groups=summarize(csv.DictReader(data.decode().splitlines())))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
