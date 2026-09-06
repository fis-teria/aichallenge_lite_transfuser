"""Pure saved-array diagnostic. No model, torch, Dataset, checkpoint or raw imports/I/O."""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

GRID = np.arange(1, 21, dtype=np.float64)/10
WINDOW_STEPS = 3  # fixed 0.3 m in nominal teacher distance grid, not measured chord
DEGENERATE_EPS_M = 1e-12  # numerical undefined-direction check, not tuned performance gate
INPUTS = ("evidence/train_targets.npz", "evidence/train_predictions.npz",
          "evidence/validation_targets.npz", "evidence/validation_predictions.npz",
          "artifacts/metrics.json", "artifacts/annotation_addendum.json")


def sha(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


def clean(value: object) -> object:
    if isinstance(value, dict): return {str(k): clean(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [clean(v) for v in value]
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer=csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(clean(rows))


def validate_arrays(target: np.ndarray, prediction: np.ndarray, mask: np.ndarray) -> None:
    if target.ndim != 3 or target.shape[1:] != (20,2) or prediction.shape != target.shape or mask.shape != target.shape[:2] or mask.dtype != bool:
        raise ValueError("expected target/prediction [N,20,2], bool mask [N,20]")
    if not np.isfinite(target[mask]).all() or not np.isfinite(prediction).all():
        raise ValueError("nonfinite valid teacher/prediction")
    if np.any(mask[:,1:] & ~mask[:,:-1]):
        raise ValueError("mask is not contiguous prefix")


def component_stats(delta: np.ndarray) -> dict:
    """[M,2] residuals. Within a distance grid M denotes anchors, not time samples."""
    if not len(delta):
        return {"count":0, "bias_dx_m":None,"bias_dy_m":None,"mae_dx_m":None,"mae_dy_m":None,"rmse_dx_m":None,"rmse_dy_m":None,"ade_m":None}
    return {"count":len(delta), "bias_dx_m":float(delta[:,0].mean()), "bias_dy_m":float(delta[:,1].mean()),
        "mae_dx_m":float(np.abs(delta[:,0]).mean()), "mae_dy_m":float(np.abs(delta[:,1]).mean()),
        "rmse_dx_m":float(np.sqrt((delta[:,0]**2).mean())),"rmse_dy_m":float(np.sqrt((delta[:,1]**2).mean())),
        "ade_m":float(np.linalg.norm(delta,axis=1).mean())}


def tangent_window(target: np.ndarray, prediction: np.ndarray, mask: np.ndarray, start: int) -> dict:
    """0.3m nominal grid window [start,start+3], no origin, smoothing or interpolation."""
    end=start+WINDOW_STEPS
    if not 0 <= start <= 16: raise ValueError("window outside saved 20 points")
    if not mask[start:end+1].all():
        return {"status":"UNKNOWN_NO_TEACHER_SUPPORT","angle_error_rad":None,"teacher_chord_m":None,"prediction_chord_m":None}
    a,b=target[end]-target[start],prediction[end]-prediction[start]
    la,lb=float(np.linalg.norm(a)),float(np.linalg.norm(b))
    if min(la,lb) <= DEGENERATE_EPS_M:
        return {"status":"UNKNOWN_DEGENERATE_CHORD","angle_error_rad":None,"teacher_chord_m":la,"prediction_chord_m":lb}
    raw=np.arctan2(b[1],b[0])-np.arctan2(a[1],a[0])
    return {"status":"DEFINED","angle_error_rad":float(np.arctan2(np.sin(raw),np.cos(raw))),"teacher_chord_m":la,"prediction_chord_m":lb}


def cohort_indices(mask: np.ndarray, ids: list[int], cohort: str) -> list[int]:
    if cohort == "variable_support": return ids
    if cohort == "fixed_2m_support": return [i for i in ids if mask[i].all()]
    raise ValueError("unknown cohort")


def summarize(target: np.ndarray, prediction: np.ndarray, mask: np.ndarray, ids: list[int]) -> dict:
    delta=prediction-target
    per_anchor=[component_stats(delta[i,mask[i]]) for i in ids if mask[i].any()]
    keys=("bias_dx_m","bias_dy_m","mae_dx_m","mae_dy_m","ade_m")
    balanced={key:float(np.mean([r[key] for r in per_anchor])) if per_anchor else None for key in keys}
    for key in ("rmse_dx_m","rmse_dy_m"):
        balanced[key]=float(np.sqrt(np.mean([r[key]**2 for r in per_anchor]))) if per_anchor else None
    grids=[]
    for k,s in enumerate(GRID):
        selected=[i for i in ids if mask[i,k]]
        grids.append({"s_m":s, **component_stats(delta[selected,k])})
    windows=[]
    for j in range(17):
        stats=[tangent_window(target[i],prediction[i],mask[i],j) for i in ids]
        angles=[r["angle_error_rad"] for r in stats if r["status"]=="DEFINED"]
        windows.append({"start_s_m":GRID[j],"end_s_m":GRID[j+3],"defined_anchors":len(angles),
            "unsupported_anchors":sum(r["status"]=="UNKNOWN_NO_TEACHER_SUPPORT" for r in stats),
            "degenerate_anchors":sum(r["status"]=="UNKNOWN_DEGENERATE_CHORD" for r in stats),
            "signed_wrapped_mean_rad":float(np.mean(angles)) if angles else None,
            "circular_mean_rad":float(np.arctan2(np.sin(angles).mean(),np.cos(angles).mean())) if angles else None,
            "mae_rad":float(np.abs(angles).mean()) if angles else None,
            "rmse_rad":float(np.sqrt(np.mean(np.square(angles)))) if angles else None})
    spacing=[]
    for k in range(20):
        chosen=[i for i in ids if mask[i,k]]
        t=[float(np.linalg.norm(target[i,k]-(target[i,k-1] if k else np.zeros(2)))) for i in chosen]
        p=[float(np.linalg.norm(prediction[i,k]-(prediction[i,k-1] if k else np.zeros(2)))) for i in chosen]
        spacing.append({"end_s_m":GRID[k],"anchors":len(chosen),"origin_to_first":k==0,
            "teacher_mean_m":float(np.mean(t)) if t else None,"prediction_mean_m":float(np.mean(p)) if p else None,
            "spacing_bias_m":float(np.mean(np.array(p)-t)) if t else None,
            "spacing_mae_m":float(np.mean(np.abs(np.array(p)-t))) if t else None})
    return {"cohort_anchors":len(ids),"supported_anchors":len(per_anchor),"valid_points":int(mask[ids].sum()),
            "anchor_balanced":balanced,"distance":grids,"tangent_windows":windows,"spacing":spacing}


def analyze(target: np.ndarray, prediction: np.ndarray, mask: np.ndarray, metadata: list[dict]) -> tuple[dict,list[dict],list[dict],list[dict],list[dict]]:
    validate_arrays(target,prediction,mask)
    if len(metadata)!=len(target) or len({r['sample_id'] for r in metadata}) != len(metadata):
        raise ValueError("metadata length/duplicate ID")
    summary={}
    for role in sorted({r["role"] for r in metadata}):
        role_ids=[i for i,r in enumerate(metadata) if r["role"]==role]
        summary[role]={}
        for cohort in ("variable_support","fixed_2m_support"):
            ids=cohort_indices(mask,role_ids,cohort)
            summary[role][cohort]={"overall":summarize(target,prediction,mask,ids),
                "sample_ids":[metadata[i]["sample_id"] for i in ids]}
            for field in ("shape","run_id","normal_recovery","collection_slice"):
                summary[role][cohort][field]={label:summarize(target,prediction,mask,[i for i in ids if metadata[i][field]==label])
                    for label in sorted({metadata[i][field] for i in role_ids})}
    points,anchors,windows,spacing=[],[],[],[]
    for i,item in enumerate(metadata):
        tags={k:item[k] for k in ("sample_id","role","run_id","shape","normal_recovery","collection_slice")}
        stats=component_stats((prediction[i]-target[i])[mask[i]])
        def length(x: np.ndarray) -> float | None:
            return float(np.linalg.norm(np.diff(np.vstack((np.zeros((1,2)),x[mask[i]])),axis=0),axis=1).sum()) if mask[i].any() else None
        lt,lp=length(target[i]),length(prediction[i])
        anchors.append({**tags,**stats,"fixed_2m_support":bool(mask[i].all()),"teacher_polyline_m":lt,"prediction_polyline_m":lp,
                        "polyline_delta_m":lp-lt if lt is not None else None})
        for k in range(20):
            valid=bool(mask[i,k]); dx,dy=prediction[i,k]-target[i,k] if valid else (None,None)
            points.append({**tags,"s_m":GRID[k],"teacher_supported":valid,"dx_m":dx,"dy_m":dy,
                           "euclidean_error_m":float(np.hypot(dx,dy)) if valid else None})
            t0=target[i,k-1] if k else np.zeros(2); p0=prediction[i,k-1] if k else np.zeros(2)
            lt=float(np.linalg.norm(target[i,k]-t0)) if valid else None
            lp=float(np.linalg.norm(prediction[i,k]-p0)) if valid else None
            spacing.append({**tags,"end_s_m":GRID[k],"origin_to_first":k==0,"teacher_supported":valid,
                            "teacher_spacing_m":lt,"prediction_spacing_m":lp,"spacing_delta_m":lp-lt if valid else None})
        for j in range(17):
            windows.append({**tags,"start_s_m":GRID[j],"end_s_m":GRID[j+3],**tangent_window(target[i],prediction[i],mask[i],j)})
    return summary,points,anchors,windows,spacing


def run(packet: Path, output: Path) -> None:
    if output.exists(): raise FileExistsError("immutable new output required")
    repo=Path(__file__).resolve().parents[1]
    if subprocess.check_output(["git","status","--porcelain"],cwd=repo,text=True).strip(): raise ValueError("commit source first")
    output.mkdir(parents=True)
    inventory=json.loads((packet/"PACKAGE_MANIFEST.json").read_text(encoding="utf-8"))
    files={r["path"]:r for r in inventory["files"]}
    blobs={}
    for name in INPUTS:
        path=packet/name
        if path.is_symlink(): raise ValueError("symlink input")
        blob=path.read_bytes()
        if sha(blob)!=files[name]["sha256"] or len(blob)!=files[name]["size_bytes"]: raise ValueError("packet input identity mismatch")
        blobs[name]=blob
    metrics=json.loads(blobs["artifacts/metrics.json"])
    addendum=json.loads(blobs["artifacts/annotation_addendum.json"])
    if sha(blobs["artifacts/metrics.json"])!=addendum["original_metrics_sha256"] or not addendum["selected_id_order_unchanged"]:
        raise ValueError("annotation addendum binding mismatch")
    labels={r["sample_id"]:r for r in addendum["selected_annotations"]}
    targets,predictions,masks,metadata=[],[],[],[]
    replay=[]
    for split in ("train","validation"):
        t=np.load(io.BytesIO(blobs[f"evidence/{split}_targets.npz"]),allow_pickle=False)
        p=np.load(io.BytesIO(blobs[f"evidence/{split}_predictions.npz"]),allow_pickle=False)
        for a in (t,p):
            for key in a.files:
                if a[key].dtype.hasobject: raise ValueError("object array")
        np.testing.assert_array_equal(t["grid_m"],GRID)
        np.testing.assert_array_equal(t["sample_ids"],p["sample_ids"])
        if not p["processed"].all(): raise ValueError("source contains unprocessed predictions")
        rows=metrics[split]["predictions"]["model"]["rows"]
        np.testing.assert_array_equal(t["sample_ids"],[r["sample_id"] for r in rows])
        validate_arrays(t["xy"],p["xy"],t["mask"])
        for i,row in enumerate(rows):
            mask=t["mask"][i]
            ade=float(np.linalg.norm(p["xy"][i,mask]-t["xy"][i,mask],axis=1).mean()) if mask.any() else None
            if int(mask.sum())!=row["valid_points"] or (ade is None)!=(row["ade_m"] is None) or (ade is not None and abs(ade-row["ade_m"])>1e-10):
                raise ValueError("saved ADE/denominator mismatch")
            if split=="validation":
                label=labels[row["sample_id"]]
                if any(label[k]!=row[k] for k in ("sample_id","run_id","role")): raise ValueError("annotation ID/role mismatch")
                row={**row,**{k:label[k] for k in ("normal_recovery","collection_slice")}}
            metadata.append(row)
        targets.append(t["xy"]); predictions.append(p["xy"]); masks.append(t["mask"])
        replay.append({"split":split,"anchors":len(rows),"supported":int(t["mask"].any(axis=1).sum()),"points":int(t["mask"].sum()),"saved_ade_per_anchor_match":True})
    target,prediction,mask=np.concatenate(targets),np.concatenate(predictions),np.concatenate(masks)
    counts={role:sum(r["role"]==role for r in metadata) for role in {r["role"] for r in metadata}}
    if counts!={"train_replay":64,"validation_main":160,"validation_observation":20}: raise ValueError("fixed cohort counts differ")
    summary,points,anchors,windows,spacing=analyze(target,prediction,mask,metadata)
    write_json(output/"summary.json",summary)
    for filename,values in (("per_point_residuals.csv",points),("per_anchor_residuals.csv",anchors),("tangent_windows.csv",windows),("point_spacing.csv",spacing)):
        write_csv(output/filename,values)
    write_json(output/"ade_reconciliation.json",replay)
    source=output/"source"; source.mkdir()
    source_files=["tools/analyze_spatial_fixed_residuals_v4.py","tests/test_spatial_fixed_residuals_v4.py","docs/spatial_path_v4_fixed_residual_method.md"]
    for name in source_files:
        (source/Path(name).name).write_bytes((repo/name).read_bytes())
    make_figures(output,target,prediction,mask,metadata,summary,anchors)
    for name,blob in blobs.items():
        if (packet/name).read_bytes()!=blob: raise ValueError("input bytes changed")
    write_json(output/"provenance.json",{"execution_commit":subprocess.check_output(["git","rev-parse","HEAD"],cwd=repo,text=True).strip(),
        "utc":datetime.now(timezone.utc).isoformat(),"input_hashes":{name:sha(blob) for name,blob in blobs.items()},
        "source_hashes":{name:sha((repo/name).read_bytes()) for name in source_files},"argv":sys.argv,"numpy":np.__version__,"python":sys.version,
        "grid_m":GRID,"tangent_window_nominal_m":.3,"tangent_origin_used":False,"degenerate_chord_epsilon_m":DEGENERATE_EPS_M,
        "component_sign":"prediction minus teacher: dx forward, dy left, meters",
        "inputs_unchanged":True,"new_inference_calls":0,"checkpoint_reads":0,"dataset_raw_reads":0,"optimizer_steps":0,
        "cohort_definition":"variable_support changes at each distance; fixed_2m_support requires original mask all20. Not reselection for evaluation.",
        "aggregation":"per-distance anchor mean; overall anchor-balanced MAE/bias/ADE, RMSE sqrt(mean per-anchor mean squared error)"})
    print(json.dumps(clean({"status":"COMPLETE","cohorts":{r:{c:{"n":v["overall"]["cohort_anchors"],**v["overall"]["anchor_balanced"]} for c,v in groups.items()} for r,groups in summary.items()}}),indent=2))


def make_figures(output: Path,target: np.ndarray,prediction: np.ndarray,mask: np.ndarray,meta: list[dict],summary: dict,anchors: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    folder=output/"figures"; folder.mkdir()
    fig,axes=plt.subplots(2,2,figsize=(10,7))
    for cohort,style in (("variable_support","-"),("fixed_2m_support","--")):
        for shape,color in (("straight","blue"),("left","green"),("right","red")):
            group=summary["validation_main"][cohort]["shape"][shape]
            for ax,key in zip(axes.flat,("bias_dx_m","bias_dy_m","mae_dx_m","mae_dy_m")):
                ax.plot(GRID,[r[key] for r in group["distance"]],style,color=color,label=f"{shape} {cohort}")
                ax.set(title=key,xlabel="nominal teacher distance [m]",ylabel="residual [m]"); ax.grid()
    axes[0,0].legend(fontsize=6); fig.tight_layout(); fig.savefig(folder/"component_profiles.png",dpi=130); plt.close(fig)
    chosen=[]
    for role,shape in (("validation_main","straight"),("validation_main","left"),("validation_main","right"),("validation_observation",None)):
        ids=[i for i,r in enumerate(meta) if r["role"]==role and (shape is None or r["shape"]==shape) and anchors[i]["ade_m"] is not None]
        if ids: chosen.append(max(ids,key=lambda i:anchors[i]["ade_m"]))
    write_json(folder/"figure_selection.json",{"policy":"max saved ADE within predeclared role/shape; visualization only, no new selection/inference","ids":[meta[i]["sample_id"] for i in chosen]})
    for i in chosen:
        fig,ax=plt.subplots(figsize=(6,6))
        ax.plot(target[i,mask[i],0],target[i,mask[i],1],"go-",label="saved teacher support")
        ax.plot(prediction[i,:,0],prediction[i,:,1],"r--",label="saved prediction (tail UNKNOWN)")
        ax.plot(prediction[i,mask[i],0],prediction[i,mask[i],1],"ro-",label="prediction on support")
        ax.set(title=meta[i]["sample_id"]+f"\n{meta[i]['role']} / {meta[i]['shape']}",xlabel="X forward [m]",ylabel="Y left [m]")
        ax.axis("equal"); ax.grid(); ax.legend(fontsize=7); fig.tight_layout(); fig.savefig(folder/f"example_{i:03d}.png",dpi=120); plt.close(fig)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet",type=Path,required=True); parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(); run(args.packet,args.output)
