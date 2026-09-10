"""Bounded reference experiment; original model output remains untouched."""
import numpy as np


def resample_shadow(raw, spacing_m: float, maximum_deviation_m: float):
    if spacing_m not in (.2,.3) or not 0<maximum_deviation_m<=.03:
        raise ValueError('SHADOW_RESAMPLE_POLICY')
    xy=np.asarray(raw,dtype=float)
    if xy.shape!=(20,2) or not np.isfinite(xy).all():raise ValueError('RAW_SHAPE_FINITE')
    ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    if np.any(ds>.5) or np.any(ds<1e-9):raise ValueError('RAW_DISCONTINUITY_OR_DUPLICATE')
    arc=np.r_[0.,np.cumsum(ds)]
    def sample(poly,step):
        s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(poly,axis=0),axis=1))]
        q=np.unique(np.r_[np.arange(0.,s[-1],step),s[-1]])
        return np.c_[np.interp(q,s,poly[:,0]),np.interp(q,s,poly[:,1])]
    out=sample(xy,spacing_m)
    def distance(points,poly):
        a=poly[:-1];d=np.diff(poly,axis=0);den=np.maximum(np.sum(d*d,axis=1),1e-24)
        q=points[:,None,:]-a;u=np.clip(np.sum(q*d,axis=2)/den,0,1)
        return float(np.max(np.min(np.linalg.norm(q-u[:,:,None]*d,axis=2),axis=1)))
    # 2mm grid => true continuous symmetric distance <= sampled maximum+1mm.
    upper=max(distance(sample(xy,.002),out),distance(sample(out,.002),xy))+.001
    if upper>maximum_deviation_m:raise ValueError('REFERENCE_DEVIATION_EXCEEDED')
    return out,upper
