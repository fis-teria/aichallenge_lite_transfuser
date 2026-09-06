"""unittest + NumPy only, no torch/model/optimizer imports or runtime."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location("residuals",Path(__file__).parents[1]/"tools/analyze_spatial_fixed_residuals_v4.py")
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


class ResidualTests(unittest.TestCase):
    def setUp(self):
        self.target=np.stack((m.GRID,np.zeros(20)),axis=1)[None]
        self.mask=np.ones((1,20),bool)

    def test_signed_components(self):
        d=np.array([[1.,-2.],[-1.,-2.]])
        s=m.component_stats(d)
        self.assertEqual(s['bias_dx_m'],0); self.assertEqual(s['bias_dy_m'],-2)
        self.assertEqual(s['mae_dx_m'],1); self.assertEqual(s['rmse_dy_m'],2)

    def test_mask_and_tail_invariance(self):
        mask=self.mask.copy(); mask[:,10:]=False
        first=m.summarize(self.target,self.target.copy(),mask,[0])
        prediction=self.target.copy(); prediction[:,10:]=100
        second=m.summarize(self.target,prediction,mask,[0])
        self.assertEqual(first,second)

    def test_zero_support_and_nonfinite_invalid_teacher(self):
        t=self.target.copy(); t[:]=np.nan
        mask=np.zeros_like(self.mask)
        m.validate_arrays(t,self.target,mask)
        result=m.summarize(t,self.target,mask,[0])
        self.assertIsNone(result['anchor_balanced']['ade_m'])
        self.assertEqual(result['valid_points'],0)

    def test_contiguous_mask_required(self):
        mask=self.mask.copy(); mask[:,2]=False
        with self.assertRaises(ValueError):m.validate_arrays(self.target,self.target,mask)

    def test_fixed_cohort_vs_variable(self):
        mask=np.ones((2,20),bool); mask[1,10:]=False
        self.assertEqual(m.cohort_indices(mask,[0,1],'fixed_2m_support'),[0])
        self.assertEqual(m.cohort_indices(mask,[0,1],'variable_support'),[0,1])

    def test_tangent_constant_offset_not_heading_error(self):
        r=m.tangent_window(self.target[0],self.target[0]+[.3,-.1],self.mask[0],5)
        self.assertAlmostEqual(r['angle_error_rad'],0)
        self.assertAlmostEqual(r['teacher_chord_m'],.3)

    def test_tangent_wrapped_rotation_and_scale(self):
        p=self.target[0][:,::-1]*2
        r=m.tangent_window(self.target[0],p,self.mask[0],0)
        self.assertAlmostEqual(r['angle_error_rad'],np.pi/2)
        self.assertAlmostEqual(r['prediction_chord_m'],.6)

    def test_unknown_chord_and_no_bridge(self):
        r=m.tangent_window(self.target[0],np.zeros((20,2)),self.mask[0],0)
        self.assertEqual(r['status'],'UNKNOWN_DEGENERATE_CHORD')
        mask=self.mask[0].copy();mask[2]=False
        self.assertEqual(m.tangent_window(self.target[0],self.target[0],mask,0)['status'],'UNKNOWN_NO_TEACHER_SUPPORT')

    def test_anchor_balanced_not_point_weighted(self):
        t=np.concatenate((self.target,self.target)); p=t.copy();p[0,:,0]+=1;p[1,:,0]+=3
        mask=np.ones((2,20),bool);mask[0,1:]=False
        r=m.summarize(t,p,mask,[0,1])
        self.assertAlmostEqual(r['anchor_balanced']['bias_dx_m'],2)
        self.assertAlmostEqual(r['anchor_balanced']['rmse_dx_m'],np.sqrt(5))

    def test_inputs_unchanged(self):
        target=self.target.copy();pred=target.copy();mask=self.mask.copy()
        m.summarize(target,pred,mask,[0])
        np.testing.assert_array_equal(target,self.target);np.testing.assert_array_equal(mask,self.mask)


if __name__=='__main__':unittest.main(verbosity=2)
