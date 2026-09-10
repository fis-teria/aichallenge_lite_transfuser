from tools.audit_spatial_long_horizon import summarize


def test_support_is_not_extrapolated_and_unknown_is_not_zero():
    rows=[dict(split='train',normal_recovery='normal',run_id='r',h30_noise_filtered_arc_m_provisional=v)
          for v in ('','nan','1.9','5','20')]
    r=summarize(rows)[0]
    assert r['known']==3 and r['unknown']==2
    assert r['potential_support']=={'2':2,'5':2,'10':1,'20':1}
