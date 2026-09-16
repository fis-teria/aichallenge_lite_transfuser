"""Select a runtime-test candidate using fixed launch/nominal/recovery evidence.

This gate authorizes only a subsequent bounded simulator test. It cannot certify
collision avoidance or closed-loop completion. Training best.pt is not promotion.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Sequence


@dataclass(frozen=True)
class StageSelectionPolicy:
    relative_error_tolerance: float = .05
    ade_tolerance_m: float = .001
    endpoint_tolerance_m: float = .002
    launch_margin_tolerance_rad: float = .001

    def validate(self) -> None:
        if any(not math.isfinite(v) or v < 0 for v in asdict(self).values()):
            raise ValueError('finite nonnegative tolerances in explicit SI units required')


def select_stage_candidate(reports: Sequence[dict[str,Any]], *, baseline_id: str,
                           policy: StageSelectionPolicy) -> dict[str,Any]:
    """Compare every candidate with the immutable initial baseline.

    xy stages require run-macro ADE/3 s errors, one fixed population fingerprint
    and positive anchor/run support. Launch cases have fixed IDs including age,
    teacher applicability, PP acceptance, and selected physical tire angles [rad].
    Every teacher-supported launch case must pass; dropping a case is an error.
    Margin comparisons allow the curvature actually required by the teacher;
    an initial prediction that understeers must not make that teacher inadmissible.
    """
    policy.validate()
    ids=[r['candidate_id'] for r in reports]
    if not reports or len(ids)!=len(set(ids)) or baseline_id not in ids:
        raise ValueError('unique candidate IDs including initial baseline required')
    baseline=next(r for r in reports if r['candidate_id']==baseline_id)
    stages=set(baseline['xy'])
    if stages!={'nominal','recovery'}:
        raise ValueError('separate nominal and recovery stages required')
    def launch_index(report: dict[str,Any]) -> dict[str,Any]:
        rows=report['launch'];result={r['case_id']:r for r in rows}
        if not rows or len(result)!=len(rows):
            raise ValueError('nonempty unique launch cases required')
        for row in rows:
            if (type(row['teacher_accepted']) is not bool or type(row['accepted']) is not bool
                    or not math.isfinite(row['age_s']) or not 0<=row['age_s']<=.5):
                raise ValueError('explicit boolean support and launch ages within 0..0.5 s required')
            teacher_angle=row.get('teacher_steer_rad')
            if row['teacher_accepted'] and (teacher_angle is None or not math.isfinite(teacher_angle)
                    or abs(teacher_angle)>.3):
                raise ValueError('supported teacher angle must satisfy the physical contract')
        return result
    initial=launch_index(baseline)
    supported={key for key,row in initial.items() if row['teacher_accepted']}
    if not supported:
        raise ValueError('no supported launch teachers')
    decisions=[]
    for report in reports:
        if report['population_sha256']!=baseline['population_sha256'] or set(report['xy'])!=stages:
            raise ValueError('candidate validation population changed')
        cases=launch_index(report)
        if cases.keys()!=initial.keys() or any(
                (cases[k]['teacher_accepted'],cases[k].get('teacher_steer_rad'),cases[k]['run_id'],cases[k]['age_s']) !=
                (initial[k]['teacher_accepted'],initial[k].get('teacher_steer_rad'),initial[k]['run_id'],initial[k]['age_s']) for k in initial):
            raise ValueError('launch cases, ages, teacher angle or denominator changed')
        reasons=[];relative=[]
        for stage in sorted(stages):
            row,old=report['xy'][stage],baseline['xy'][stage]
            if (row['anchors']!=old['anchors'] or row['runs']!=old['runs']
                    or row['anchors']<=0 or row['runs']<=0):
                raise ValueError('fixed positive stage support required')
            for metric,tolerance in [('ade_m',policy.ade_tolerance_m),('endpoint_3s_m',policy.endpoint_tolerance_m)]:
                value,previous=row[metric],old[metric]
                if previous is None or not math.isfinite(previous) or previous<0:
                    raise ValueError('baseline stage metric must have finite support')
                if value is None or not math.isfinite(value) or value<0:
                    reasons.append(f'{stage}:{metric}:NONFINITE');continue
                limit=previous+max(tolerance,previous*policy.relative_error_tolerance)
                if value>limit:reasons.append(f'{stage}:{metric}:REGRESSED')
                relative.append(value/max(previous,1e-6))
        rejected=[];margins=[]
        for key in sorted(supported):
            row,old=cases[key],initial[key]
            if not row['accepted']:
                rejected.append(key);continue
            angle=row.get('steer_rad')
            if angle is None or not math.isfinite(angle) or abs(angle)>.3:
                rejected.append(key);continue
            margin=.3-abs(angle);margins.append(margin)
            reference_margin=.3-abs(row['teacher_steer_rad'])
            if old['accepted']:
                old_angle=old.get('steer_rad')
                if old_angle is None or not math.isfinite(old_angle) or abs(old_angle)>.3:
                    raise ValueError('baseline accepted PP angle outside physical contract')
                reference_margin=min(reference_margin,.3-abs(old_angle))
            if margin+policy.launch_margin_tolerance_rad < reference_margin:
                reasons.append('launch:STEERING_MARGIN_REGRESSED:'+key)
        if rejected:reasons.append('launch:PP_REJECTED')
        decisions.append(dict(candidate_id=report['candidate_id'],eligible=not reasons,reasons=reasons,
            launch_supported=len(supported),launch_rejected_case_ids=rejected,
            minimum_selected_margin_rad=min(margins) if margins else None,
            score=sum(relative)/len(relative) if len(relative)==4 else None))
    eligible=[d for d in decisions if d['eligible']]
    # Baseline participates in ranking; deterministic ties retain it.
    selected=min(eligible,key=lambda d:(d['score'],d['candidate_id']!=baseline_id,d['candidate_id'])) if eligible else None
    promoted=selected is not None and selected['candidate_id']!=baseline_id
    return dict(status='CANDIDATE_FOR_AWSIM_TEST' if promoted else 'NO_NEW_CANDIDATE',
        selected_candidate_id=selected['candidate_id'] if selected else baseline_id,
        selected_meets_gate=selected is not None,runtime_test_allowed=promoted,
        baseline_id=baseline_id,policy=asdict(policy),decisions=decisions,
        launch_margin_reference='minimum_of_teacher_and_accepted_initial_margin_else_teacher',
        scope='OFFLINE_DEVELOPMENT_GATE_NOT_CLOSED_LOOP_CERTIFICATION')
