"""Source-backed evaluation checks, including failed inputs and biased references."""
import numpy as np
from .episode_10_1 import write
from .trajectory import reference
from .comparison_results import error_metrics


def verify(out):
    from .trajectory_checks import verify as trajectory_verify
    report=trajectory_verify(out)
    t=np.arange(0.,30.,1/30);r=reference(t,[0.,0.,0.])
    data=np.zeros((len(t),26));data[:,1]=t;data[:,2:5]=r[:,:3];data[:,14:20]=r
    data[:,20]=.03;data[:,21]=.04
    metrics=error_metrics(data)
    assert abs(metrics['rmse_normal_m']-.05)<1e-12
    # A large transient outside the predetermined window must not alter its RMSE.
    transient=data.copy();transient[t<7,20]=1
    assert abs(error_metrics(transient)['rmse_normal_m']-.05)<1e-12
    assert error_metrics(transient)['rmse_all_m']>metrics['rmse_all_m']
    failed=data.copy();failed[:,20]=.20
    assert error_metrics(failed)['rmse_normal_m']>.12
    partial=data[t<15]
    try:error_metrics(partial)
    except RuntimeError as error:
        report['cases'].append(dict(name='incomplete_evaluation_window',kind='executed_input_rejection',success=False,
                                    expected_failure=True,verification_passed=True,reason=str(error)))
    else:raise AssertionError('Partial window accepted')
    wrong_time=reference(t+.4,[0.,0.,0.])
    assert np.max(np.linalg.norm(r[:,:3]-wrong_time[:,:3],axis=1))>.05
    np.savez(out/'evaluation_checks_raw.npz',t=t,data=data,transient=transient,failed=failed,wrong_time_reference=wrong_time)
    report['checks'] += ['known 3D RMSE .05m','fixed-window transient exclusion','failure criterion .12m',
                         'incomplete window refusal','reference clock shift is observable']
    report['synthetic_evaluation']=dict(rmse_m=metrics['rmse_normal_m'],failed_rmse_m=error_metrics(failed)['rmse_normal_m'],
                                      note='Synthetic checks, not measured Gazebo outcomes')
    write(out/'meaningful_checks.json',report)
    return report
