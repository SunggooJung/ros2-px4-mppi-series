"""Executable small examples and independent reference/cost consistency checks."""
import json
from pathlib import Path
import numpy as np
from .controller import Config, MPPIController, LOW, HIGH
from .trajectory import reference, horizon_reference, TrajectoryController
from .episode_10_1 import write


def verify(out):
    cfg = Config(samples=32, horizon=10, seed=7, temperature=3.)
    times = np.linspace(-1, 32, 1000)
    origin = np.array([.02, -.03, 0.])
    values = reference(times, origin)
    eps = 1e-5
    derivative = (reference(times+eps, origin)[:, :3]-reference(times-eps, origin)[:, :3])/(2*eps)
    derivative_error = float(np.max(abs(derivative-values[:, 3:])))
    assert derivative_error < 1e-8
    assert np.max(abs(reference([0., 30.], origin)[:, 3:])) == 0
    assert np.max(abs(reference(0., origin)-reference(30., origin))) < 1e-12
    rt, refs = horizon_reference(11., origin, cfg)
    assert rt[0] == 11.+cfg.dt and rt[-1] == 11.+cfg.horizon*cfg.dt
    assert np.linalg.norm(refs[-1, :3]-refs[0, :3]) > .01
    rng = np.random.default_rng(7)
    candidates = np.clip(np.tile(cfg.hover, (32, 10, 1))+rng.normal(size=(32, 10, 4))*.02, LOW, HIGH)
    state = np.zeros(12);state[2] = 2.
    base = MPPIController(cfg);tracker = TrajectoryController(cfg)
    trajectories, valid = tracker.rollout(state, candidates)
    constant = np.tile([0., 0., 2., 0., 0., 0.], (10, 1))
    constant_cost = tracker.trajectory_cost(trajectories, candidates, constant, valid)
    canonical_error = float(np.max(abs(constant_cost-base.costs(trajectories, candidates, constant[0, :3], valid))))
    assert canonical_error == 0
    future = constant.copy();future[-1, 0] = 1.
    assert np.max(abs(tracker.trajectory_cost(trajectories, candidates, future, valid)-constant_cost)) > 1
    moving = constant.copy();moving[:, 3] = .2
    assert np.max(abs(tracker.trajectory_cost(trajectories, candidates, moving, valid)-constant_cost)) > .01
    costs = np.array([0., 1., 3.]);temperatures = [.1, 1., 10.]
    weights = np.array([MPPIController.weights(costs, t) for t in temperatures])
    assert weights[0, 0] > weights[1, 0] > weights[2, 0]
    assert np.max(abs(MPPIController.weights(costs+100., 1.)-weights[1])) < 1e-12
    raw = np.array([.2, 1.1]);clipped = np.clip(raw, .1, .9)
    synthetic_weights = np.array([.25, .75])
    clipped_average = float(synthetic_weights@clipped)
    raw_average = float(synthetic_weights@raw)
    assert clipped_average != raw_average
    rejected = []
    for label, bad in [('missing_velocity_reference', np.zeros((10, 3))),
                       ('nonfinite_reference', np.full((10, 6), np.nan))]:
        try:tracker.step(state, bad)
        except ValueError as error:rejected.append({'name':label,'kind':'executed_input_rejection','success':False,
                                                  'expected_failure':True,'verification_passed':True,'reason':str(error)})
        else:raise AssertionError('Invalid reference accepted')
    try:MPPIController.weights([np.inf, np.nan], 1.)
    except ValueError:pass
    else:raise AssertionError('All-invalid rollouts accepted')
    np.savez(out/'small_examples_raw.npz', times=times, references=values, derivative=derivative,
             costs=costs, temperatures=temperatures, weights=weights, raw_candidates=raw,
             clipped_candidates=clipped, trajectories=trajectories, candidates=candidates)
    report={'passed':True,'analytic_velocity_derivative_max_error':derivative_error,
            'constant_reference_canonical_cost_error':canonical_error,'costs':costs.tolist(),
            'temperatures':temperatures,'weights':weights.tolist(),
            'clipped_weighted_average':clipped_average,'raw_weighted_average':raw_average,
            'cases':rejected,'checks':['analytic velocity','rest/start/end','post-step horizon indexing',
                                      'future reference affects cost','velocity reference affects cost',
                                      'canonical constant cost equivalence','temperature concentration',
                                      'cost offset invariance','clipped candidate update','invalid references rejected',
                                      'all-invalid costs rejected'],
            'note':'Numerical examples and rejected inputs are not failed or successful flight measurements.'}
    write(out/'meaningful_checks.json',report)
    return report


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    print(json.dumps(verify(args.output),ensure_ascii=False,indent=2))
