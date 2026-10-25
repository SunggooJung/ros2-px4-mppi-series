"""Recheck preserved /89 and /90 examples before using the canonical rate model.

reference_inputs are exact copies of the source-backed chapter_7_audit fragments.
All values here are executed mathematical examples, not Gazebo measurements.
"""
import hashlib
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace

import numpy as np
from .rigid_body_reference import SIParams, allocation, allocate, realized_wrench, rotation, euler_rates, step
from .controller import Config, dynamics


def verify(out):
    root = Path(__file__).parent / 'reference_inputs'
    namespace = {'np': np}
    hashes = {}
    for name in ['89_code_6.txt', '90_code_9.txt', '90_code_5.txt']:
        path = root / name
        hashes['simulation/mppi_series/reference_inputs/' + name] = hashlib.sha256(path.read_bytes()).hexdigest()
        exec(compile(textwrap.dedent(path.read_text()), name, 'exec'), namespace)
    params = SIParams()
    dummy = SimpleNamespace(min_thrust=0., max_thrust=20., arm_length=.24,
                            moment_constant=.016, motor_directions=np.array([1., 1., -1., -1.]),
                            motor_speeds=np.zeros(4), max_rot_velocity=1000.)
    legacy = namespace['motor_model'](dummy, [params.mass * 9.81, 0., 0., 0.])
    old_dynamics = SimpleNamespace(motor_model=lambda u: np.asarray(u),
                                  motor_constant=params.k_f, moment_constant=params.kappa,
                                  arm_length=params.arm_radius, mass=params.mass,
                                  Ixx=params.inertia[0], Iyy=params.inertia[1], Izz=params.inertia[2],
                                  g=np.array([0., 0., -9.81]),
                                  euler_to_rotmat=lambda e: namespace['euler_to_rotmat'](None, e))
    # Probe the published dynamics with four unit unsigned motor speeds to
    # recover its actual wrench mixer, rather than hand-writing its signs.
    mixer_columns = []
    for speeds in np.eye(4):
        response = namespace['dynamics'](old_dynamics, np.zeros(12), speeds, .01)
        force = (response[5] / .01 + 9.81) * params.mass
        torque = response[9:] / .01 * np.asarray(params.inertia)
        mixer_columns.append(np.r_[force, torque])
    legacy_matrix = np.asarray(mixer_columns).T
    legacy_rank = np.linalg.matrix_rank(legacy_matrix)
    # Independent finite-difference Rdot = R [omega]x checks body/Euler rates.
    rng = np.random.default_rng(7)
    records = []
    for _ in range(100):
        angles = rng.uniform([-.7, -.7, -2.], [.7, .7, 2.])
        rates = rng.uniform(-.8, .8, 3)
        R = namespace['euler_to_rotmat'](None, angles)
        skew = np.array([[0., -rates[2], rates[1]], [rates[2], 0., -rates[0]], [-rates[1], rates[0], 0.]])
        h = 1e-7
        rotation_error = max(np.max(abs(R - rotation(angles))), np.max(abs(R.T @ R - np.eye(3))), abs(np.linalg.det(R) - 1))
        corrected_error = np.max(abs((rotation(angles + euler_rates(angles, rates) * h) - R) / h - R @ skew))
        old_next = namespace['dynamics'](old_dynamics, np.r_[np.zeros(6), angles, rates], np.zeros(4), h)
        old_error = np.max(abs((rotation(old_next[6:9]) - R) / h - R @ skew))
        speeds = rng.uniform(600., 800., 4)
        roundtrip_error = np.max(abs(allocate(realized_wrench(speeds, params), params) - speeds))
        state = np.zeros(12); state[6:9] = angles; state[9:] = rates
        cfg = Config()
        si = step(state, [params.mass * 9.81, 0., 0., 0.], cfg.dt, params)
        normalized = dynamics(state, cfg.hover, cfg)
        bridge_error = np.max(abs(si[:9] - normalized[:9]))
        # Torque-free non-principal-axis rotation has an inertia cross term.
        expected_rate = rates - cfg.dt * np.cross(rates, np.asarray(params.inertia) * rates) / params.inertia
        np.testing.assert_allclose(si[9:], expected_rate, atol=1e-13)
        records.append([*angles, *rates, *speeds, rotation_error, corrected_error,
                        old_error, roundtrip_error, bridge_error])
    rows = np.asarray(records)
    np.savetxt(out / 'chapter_7_raw.csv', rows, delimiter=',', comments='',
               header='roll,pitch,yaw,p,q,r,w1,w2,w3,w4,rotation_error,euler_derivative_error,old_derivative_error,allocation_error,bridge_error')
    rejected = 0
    for wrench in [[-1, 0, 0, 0], [100, 0, 0, 0], [1, 10, 0, 0]]:
        try: allocate(wrench)
        except ValueError: rejected += 1
    rank = int(np.linalg.matrix_rank(allocation()))
    if not (rank == 4 and legacy_rank == 3 and rejected == 3 and
            rows[:, 10].max() < 1e-12 and rows[:, 11].max() < 1e-6 and
            rows[:, 13].max() < 1e-9 and rows[:, 14].max() < 1e-12):
        raise RuntimeError('Chapter 7 source-backed model consistency checks failed')
    report = {'passed': True, 'post_ids': [89, 90], 'seed': 7, 'cases': 100,
              'scope': 'SI/reference mathematics; no calibrated motor or flight measurement',
              'source_sha256': hashes, 'old_hover_output_labeled_rad_s': legacy.tolist(),
              'old_implied_thrust_N': float(params.k_f * np.sum(legacy ** 2)),
              'requested_hover_N': params.mass * 9.81, 'old_mixer_rank': int(legacy_rank),
              'old_mixer_from_executed_dynamics': legacy_matrix.tolist(),
              'corrected_mixer_rank': rank,
              'correct_hover_rad_s': allocate([params.mass * 9.81, 0., 0., 0.]).tolist(),
              'rotation_max_error': float(rows[:, 10].max()),
              'correct_euler_derivative_max_error': float(rows[:, 11].max()),
              'old_body_as_euler_derivative_max_error': float(rows[:, 12].max()),
              'allocation_roundtrip_max_error_rad_s': float(rows[:, 13].max()),
              'si_normalized_translation_euler_max_error': float(rows[:, 14].max()),
              'inertia_cross_term_checked_cases': len(rows), 'infeasible_requests_rejected': rejected,
              'units': {'wrench': 'N,N m,N m,N m', 'motor_speed': 'rad/s unsigned',
                        'k_f': 'N/(rad/s)^2', 'kappa': 'm', 'normalized_input': '1,rad/s,rad/s,rad/s'},
              'frames': {'world': 'ENU', 'body': 'FLU', 'rotation': 'body to world ZYX'},
              'old_post_server_revision_applied': False}
    (out / 'chapter_7_checks.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report
