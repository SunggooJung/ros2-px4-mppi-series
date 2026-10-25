"""11-1 timed ENU references and horizon-aware costs; reuse canonical dynamics."""
import numpy as np
from .controller import MPPIController, NOISE


DURATION = 30.0
PROFILES = {'short_horizon': 10, 'long_horizon': 20}


def reference(times, origin):
    """Rest 3s, one 0.5m-radius circle over 24s, rest 3s.

    Angular speed ramps with a cubic smoothstep over the first/last 4s.
    Its integral is 20s, hence cruise omega=2*pi/20. Position and velocity
    are analytic and continuous. Input/output time is seconds after handover.
    """
    times = np.asarray(times, dtype=float)
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all() or not np.isfinite(times).all():
        raise ValueError('Non-finite or malformed trajectory reference')
    t = np.clip(times - 3., 0., 24.)
    q = np.minimum(t, 4.) / 4.
    ramp_integral = 4. * (q**3 - .5*q**4)
    tail = np.clip((24.-t)/4., 0., 1.)
    progress = np.where(t < 4., ramp_integral,
                        np.where(t <= 20., t-2., 20.-4.*(tail**3-.5*tail**4)))
    speed = np.where(t < 4., 3*q*q-2*q**3,
                     np.where(t <= 20., 1., 3*tail*tail-2*tail**3))
    speed = np.where((times <= 3.) | (times >= 27.), 0., speed)
    angle = progress * (2*np.pi/20.)
    omega = speed * (2*np.pi/20.)
    position = np.stack([origin[0]+.5*(1-np.cos(angle)),
                         origin[1]+.5*np.sin(angle), np.full_like(t, 2.)], axis=-1)
    velocity = np.stack([.5*np.sin(angle)*omega, .5*np.cos(angle)*omega,
                         np.zeros_like(t)], axis=-1)
    return np.concatenate([position, velocity], axis=-1)


def horizon_reference(elapsed, origin, cfg):
    # rollout stores post-integration states, so first reference is t+dt.
    times = elapsed + (np.arange(cfg.horizon)+1)*cfg.dt
    return times, reference(times, origin)


class TrajectoryController(MPPIController):
    def trajectory_cost(self, trajectory, candidates, goal, valid):
        goal = np.asarray(goal, dtype=float)
        if goal.shape != (self.cfg.horizon, 6) or not np.isfinite(goal).all():
            raise ValueError('Each predicted step requires finite ENU position/velocity')
        position = np.sum((trajectory[:, :, :3]-goal[None, :, :3])**2, axis=2)
        velocity = np.sum((trajectory[:, :, 3:6]-goal[None, :, 3:6])**2, axis=2)
        tilt = np.sum((trajectory[:, :, 6:8]/.35)**2, axis=2)
        heading = np.arctan2(np.sin(trajectory[:, :, 8]), np.cos(trajectory[:, :, 8]))**2
        rate = np.sum(trajectory[:, :, 9:12]**2, axis=2)
        deviation = (candidates-self.cfg.hover)/NOISE
        previous = np.concatenate([self.nominal[:1][None, :, :].repeat(len(candidates), axis=0),
                                   candidates[:, :-1]], axis=1)
        change = (candidates-previous)/NOISE
        running = (30*position+3*velocity+.5*tilt+heading+.2*rate
                   +.03*np.sum(deviation**2, axis=2)+.02*np.sum(change**2, axis=2))
        cost = self.cfg.dt*np.sum(running, axis=1)+40*position[:, -1]+5*velocity[:, -1]
        delta = candidates-self.nominal[None, :, :]
        cost += self.cfg.temperature*np.sum((self.nominal-self.cfg.hover)[None, :, :]*delta/(NOISE**2), axis=(1, 2))
        return np.where(valid & np.isfinite(cost), cost, np.inf)

    def step(self, state, goal):
        # The parent's sampling/update method validates a 3-vector. The
        # sentinel supplies that shape; overridden costs uses the full plan.
        goal = np.asarray(goal, dtype=float)
        if goal.shape != (self.cfg.horizon, 6) or not np.isfinite(goal).all():
            raise ValueError('Each predicted step requires finite ENU position/velocity')
        self._reference = goal.copy()
        return super().step(state, goal[0, :3])

    def costs(self, trajectory, candidates, goal, valid):
        return self.trajectory_cost(trajectory, candidates, self._reference, valid)


# Keep the full horizon cost callable independently for source-backed checks.
