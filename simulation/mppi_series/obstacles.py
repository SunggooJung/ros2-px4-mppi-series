"""11-2 known vertical cylinder, conservative x500 envelope, soft cost only."""
from dataclasses import dataclass
import numpy as np
from .trajectory import TrajectoryController

DURATION=30.
PROFILES={'weak_cost':{'horizon':20,'weight':2.},
          'inflated_cost':{'horizon':20,'weight':80.},
          'short_horizon':{'horizon':10,'weight':80.}}

@dataclass(frozen=True)
class Obstacle:
    x:float=.85
    y:float=0.
    radius:float=.12
    height:float=5.
    vehicle_radius:float=.40
    margin:float=.10
    influence:float=.25

    def clearance(self, position):
        p=np.asarray(position,dtype=float)
        if p.shape[-1]!=3 or not np.isfinite(p).all():raise ValueError('Finite xyz required')
        return np.linalg.norm(p[...,:2]-[self.x,self.y],axis=-1)-self.radius-self.vehicle_radius

    def penalty(self, position):
        # Vertical column spans the allowed 0.5..4m flight envelope.
        d=self.clearance(position)-self.margin
        return np.maximum(0.,1.-d/self.influence)**2

OBSTACLE=Obstacle()

def reference(times,origin):
    """Straight timed reference, no preplanned detour; 3s rest, 20s ramp, rest.

    Endpoint y=0.12m supplies an explicit weak asymmetry. The straight line
    still intersects the inflated obstacle; it is identical in every trial.
    """
    t=np.asarray(times,dtype=float)
    if not np.isfinite(t).all():raise ValueError('Finite time required')
    q=np.clip((t-3.)/20.,0.,1.)
    a=3*q*q-2*q**3;v=np.where((t>3.)&(t<23.),(6*q-6*q*q)/20.,0.)
    return np.stack([origin[0]+1.7*a,origin[1]+.12*a,np.full_like(a,2.),1.7*v,.12*v,np.zeros_like(v)],axis=-1)

def horizon_reference(elapsed,origin,cfg):
    times=elapsed+(np.arange(cfg.horizon)+1)*cfg.dt
    return times,reference(times,origin)

class ObstacleController(TrajectoryController):
    def __init__(self,cfg,profile,obstacle=OBSTACLE):
        super().__init__(cfg)
        self.profile=profile;self.obstacle=obstacle;self.weight=PROFILES[profile]['weight']

    def trajectory_cost(self,trajectory,candidates,goal,valid):
        base=super().trajectory_cost(trajectory,candidates,goal,valid)
        penalty=self.obstacle.penalty(trajectory[...,:3])
        return base+self.weight*(self.cfg.dt*np.sum(penalty,axis=1)+penalty[:,-1])

    def step(self,state,goal):
        # A weighted mean can exceed a closed floating-point bound by one ULP.
        # Restore the identical physical interval before canonical sampling.
        from .controller import LOW,HIGH
        self.nominal=np.clip(self.nominal,LOW,HIGH)
        return np.clip(super().step(state,goal),LOW,HIGH)
