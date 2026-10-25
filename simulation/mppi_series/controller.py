"""Chapter 9-1: executable educational path-integral sampling controller.

State: position/velocity ENU, ZYX Euler angles, body rates FLU (12 values).
Input: normalized collective thrust, desired body rates FLU [c,p_sp,q_sp,r_sp].
The first-order rate response and linear local thrust map are explicit model
assumptions, NOT an identified PX4 model. This file does not command a vehicle.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Config:
    samples:int=500
    horizon:int=20
    dt:float=.05
    temperature:float=1.
    thrust_hover:float=.6
    rate_tau:float=.08
    mass:float=2.
    seed:int=7

    def __post_init__(self):
        if type(self.samples) is not int or self.samples<2 or type(self.horizon) is not int or self.horizon<1:
            raise ValueError('Invalid sample count/horizon')
        if not all(math.isfinite(v) for v in [self.dt,self.temperature,self.thrust_hover,self.rate_tau,self.mass]):
            raise ValueError('Non-finite model parameters')
        if not 0<self.dt<=.1 or self.temperature<=0 or not .1<self.thrust_hover<.9 or self.rate_tau<=0 or self.mass<=0:
            raise ValueError('Invalid model parameters')

    @property
    def hover(self):return np.array([self.thrust_hover,0.,0.,0.])


LOW=np.array([.1,-1.5,-1.5,-1.])
HIGH=np.array([.9,1.5,1.5,1.])
NOISE=np.array([.06,.25,.25,.20])


def _inputs(state,control,batch):
    state=np.asarray(state,dtype=float);control=np.asarray(control,dtype=float)
    if (batch and (state.ndim!=2 or state.shape[1]!=12 or control.shape!=(len(state),4))) or (not batch and (state.shape!=(12,) or control.shape!=(4,))):
        raise ValueError('State/control shape mismatch')
    if not np.isfinite(state).all() or not np.isfinite(control).all():raise ValueError('Non-finite state/input')
    if np.any(control<LOW) or np.any(control>HIGH):raise ValueError('Input outside defined bounds')
    return state,control


def dynamics(state,control,cfg=Config()):
    """Independent scalar implementation for model/reference checks."""
    s,u=_inputs(state,control,False)
    phi,theta,psi=s[6:9];p,q,r=s[9:12]
    if abs(math.cos(theta))<.1:raise ValueError('Euler coordinate singularity')
    sp,cp=math.sin(phi),math.cos(phi);st,ct=math.sin(theta),math.cos(theta)
    sy,cy=math.sin(psi),math.cos(psi)
    up=np.array([cp*st*cy+sp*sy,cp*st*sy-sp*cy,cp*ct])
    # T(c) = m*g*c/c_hover. Local assumed map, not measured motor thrust.
    acceleration=up*9.81*u[0]/cfg.thrust_hover+np.array([0.,0.,-9.81])
    euler_rate=np.array([p+(sp*q+cp*r)*st/ct,cp*q-sp*r,(sp*q+cp*r)/ct])
    out=s.copy();out[:3]+=s[3:6]*cfg.dt;out[3:6]+=acceleration*cfg.dt
    out[6:9]+=euler_rate*cfg.dt
    # Exact one-step solution of omega_dot=(omega_sp-omega)/tau.
    out[9:12]=u[1:]+(s[9:12]-u[1:])*math.exp(-cfg.dt/cfg.rate_tau)
    return out


def dynamics_batch(state,control,cfg=Config()):
    """Vectorized counterpart; invalid trajectories remain marked by NaN."""
    s,u=_inputs(state,control,True)
    phi,theta,psi=s[:,6:9].T;p,q,r=s[:,9:12].T
    sp,cp=np.sin(phi),np.cos(phi);st,ct=np.sin(theta),np.cos(theta)
    sy,cy=np.sin(psi),np.cos(psi)
    invalid=np.abs(ct)<.1
    safe_ct=np.where(invalid,np.nan,ct)
    up=np.column_stack([cp*st*cy+sp*sy,cp*st*sy-sp*cy,cp*ct])
    acceleration=up*(9.81*u[:,0]/cfg.thrust_hover)[:,None]+[0.,0.,-9.81]
    euler_rate=np.column_stack([p+(sp*q+cp*r)*st/safe_ct,cp*q-sp*r,(sp*q+cp*r)/safe_ct])
    out=s.copy();out[:,:3]+=s[:,3:6]*cfg.dt;out[:,3:6]+=acceleration*cfg.dt
    out[:,6:9]+=euler_rate*cfg.dt
    out[:,9:12]=u[:,1:]+(s[:,9:12]-u[:,1:])*np.exp(-cfg.dt/cfg.rate_tau)
    out[invalid]=np.nan
    return out


class MPPIController:
    """Clipped sampling controller; not a claim of exact MPPI equivalence.

    Cost uses scaled squared error and hover-relative input/variation terms.
    Gaussian proposal correction is included; clipping and a deterministic
    nominal candidate change the proposal, so exact path-integral assumptions
    are not asserted. Model consistency is tested separately from flight.
    """
    def __init__(self,cfg=Config()):
        self.cfg=cfg;self.rng=np.random.default_rng(cfg.seed)
        self.nominal=np.tile(cfg.hover,(cfg.horizon,1));self.last=None

    def rollout(self,state,candidates):
        state=np.asarray(state,dtype=float);candidates=np.asarray(candidates,dtype=float)
        if state.shape!=(12,) or candidates.ndim!=3 or candidates.shape[1:]!=(self.cfg.horizon,4):raise ValueError('Rollout shape mismatch')
        if not np.isfinite(state).all() or not np.isfinite(candidates).all():raise ValueError('Non-finite rollout input')
        if np.any(candidates<LOW) or np.any(candidates>HIGH):raise ValueError('Candidate outside bounds')
        current=np.tile(state,(len(candidates),1));trajectory=[];valid=np.ones(len(candidates),dtype=bool)
        for t in range(self.cfg.horizon):
            # Invalid rows cannot feed NaN back into another valid candidate.
            current=dynamics_batch(np.where(valid[:,None],current,0.),candidates[:,t],self.cfg)
            valid &= np.isfinite(current).all(axis=1)&(np.abs(current[:,7])<1.2)&(np.abs(current[:,6])<1.2)
            trajectory.append(current.copy())
        return np.stack(trajectory,axis=1),valid

    def costs(self,trajectory,candidates,goal,valid):
        position=np.sum((trajectory[:,:,:3]-goal[None,None,:])**2,axis=2)
        velocity=np.sum(trajectory[:,:,3:6]**2,axis=2)
        tilt=np.sum((trajectory[:,:,6:8]/.35)**2,axis=2)
        heading=np.arctan2(np.sin(trajectory[:,:,8]),np.cos(trajectory[:,:,8]))**2
        rate=np.sum(trajectory[:,:,9:12]**2,axis=2)
        deviation=(candidates-self.cfg.hover)/NOISE
        previous=np.concatenate([self.nominal[:1][None,:,:].repeat(len(candidates),axis=0),candidates[:,:-1]],axis=1)
        change=(candidates-previous)/NOISE
        running=30*position+3*velocity+.5*tilt+heading+.2*rate+.03*np.sum(deviation**2,axis=2)+.02*np.sum(change**2,axis=2)
        cost=self.cfg.dt*np.sum(running,axis=1)+40*position[:,-1]+5*velocity[:,-1]
        # Shifted-Gaussian proposal correction, using evaluated clipped delta.
        delta=candidates-self.nominal[None,:,:]
        correction=self.cfg.temperature*np.sum((self.nominal-self.cfg.hover)[None,:,:]*delta/(NOISE**2),axis=(1,2))
        cost+=correction
        return np.where(valid&np.isfinite(cost),cost,np.inf)

    @staticmethod
    def weights(costs,temperature):
        costs=np.asarray(costs,dtype=float);finite=np.isfinite(costs)
        if not finite.any():raise ValueError('All rollouts invalid; no command returned')
        weights=np.zeros(len(costs));weights[finite]=np.exp(-(costs[finite]-costs[finite].min())/temperature)
        weights/=weights.sum()
        return weights

    def step(self,state,goal):
        state=np.asarray(state,dtype=float);goal=np.asarray(goal,dtype=float)
        if state.shape!=(12,) or goal.shape!=(3,) or not np.isfinite(state).all() or not np.isfinite(goal).all():raise ValueError('Invalid state/goal')
        if abs(state[6])>=1.2 or abs(state[7])>=1.2:raise ValueError('Outside model tilt range')
        noise=self.rng.normal(size=(self.cfg.samples,self.cfg.horizon,4))*NOISE
        candidates=np.clip(self.nominal[None,:,:]+noise,LOW,HIGH)
        candidates[0]=self.nominal
        trajectory,valid=self.rollout(state,candidates)
        cost=self.costs(trajectory,candidates,goal,valid)
        weights=self.weights(cost,self.cfg.temperature)
        updated=np.einsum('n,ntc->tc',weights,candidates)
        command=updated[0].copy()
        self.last={'costs':cost.copy(),'weights':weights.copy(),'candidate_count':len(cost),
                   'valid_count':int(np.isfinite(cost).sum()),'updated_sequence':updated.copy()}
        self.nominal[:-1]=updated[1:];self.nominal[-1]=self.cfg.hover
        return command


def attitude_target_fields(command):
    """Message-field contract only; no publisher or arming/mode calls."""
    _,u=_inputs(np.zeros(12),command,False)
    return {'type_mask':128,'orientation_xyzw':[0.,0.,0.,1.],
            'body_rate_flu_rad_s':u[1:].tolist(),'thrust_01':float(u[0])}
