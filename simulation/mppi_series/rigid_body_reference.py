"""Chapters 7-1/7-2: SI reference model, separate from PX4 rate-target model.

World ENU; body FLU; R=Rz(yaw)Ry(pitch)Rx(roll), body to world.
Input [T,tau_x,tau_y,tau_z] in N and N m. Instantaneous rotor force,
no motor dynamics or calibrated Gazebo parameters. Never publishes commands.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class SIParams:
    mass: float = 2.0
    inertia: tuple = (0.0216667, 0.0216667, 0.04)
    arm_radius: float = 0.24
    k_f: float = 8.54858e-6  # N/(rad/s)^2, illustrative, not identified
    kappa: float = 0.016  # reaction torque / rotor thrust, m
    omega_max: float = 1000.0  # rad/s

    def __post_init__(self):
        values=np.asarray([self.mass,*self.inertia,self.arm_radius,self.k_f,self.kappa,self.omega_max])
        if not np.isfinite(values).all() or np.any(values<=0):raise ValueError('Positive finite SI parameters required')


def rotation(euler):
    phi,theta,psi=np.asarray(euler,dtype=float)
    cp,sp=np.cos(phi),np.sin(phi);ct,st=np.cos(theta),np.sin(theta);cy,sy=np.cos(psi),np.sin(psi)
    return np.array([[ct*cy,sp*st*cy-cp*sy,cp*st*cy+sp*sy],
                     [ct*sy,sp*st*sy+cp*cy,cp*st*sy-sp*cy],[-st,sp*ct,cp*ct]])


def euler_rates(euler,body_rates):
    phi,theta,_=np.asarray(euler,dtype=float);p,q,r=np.asarray(body_rates,dtype=float)
    if abs(np.cos(theta))<0.1:raise ValueError('Euler chart near singularity; use quaternion integration')
    sp,cp=np.sin(phi),np.cos(phi);ct=np.cos(theta)
    return np.array([p+(sp*q+cp*r)*np.tan(theta),cp*q-sp*r,(sp*q+cp*r)/ct])


def allocation(params=SIParams()):
    a=params.arm_radius/np.sqrt(2)
    # Front-left, front-right, rear-right, rear-left. Sign means BODY yaw
    # reaction, not signed rotor speed or a firmware motor channel number.
    xy=np.array([[a,a],[a,-a],[-a,-a],[-a,a]])
    sigma=np.array([1.,-1.,1.,-1.])
    return np.vstack([np.ones(4),xy[:,1],-xy[:,0],params.kappa*sigma])


def allocate(wrench,params=SIParams()):
    u=np.asarray(wrench,dtype=float)
    if u.shape!=(4,) or not np.isfinite(u).all():raise ValueError('Finite [N,Nm,Nm,Nm] input required')
    forces=np.linalg.solve(allocation(params),u)
    ceiling=params.k_f*params.omega_max**2
    if np.any(forces < -1e-12) or np.any(forces > ceiling+1e-12):
        raise ValueError('Infeasible rotor forces: choose bounded allocation, not silent clipping')
    return np.sqrt(np.maximum(forces,0.)/params.k_f)


def realized_wrench(omega,params=SIParams()):
    w=np.asarray(omega,dtype=float)
    if w.shape!=(4,) or not np.isfinite(w).all() or np.any(w<0) or np.any(w>params.omega_max):
        raise ValueError('Unsigned rotor speed magnitude in [0,omega_max] required')
    return allocation(params)@(params.k_f*w*w)


def step(state,wrench,dt,params=SIParams()):
    s=np.asarray(state,dtype=float);u=np.asarray(wrench,dtype=float)
    if s.shape!=(12,) or u.shape!=(4,) or not np.isfinite(s).all() or not np.isfinite(u).all() or not 0<dt<=0.1:
        raise ValueError('Invalid SI state/input/integration step')
    # Reconstruct the requested wrench from feasible unsigned rotor speeds.
    achieved=realized_wrench(allocate(u,params),params)
    omega=s[9:12];inertia=np.asarray(params.inertia)
    acceleration=rotation(s[6:9])@np.array([0.,0.,achieved[0]/params.mass])+[0.,0.,-9.81]
    omega_dot=(achieved[1:]-np.cross(omega,inertia*omega))/inertia
    out=s.copy();out[:3]+=s[3:6]*dt;out[3:6]+=acceleration*dt
    out[6:9]+=euler_rates(s[6:9],omega)*dt;out[9:12]+=omega_dot*dt
    return out
