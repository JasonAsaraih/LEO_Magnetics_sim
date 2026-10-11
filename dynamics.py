"""Attitude propagation and wall-clock playback, independent of the UI.

Uniform cube inertia is scalar, so world-frame omega obeys J omega_dot = tau.
Quaternion q maps body vectors into world coordinates. RK4 evolves q and omega.
"""
from dataclasses import dataclass, replace
import numpy as np


def multiply(a, b):
    return np.r_[a[0]*b[0]-a[1:]@b[1:], a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]


def rotation(q):
    w,x,y,z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def quaternion(matrix):
    # Robust for 180-degree rotations as well as identity.
    m = np.asarray(matrix)
    if np.trace(m) > 0:
        s = np.sqrt(1+np.trace(m))*2
        q = np.array([s/4,(m[2,1]-m[1,2])/s,(m[0,2]-m[2,0])/s,(m[1,0]-m[0,1])/s])
    else:
        i = int(np.argmax(np.diag(m))); j=(i+1)%3; k=(i+2)%3
        s = np.sqrt(1+m[i,i]-m[j,j]-m[k,k])*2
        q = np.zeros(4)
        q[0]=(m[k,j]-m[j,k])/s
        q[i+1]=s/4; q[j+1]=(m[j,i]+m[i,j])/s; q[k+1]=(m[k,i]+m[i,k])/s
    return q/np.linalg.norm(q)


@dataclass
class Simulation:
    initial_cube: object
    external_field: np.ndarray
    initial_omega: np.ndarray

    def __post_init__(self):
        self.external_field=np.array(self.external_field,dtype=float)
        self.initial_omega=np.array(self.initial_omega,dtype=float)
        self.body_moment=self.initial_cube.rotation.T@self.initial_cube.moment
        self.inertia=self.initial_cube.inertia[0,0]
        self.reset()

    def reset(self):
        self.state=np.r_[quaternion(self.initial_cube.rotation),self.initial_omega]
        self.time=0.0

    @property
    def cube(self):
        return replace(self.initial_cube, attitude=tuple(rotation(self.state[:4]).ravel()))

    @property
    def omega(self):
        return self.state[4:]

    def derivative(self, state):
        q,omega=state[:4],state[4:]
        moment=rotation(q)@self.body_moment
        acceleration=np.cross(moment,self.external_field)/self.inertia
        return np.r_[0.5*multiply(np.r_[0,omega],q),acceleration]

    def advance(self, duration, max_steps=2000):
        """Bound rotation per step; return simulated duration actually integrated.

        A per-refresh work limit leaves remaining clock time pending, never
        silently discards it or advances the displayed clock without physics.
        """
        if not np.isfinite(duration) or duration < 0:
            raise ValueError('Duration must be finite and nonnegative.')
        advanced=0.0
        frequency=np.sqrt(np.linalg.norm(self.body_moment)*np.linalg.norm(self.external_field)/self.inertia)
        for _ in range(max_steps):
            remaining=duration-advanced
            if remaining <= 1e-12:
                break
            step=min(remaining,0.05,0.03/max(np.linalg.norm(self.omega),frequency,1e-12))
            y=self.state
            k1=self.derivative(y); k2=self.derivative(y+step*k1/2)
            k3=self.derivative(y+step*k2/2); k4=self.derivative(y+step*k3)
            self.state=y+step*(k1+2*k2+2*k3+k4)/6
            self.state[:4]/=np.linalg.norm(self.state[:4])
            advanced+=step
        self.time+=advanced
        return advanced


@dataclass
class PlaybackClock:
    speed: float = 1.0
    playing: bool = False
    target_time: float = 0.0
    wall_elapsed: float = 0.0
    last_wall: float | None = None

    def tick(self, now):
        if self.playing and self.last_wall is not None:
            elapsed=max(0.0,now-self.last_wall)
            self.target_time+=elapsed*self.speed
            self.wall_elapsed+=elapsed
        self.last_wall=now

    def set_playing(self, playing, now):
        self.tick(now)
        self.playing=playing

    def set_speed(self, speed, now):
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError('Time scale must be finite and positive.')
        self.tick(now)
        self.speed=speed
