"""Python reference for target orientation and the PID's initial current command.

The full sampled controller is in web/controller.js; this reference keeps the
initial-condition tables consistent with the first controlled browser state.
"""
import numpy as np
from dynamics import multiply, quaternion


def euler_quaternion_degrees(angles):
    """Roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll), body to world."""
    r, p, y = np.deg2rad(angles)/2
    cr, cp, cy = np.cos([r,p,y]); sr, sp, sy = np.sin([r,p,y])
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def initial_pid_currents(cube, field, omega, settings):
    """At reset, integral and filtered derivative histories are zero."""
    q=quaternion(cube.rotation)
    error_q=multiply(np.asarray(settings['targetQuaternion']),q*np.array([1,-1,-1,-1]))
    error_q/=np.linalg.norm(error_q)
    if error_q[0]<0:
        error_q=-error_q
    length=np.linalg.norm(error_q[1:])
    angle_error=2*error_q[1:] if length<1e-12 else error_q[1:]*2*np.arctan2(length,error_q[0])/length
    rate_error=np.asarray(settings['targetRate'])-omega
    attitude=np.linalg.norm(settings['targetRate'])<1e-12
    requested=settings['kp']*(angle_error if attitude else rate_error)+(settings['kd']*rate_error if attitude else 0)
    field=np.asarray(field)
    squared=field@field
    moment=np.cross(field,requested)/squared if squared>1e-24 else np.zeros(3)
    required=(cube.rotation.T@moment)/(np.array(cube.turns)*np.pi*(cube.side_m*cube.radius_fraction)**2)
    return np.clip(required,-np.array(settings['currentLimits']),np.array(settings['currentLimits']))
