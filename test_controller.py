import numpy as np
from controller import euler_quaternion_degrees, initial_pid_currents
from physics import CubeSat


def settings(**kwargs):
    return dict(targetRate=[0,0,0],targetQuaternion=euler_quaternion_degrees([0,30,0]),kp=1e-5,kd=2e-4,currentLimits=[1,1,1],**kwargs)


def test_initial_attitude_pid_command_and_current_limit():
    cube=CubeSat(.1,1)
    config=settings()
    current=initial_pid_currents(cube,[0,0,40e-6],np.zeros(3),config)
    expected=-config['kp']*np.pi/6/(40e-6*100*np.pi*.04**2)
    np.testing.assert_allclose(current,[expected,0,0],atol=1e-15)
    config['currentLimits']=[.01,.01,.01]
    np.testing.assert_allclose(initial_pid_currents(cube,[0,0,40e-6],np.zeros(3),config),[-.01,0,0],atol=1e-15)


def test_initial_rate_pid_ignores_final_angle():
    cube=CubeSat(.1,1)
    config=settings();config['targetRate']=[.1,0,0]
    first=initial_pid_currents(cube,[0,0,40e-6],np.zeros(3),config)
    config['targetQuaternion']=euler_quaternion_degrees([90,80,70])
    np.testing.assert_allclose(initial_pid_currents(cube,[0,0,40e-6],np.zeros(3),config),first)
    np.testing.assert_allclose(initial_pid_currents(cube,[0,0,0],np.zeros(3),config),[0,0,0])
