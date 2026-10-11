import numpy as np
from dynamics import Simulation, PlaybackClock, quaternion, rotation
from physics import CubeSat


def test_quaternion_round_trip_including_half_turns():
    for tilt, azimuth in ((0,0),(180,0),(180,180),(90,90),(37,-120)):
        matrix=CubeSat(.1,1,tilt_deg=tilt,azimuth_deg=azimuth).rotation
        np.testing.assert_allclose(rotation(quaternion(matrix)),matrix,atol=1e-15)


def test_free_rotation_and_reset():
    simulation=Simulation(CubeSat(.1,1),[0,0,0],[0,0,.2])
    simulation.advance(2)
    expected=np.array([[np.cos(.4),-np.sin(.4),0],[np.sin(.4),np.cos(.4),0],[0,0,1]])
    np.testing.assert_allclose(simulation.cube.rotation,expected,atol=1e-10)
    np.testing.assert_allclose(simulation.omega,[0,0,.2])
    simulation.reset()
    assert simulation.time==0
    np.testing.assert_allclose(simulation.cube.rotation,np.eye(3),atol=1e-15)


def test_torque_dynamics_and_energy_conservation():
    cube=CubeSat(.1,1,(.1,0,0))
    field=np.array([0,0,40e-6])
    sim=Simulation(cube,field,np.zeros(3))
    initial_energy=-cube.moment@field
    sim.advance(.01)
    assert sim.omega[1]<0
    np.testing.assert_allclose(sim.omega,cube.angular_acceleration(field)*.01,rtol=1e-7,atol=1e-15)
    sim.advance(40)
    energy=.5*sim.inertia*(sim.omega@sim.omega)-sim.cube.moment@field
    assert abs(energy-initial_energy)<1e-12
    np.testing.assert_allclose(sim.cube.rotation.T@sim.cube.rotation,np.eye(3),atol=1e-14)
    assert np.linalg.det(sim.cube.rotation)>0


def test_clock_speed_changes_and_pause_exclude_paused_time():
    clock=PlaybackClock()
    clock.set_playing(True,100)
    clock.tick(102)
    assert clock.target_time==2
    clock.set_speed(10,103)
    clock.tick(104)
    assert clock.target_time==13
    clock.set_playing(False,105)
    assert clock.target_time==23
    clock.tick(200)
    assert clock.target_time==23
    assert clock.wall_elapsed==5
    clock.set_playing(True,201)
    clock.tick(202)
    assert clock.target_time==33


def test_work_budget_never_advances_clock_without_integration():
    sim=Simulation(CubeSat(.1,1),[0,0,40e-6],np.zeros(3))
    advanced=sim.advance(10,max_steps=2)
    assert 0 < advanced < 10
    assert sim.time==advanced
