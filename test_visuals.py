"""Regression checks for physical-vector scaling and field composition."""
import numpy as np
from physics import CubeSat
from visuals import scene


def displacement(fig, name):
    trace = next(t for t in fig.data if t.name == name)
    return np.array([trace.x[-1]-trace.x[0], trace.y[-1]-trace.y[0], trace.z[-1]-trace.z[0]])


def test_magnitude_controls_vector_lengths():
    base = scene(CubeSat(.1,1), [0,0,40e-6], show_lines=False)
    current = scene(CubeSat(.1,1,(.2,.2,.2)), [0,0,40e-6], show_lines=False)
    field = scene(CubeSat(.1,1), [0,0,80e-6], show_lines=False)
    for name in ('m X','m Y','m Z','m net','Torque τ'):
        np.testing.assert_allclose(displacement(current,name), 2*displacement(base,name))
    for name in ('B external','Torque τ'):
        np.testing.assert_allclose(displacement(field,name), 2*displacement(base,name))
    reversed_coil = scene(CubeSat(.1,1,(-.1,0,0)), [0,0,40e-6], show_lines=False)
    np.testing.assert_allclose(displacement(reversed_coil,'m X'), -displacement(base,'m X'))


def test_external_field_always_included_and_zero_moment():
    fig = scene(CubeSat(.1,1,(0,0,0)), [0,0,40e-6], show_lines=False)
    arrows = next(t for t in fig.data if t.name == 'Field direction')
    assert len(arrows.z) > 0
    np.testing.assert_allclose(arrows.u, 0)
    np.testing.assert_allclose(arrows.v, 0)
    assert (np.array(arrows.w) > 0).all()
    np.testing.assert_allclose(displacement(fig,'m net'), 0)
    np.testing.assert_allclose(displacement(fig,'Torque τ'), 0)
