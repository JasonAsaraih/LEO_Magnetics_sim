import numpy as np
from physics import Coil, axial_field, coil_field, torque


def test_numerical_field_matches_exact_axis():
    coil = Coil(0.2, 200, 0.04, 67, 32)
    z = np.array([0, 0.02, 0.1])
    actual = coil_field(coil, z[:,None]*coil.basis[2])
    expected = axial_field(coil,z)[:,None]*coil.basis[2]
    np.testing.assert_allclose(actual, expected, rtol=3e-4, atol=1e-12)


def test_torque_orientation_and_scaling():
    aligned = Coil(0.1,100,0.04,0,0)
    np.testing.assert_allclose(torque(aligned,[0,0,40e-6]),0,atol=1e-15)
    transverse = Coil(0.1,100,0.04,90,0)
    np.testing.assert_allclose(torque(transverse,[0,0,40e-6]),[0,-np.linalg.norm(transverse.moment)*40e-6,0],atol=1e-15)
    doubled = Coil(0.2,100,0.04,90,0)
    np.testing.assert_allclose(torque(doubled,[0,0,40e-6]),2*torque(transverse,[0,0,40e-6]))


def test_current_reversal_and_wire_exclusion():
    positive, negative = Coil(), Coil(current_a=-0.1)
    np.testing.assert_allclose(coil_field(positive,[[0,0,0]]),-coil_field(negative,[[0,0,0]]))
    assert np.isnan(coil_field(positive, positive.wire()[0:1])).all()


def test_field_lines_follow_calculated_field():
    from physics import field_lines
    coil = Coil()
    paths = field_lines(coil)
    assert len(paths) == 12
    for path in paths:
        assert np.isfinite(path).all()
        tangent = path[2:] - path[:-2]
        field = coil_field(coil, path[1:-1])
        cosine = np.sum(tangent*field, axis=1)/(np.linalg.norm(tangent, axis=1)*np.linalg.norm(field, axis=1))
        assert np.quantile(cosine, 0.05) > 0.99


def test_field_lines_zero_and_uniform_fields():
    from physics import field_lines
    coil = Coil(current_a=0)
    assert not field_lines(coil)
    paths = field_lines(coil, [0, 0, 40e-6])
    assert paths
    for path in paths:
        np.testing.assert_allclose(path[:,:2], np.broadcast_to(path[0,:2], path[:,:2].shape))
        assert (np.diff(path[:,2]) > 0).all()


def test_normal_vector_orientation():
    import pytest
    for normal in ([1, 2, -3], [0, 0, -1], [0, 1, 0], [1e300, 0, 1e300]):
        coil = Coil.from_normal(0.1, 100, 0.04, normal)
        unit = np.array(normal, dtype=float)/max(abs(v) for v in normal)
        unit /= np.linalg.norm(unit)
        np.testing.assert_allclose(coil.basis[2], unit, atol=1e-15)
        np.testing.assert_allclose(coil.wire()@unit, 0, atol=1e-15)
    for normal in ([0, 0, 0], [float('nan'), 0, 1]):
        with pytest.raises(ValueError):
            Coil.from_normal(0.1, 100, 0.04, normal)


def test_cube_geometry_and_combined_field():
    from physics import CubeSat
    cube = CubeSat(0.1, 1, (0.1, -0.2, 0.3), (100, 150, 200), tilt_deg=37, azimuth_deg=22)
    np.testing.assert_allclose(cube.rotation.T@cube.rotation, np.eye(3), atol=1e-15)
    for i, coil in enumerate(cube.coils):
        np.testing.assert_allclose(coil.center_m, cube.rotation[:,i]*0.05)
        np.testing.assert_allclose((coil.wire()-coil.center_m)@cube.rotation[:,i], 0, atol=1e-15)
    points = np.array([[0,0,0], [0.12,0.08,0.15]])
    np.testing.assert_allclose(cube.field(points), sum(coil_field(c, points) for c in cube.coils))
    expected = cube.rotation@(np.array(cube.currents_a)*cube.turns*np.pi*0.04**2)
    np.testing.assert_allclose(cube.moment, expected)


def test_cube_inertia_and_independent_coils():
    from physics import CubeSat
    cube = CubeSat(0.1, 1, (0.1,0,0))
    np.testing.assert_allclose(cube.inertia, np.eye(3)/600)
    np.testing.assert_allclose(cube.moment, [100*0.1*np.pi*0.04**2,0,0], atol=1e-15)
    doubled_mass = CubeSat(0.1, 2, (0.1,0,0))
    np.testing.assert_allclose(doubled_mass.moment, cube.moment)
    np.testing.assert_allclose(doubled_mass.angular_acceleration([0,0,40e-6]), cube.angular_acceleration([0,0,40e-6])/2)
    doubled_size = CubeSat(0.2, 1, (0.1,0,0))
    np.testing.assert_allclose(doubled_size.inertia, 4*cube.inertia)
    np.testing.assert_allclose(doubled_size.moment, 4*cube.moment)
    off = CubeSat(0.1, 1, (0,0,0))
    np.testing.assert_allclose(off.field([[0,0,0]]), 0)
    assert not off.field_lines()


def test_translated_coil_axis_solution():
    from dataclasses import replace
    coil = replace(Coil(), center_m=(0.02,0.03,0.04))
    z = np.array([0,0.02,0.1])
    points = np.array(coil.center_m)+z[:,None]*coil.basis[2]
    np.testing.assert_allclose(coil_field(coil,points),axial_field(coil,z)[:,None]*coil.basis[2],rtol=3e-4,atol=1e-12)
