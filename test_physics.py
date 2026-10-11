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
