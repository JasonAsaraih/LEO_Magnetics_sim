"""SI-unit, air-core circular-coil model. No UI dependencies."""
from dataclasses import dataclass
import numpy as np

MU0 = 4 * np.pi * 1e-7  # H/m

@dataclass(frozen=True)
class Coil:
    current_a: float = 0.1
    turns: int = 100
    radius_m: float = 0.04
    tilt_deg: float = 45
    azimuth_deg: float = 0

    def __post_init__(self):
        if self.radius_m <= 0 or self.turns < 1:
            raise ValueError('Radius must be positive and turns must be at least one.')

    @property
    def basis(self):
        theta, phi = np.deg2rad([self.tilt_deg, self.azimuth_deg])
        normal = np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)])
        u = np.array([np.cos(theta)*np.cos(phi), np.cos(theta)*np.sin(phi), -np.sin(theta)])
        return u, np.cross(normal, u), normal

    @property
    def moment(self):
        return self.turns * self.current_a * np.pi * self.radius_m**2 * self.basis[2]

    def wire(self, segments=240):
        angles = np.linspace(0, 2*np.pi, segments+1)
        u, v, _ = self.basis
        return self.radius_m * (np.cos(angles)[:, None]*u + np.sin(angles)[:, None]*v)


def coil_field(coil, points, segments=240, exclusion_m=0.002):
    """Midpoint Biot–Savart quadrature; NaN within exclusion of ideal wire.

    B = μ₀ N I / (4π) ∮ dl × (r-r') / |r-r'|³.
    All turns are coincident; finite winding thickness is not modeled.
    """
    points = np.asarray(points, dtype=float)
    wire = coil.wire(segments)
    dl = np.diff(wire, axis=0)
    mid = (wire[1:] + wire[:-1])/2
    r = points[..., None, :] - mid
    distance = np.linalg.norm(r, axis=-1)
    # Distance to line segments, rather than just their midpoints.
    fraction = np.clip(np.sum((points[..., None, :]-wire[:-1])*dl, axis=-1)/np.sum(dl*dl, axis=-1), 0, 1)
    closest = wire[:-1] + fraction[..., None]*dl
    excluded = np.any(np.linalg.norm(points[..., None, :]-closest, axis=-1) < exclusion_m, axis=-1)
    with np.errstate(divide='ignore', invalid='ignore'):
        field = MU0*coil.turns*coil.current_a/(4*np.pi)*np.sum(np.cross(dl, r)/distance[..., None]**3, axis=-2)
    return np.where(excluded[..., None], np.nan, field)


def torque(coil, external_field_t):
    """τ = m × B_external (N·m). The coil's own field supplies no net torque."""
    return np.cross(coil.moment, np.asarray(external_field_t))


def axial_field(coil, z_m):
    """Exact circular-loop field along its normal, relative to coil center."""
    return MU0*coil.turns*coil.current_a*coil.radius_m**2/(2*(coil.radius_m**2+np.asarray(z_m)**2)**1.5)
