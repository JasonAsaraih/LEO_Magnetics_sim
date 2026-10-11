"""SI-unit, air-core circular-coil model. No UI dependencies."""
from dataclasses import dataclass, replace
import numpy as np

MU0 = 4 * np.pi * 1e-7  # H/m

@dataclass(frozen=True)
class Coil:
    current_a: float = 0.1
    turns: int = 100
    radius_m: float = 0.04
    tilt_deg: float = 45
    azimuth_deg: float = 0
    center_m: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        if self.radius_m <= 0 or self.turns < 1:
            raise ValueError('Radius must be positive and turns must be at least one.')

    @classmethod
    def from_normal(cls, current_a, turns, radius_m, normal):
        """Orient the coil from any finite, nonzero normal vector (x, y, z)."""
        normal = np.asarray(normal, dtype=float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or not np.any(normal):
            raise ValueError('Coil normal must be a finite, nonzero x/y/z vector.')
        # Scale before normalizing to avoid overflow for large components.
        unit = normal / np.max(np.abs(normal))
        unit /= np.linalg.norm(unit)
        tilt = np.rad2deg(np.arccos(np.clip(unit[2], -1, 1)))
        azimuth = np.rad2deg(np.arctan2(unit[1], unit[0]))
        return cls(current_a, turns, radius_m, float(tilt), float(azimuth))

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
        return np.asarray(self.center_m) + self.radius_m * (np.cos(angles)[:, None]*u + np.sin(angles)[:, None]*v)


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


def field_lines(coil, external_field_t=None, extent=None, steps=360):
    """Trace dr/ds = B/|B| using midpoint integration in both directions.

    Seeds lie inside the coil in four meridional planes. Stop at the
    display boundary, near the wire, at a field null, or on returning to
    a seed. Seed density is illustrative, not a measure of magnetic flux.
    """
    extent = max(0.12, coil.radius_m*3) if extent is None else extent
    external = np.zeros(3) if external_field_t is None else np.asarray(external_field_t)
    u, v, _ = coil.basis
    seeds = np.array([np.asarray(coil.center_m)+coil.radius_m*f*(np.cos(a)*u+np.sin(a)*v)
                      for a in np.linspace(0, 2*np.pi, 4, endpoint=False)
                      for f in (0.25, 0.5, 0.75)])
    step = min(coil.radius_m/24, extent/100)

    return trace_field_lines(lambda points: coil_field(coil, points)+external, seeds, extent, step, steps)


def trace_field_lines(field_function, seeds, extent, step, steps=360):
    """Midpoint integration shared by single-coil and multi-coil models."""
    def direction(points):
        field = field_function(points)
        magnitude = np.linalg.norm(field, axis=-1)
        valid = np.isfinite(magnitude) & (magnitude > 1e-20)
        return np.divide(field, magnitude[:, None], out=np.zeros_like(field), where=valid[:, None]), valid

    branches = []
    for sign in (-1, 1):
        points = seeds.copy()
        active = np.ones(len(seeds), dtype=bool)
        paths = [[seed.copy()] for seed in seeds]
        for iteration in range(steps):
            if not active.any():
                break
            d1, valid1 = direction(points)
            d2, valid2 = direction(points+sign*step*d1/2)
            candidate = points+sign*step*d2
            _, valid3 = direction(candidate)
            active &= valid1 & valid2 & valid3 & (np.abs(candidate) <= extent).all(axis=1)
            for index in np.flatnonzero(active):
                paths[index].append(candidate[index].copy())
            closed = (iteration > 20) & (np.linalg.norm(candidate-seeds, axis=1) < step*0.8)
            active &= ~closed
            points = candidate
        branches.append(paths)
    return [np.array(backward[:0:-1]+forward) for backward, forward in zip(*branches)
            if len(backward)+len(forward) > 3]


@dataclass(frozen=True)
class CubeSat:
    """Uniform solid cube with circular coils on its +X, +Y, +Z faces.

    Body axes are columns of rotation. Coil centers are displaced from
    the center of mass; torque uses a uniform external field.
    """
    side_m: float
    mass_kg: float
    currents_a: tuple = (0.1, 0.1, 0.1)
    turns: tuple = (100, 100, 100)
    radius_fraction: float = 0.4
    tilt_deg: float = 0
    azimuth_deg: float = 0

    def __post_init__(self):
        if not np.isfinite([self.side_m, self.mass_kg, self.radius_fraction]).all() or self.side_m <= 0 or self.mass_kg <= 0:
            raise ValueError('Cube size and mass must be finite and positive.')
        if not 0 < self.radius_fraction < 0.5:
            raise ValueError('Coil radius must be less than half the cube side.')
        if len(self.currents_a) != 3 or len(self.turns) != 3 or not np.isfinite(self.currents_a).all() or any(n < 1 or int(n) != n for n in self.turns):
            raise ValueError('Provide three finite currents and three positive integer turn counts.')

    @property
    def rotation(self):
        return np.column_stack(Coil(tilt_deg=self.tilt_deg, azimuth_deg=self.azimuth_deg).basis)

    @property
    def coils(self):
        result = []
        for index in range(3):
            normal = self.rotation[:, index]
            coil = Coil.from_normal(self.currents_a[index], self.turns[index], self.side_m*self.radius_fraction, normal)
            result.append(replace(coil, center_m=tuple(normal*self.side_m/2)))
        return result

    @property
    def moment(self):
        return np.sum([coil.moment for coil in self.coils], axis=0)

    @property
    def inertia(self):
        """Inertia about center of mass; identical on all axes (kg m²)."""
        return np.eye(3)*self.mass_kg*self.side_m**2/6

    def field(self, points):
        # Inactive coils contribute exactly zero and have no singular field.
        terms = [coil_field(c, points) for c in self.coils if c.current_a != 0]
        return np.sum(terms, axis=0) if terms else np.zeros_like(np.asarray(points, dtype=float))

    def angular_acceleration(self, external_field_t):
        return np.linalg.solve(self.inertia, torque(self, external_field_t))

    def field_lines(self, external_field_t=None, extent=None):
        extent = self.side_m*2 if extent is None else extent
        external = np.zeros(3) if external_field_t is None else np.asarray(external_field_t)
        seeds = []
        for coil in self.coils:
            u, v, _ = coil.basis
            for angle in np.linspace(0, 2*np.pi, 4, endpoint=False):
                seeds.append(np.asarray(coil.center_m)+coil.radius_m*0.5*(np.cos(angle)*u+np.sin(angle)*v))
        return trace_field_lines(lambda p: self.field(p)+external, np.array(seeds), extent, self.side_m/60)
