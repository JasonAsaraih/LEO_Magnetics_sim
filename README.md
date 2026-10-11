# CubeSat Magnetorquer Lab

Interactive 3D field and torque calculator for three orthogonal air-core circular coils mounted on the +X, +Y, and +Z faces of a cube.

## Run

Python 3.12 is validated. From this checkout:

```bash
python -m venv /workspace/leo-magnetics-env/venv
/workspace/leo-magnetics-env/venv/bin/python -m pip install -r requirements.txt
/workspace/leo-magnetics-env/venv/bin/python -m streamlit run app.py --server.headless=true --browser.gatherUsageStats=false
```

Streamlit normally listens on port 8501. Run locally or use your development platform's supported port access to view the UI; cloud onboarding does not provide web previews.

## Controls and calculations

- Set cube side length and total mass. Rotational inertia assumes a uniform solid cube: J = ML²/6 on each axis about the center of mass.
- Set current (including negative/zero) and turns independently for the X, Y, and Z coils.
- Coil radius is a configurable fraction of cube side length, below half the side. Coils stay centered on three perpendicular faces.
- Rotate the whole cube with tilt/azimuth or a body +Z normal vector. The vector is normalized; zero vectors are rejected. Body X/Y follow the deterministic orthogonal basis displayed in the calculations tab. At zero tilt and azimuth, body and world axes coincide.
- Set the external field's world-coordinate x/y/z components in µT.
- Toggle field lines and field arrows independently. Hiding arrows also hides magnitude dots and the color scale. Arrow size controls actual raw Plotly cone lengths.
- Net magnetic moment is the vector sum of NIπR² times each coil normal. Torque is m_net × B_external. Angular acceleration is J⁻¹τ. Mass changes inertia/acceleration, not magnetic moment.

The external field vector is teal; net torque is magenta. Their displayed lengths are normalized and labels show actual magnitudes. Coils are orange (X), green (Y), and violet (Z). A current sweep shows net torque versus X-coil current while Y/Z remain fixed. Equations, per-coil and combined vectors, body axes, and CSV export appear in the calculations tab.

## Code and validation

- `physics.py`: UI-independent SI equations; coil geometry including face offsets; Biot–Savart field integration; CubeSat geometry, inertia, and superposition; shared field-line tracing.
- `visuals.py`: cube, coils, field visualization, and labeled vector rendering.
- `app.py`: controls and calculation review.
- `test_physics.py`: analytical field, torque, normal vectors, geometry, superposition, inertia and scaling checks.

```bash
/workspace/leo-magnetics-env/venv/bin/python -m pytest -q
```

Fields use 240 segments per circular winding. Field lines integrate dr/ds = B/|B| using midpoint integration in both directions from coil-plane seeds. They trace the combined coil field plus the external field when selected. Traces stop at the display boundary, near energized wires, at a field null, when returning to a seed, or after 360 steps per direction. Seed spacing is illustrative, not flux density. Points within 2 mm of energized ideal wires are omitted.

Assumptions: uniform solid cube mass distribution, coincident circular turns, vacuum permeability, and a uniform external field. No core saturation, heating, resistance, orbital model, or time integration of attitude. Coils' own fields do not provide net external torque. Magnetorquers cannot produce torque along the external field. This is an instantaneous engineering calculator, not a flight qualification model.
