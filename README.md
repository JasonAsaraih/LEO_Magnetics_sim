# CubeSat Magnetorquer Lab

Interactive 3D field, torque, and attitude simulator for three orthogonal air-core circular coils mounted on the +X, +Y, and +Z faces of a cube.

## Run

Python 3.12 is validated. From this checkout:

```bash
python -m venv /workspace/leo-magnetics-env/venv
/workspace/leo-magnetics-env/venv/bin/python -m pip install -r requirements.txt
/workspace/leo-magnetics-env/venv/bin/python -m streamlit run app.py --server.headless=true --browser.gatherUsageStats=false
```

Streamlit normally listens on port 8501. Run locally or use your development platform's supported port access to view the UI; cloud onboarding does not provide web previews.

## Controls and calculations

- Set cube side length and total mass in the initial-conditions form. Rotational inertia assumes a uniform solid cube: J = ML²/6 on each axis about the center of mass.
- Set current (including negative/zero) and turns independently for the X, Y, and Z coils.
- Coil radius is a configurable fraction of cube side length, below half the side. Coils stay centered on three perpendicular faces.
- Rotate the whole cube with tilt/azimuth or a body +Z normal vector. The vector is normalized; zero vectors are rejected. Body X/Y follow the deterministic orthogonal basis displayed in the calculations tab. At zero tilt and azimuth, body and world axes coincide.
- Set the external field's world-coordinate x/y/z components in µT.
- Toggle field lines and field arrows independently. Hiding arrows also hides magnitude dots and the color scale. Arrow size controls actual raw Plotly cone lengths.
- Net magnetic moment is the vector sum of NIπR² times each coil normal. Torque is m_net × B_external. Angular acceleration is J⁻¹τ. Mass changes inertia/acceleration, not magnetic moment.

The external field vector is teal; net torque is magenta. All moment, torque, and external-field vector lengths scale linearly with magnitude. Coil-colored arrows show individual moments and a gold arrow shows the net moment. All moment vectors share a common scale. Separate scales apply to unlike units: 0.1 A·m², 40 µT, or 4 µN·m each span 60% of the field-view half-width at vector scale 1. Use the vector scale slider to shrink large vectors that extend beyond the view; zero vectors have zero length and a zero label. Coils are orange (X), green (Y), and violet (Z). A current sweep shows net torque versus X-coil current while Y/Z remain fixed. Equations, per-coil and combined vectors, body axes, and CSV export appear in the calculations tab.

## Code and validation

- `physics.py`: UI-independent SI equations; coil geometry including face offsets; Biot–Savart field integration; CubeSat geometry, inertia, and superposition; shared field-line tracing.
- `visuals.py`: cube, coils, field visualization, and labeled vector rendering.
- `app.py`: controls and calculation review.
- `test_physics.py`: analytical field, torque, normal vectors, geometry, superposition, inertia and scaling checks.

```bash
/workspace/leo-magnetics-env/venv/bin/python -m pytest -q
```

Fields use 240 segments per circular winding. Field lines integrate dr/ds = B/|B| using midpoint integration in both directions from 12 coil-plane seeds (4 per coil). They trace the combined coil field plus the external field at all times (there is no coil-only display toggle). Traces stop at the display boundary, near energized wires, at a field null, when returning to a seed, or after 360 steps per direction. Seed spacing is illustrative, not flux density. Points within 2 mm of energized ideal wires are omitted.

Assumptions: uniform solid cube mass distribution, coincident circular turns, vacuum permeability, and a uniform external field. No core saturation, heating, resistance, damping, feedback controller, or orbital field model. Coils' own fields do not provide net external torque. Magnetorquers cannot produce torque along the external field. This is an instantaneous engineering calculator, not a flight qualification model.


## Time simulation

1. Configure cube geometry/mass, coil currents/turns, initial orientation, initial angular velocity (world-frame degrees per second), and external magnetic field.
2. Press **Apply initial conditions / Reset** to load the form and return to t = 0, paused. Editing the form alone does not alter a running simulation.
3. Press **Play**. The cube rotates under net magnetorquer torque; currents remain constant in the body frame and the external field stays fixed in the world frame. **Pause** freezes playback; **Play** resumes. **Reset to applied initial conditions** restores the last submitted configuration, not unsubmitted form edits.
4. Set **Time scale** live: 10× advances 10 simulated seconds per active real second. The display shows integrated simulation time, active real time (excluding pauses), time scale, and status. Display controls remain live.

`dynamics.py` contains the quaternion attitude state, fourth-order Runge–Kutta integration, and playback clock. With isotropic cube inertia, world angular velocity follows J ω̇ = m_world × B_external. Quaternion orientation is normalized each step, and step size is bounded by angular velocity and torque frequency. Acceleration and torque are recomputed as orientation changes. A live angular-speed history shows the most recent 2,000 samples.

Wall time uses a monotonic clock; speed changes settle elapsed time at the old speed first. Frames update approximately every 0.25 s while playing, subject to rendering time. A per-frame integration budget preserves outstanding simulation time; if computation falls behind, the displayed clock reports actual integrated time and a warning shows the pending duration. Reduce time scale if needed. Playback requires an active app session; browser refresh/session loss does not persist the trajectory.

While playing, field-line visualization uses trilinear interpolation of the calculated 9×9×9 field grid for responsiveness. Paused lines use direct Biot–Savart tracing. Both use the previous 12 seeds, always including the external field. This display approximation never enters the attitude equations. Camera position is retained across frames.
