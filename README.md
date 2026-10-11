# CubeSat Magnetorquer Lab

Interactive field and attitude simulation for three orthogonal air-core circular coils mounted on the +X, +Y, and +Z faces of a uniform cube.

## Run locally

Python 3.12 is validated. From this checkout:

```bash
python -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m streamlit run app.py
```

Streamlit hosts the initial-condition form and equation review. Live simulation runs on your computer in the browser, with no server round trips during playback. **Download standalone player (HTML)** exports a self-contained player using the applied initial conditions. Open that file in a modern browser to run without Streamlit or an Internet connection. If your browser blocks local HTML, serve the containing folder with `python -m http.server 8000` and open the file through that local server. The standalone file uses its exported initial conditions; change them in Streamlit and export again for a new configuration.

The bundled browser player is committed, so Node is not required to run the app. After editing browser source, use Node 20+:

```bash
npm ci
npm run build
npm test
```

This regenerates `web/dist/player.html`. It bundles Three.js and the worker into one HTML file; there are no runtime CDN requests. Three.js is MIT-licensed; see `web/THREE-LICENSE.txt`.

## Initial conditions and playback

1. Set cube side length, total mass, coil radius, each coil's current/turns, initial orientation, angular velocity, and external field in the sidebar.
2. Press **Apply initial conditions / Reset**. Form edits alone do not alter the applied model. All coils remain perpendicular; orientation rotates the whole cube. Body +Z normal vectors are normalized; zero is rejected. Angular velocity and external field use world coordinates.
3. Use **Play**, **Pause**, and **Reset** inside the player. Coil currents stay fixed in the body frame; the external field stays constant in the world frame. Reset returns to the applied initial conditions.
4. Drag the **Time scale** slider while playing: 10× means 10 simulated seconds per active real second. Simulation time, active real time, angular speed, magnetic moment, and torque update live. Pauses exclude elapsed wall time.

Mouse drag rotates the camera, wheel/pinch zooms, and right-drag pans. Camera controls run independently of the physics worker. Field lines and field arrows can be toggled independently; hiding arrows also hides magnitude dots. Arrow size and physical-vector display scale are native sliders and do not trigger Streamlit redraws. Applying initial conditions reloads the player. Refreshing the page does not preserve a trajectory.

Coils and their magnetic moment vectors are orange (X), green (Y), and violet (Z). Net magnetic moment is gold, external field is teal, and torque is magenta. Vector lengths scale linearly with magnitude. All moment vectors share a scale; different physical units have separate reference scales: 0.1 A·m², 40 µT, and 4 µN·m each span 60% of the field-view half-width at display scale 1.

The calculations tab and metrics above the player describe **initial conditions**. The player shows live quantities and exports the last 2,000 trajectory samples as CSV (time, quaternion, and angular velocity).

## Equations and approximations

Each moment is m_i = N_i I_i πR² n_i; net moment is their vector sum. Torque is m_world × B_external. Uniform solid-cube inertia about its center of mass is J = ML²/6 on each axis. Mass affects inertia and angular acceleration, not magnetic moment.

Attitude uses quaternion RK4 integration: q̇ = ½[0,ω_world]⊗q and ω̇ = τ/J. Quaternion length is normalized after each step. Step size is limited by angular speed and torque frequency. The worker integrates toward the scaled monotonic wall-clock target, subject to a per-update work budget. It never advances the displayed time without integrating physics. If extreme parameters or speeds exceed available CPU, a pending-time message suggests lowering the time scale.

The 3D scene renders via WebGL on browser animation frames. Physics runs in a Web Worker; field geometry refreshes at up to 2 Hz, so it may lag slightly behind attitude. Biot–Savart uses 240 segments per energized circular coil, summed with the external field. Display field lines use midpoint integration of a trilinearly interpolated 9×9×9 field grid from **12 seeds (4 per coil)**. Trace spacing is illustrative, not flux density. Near-wire points within 2 mm are excluded. Display interpolation does not enter attitude dynamics.

Assumptions: uniform solid-cube mass distribution, coincident circular turns, vacuum permeability, and uniform external field. No core saturation, heating, resistance, damping, feedback control, or orbital model. Self-fields do not supply net external torque. Magnetorquers cannot produce torque parallel to the external field. This is an engineering exploration model, not a flight qualification model.

## Code and tests

- `physics.py`, `dynamics.py`: Python reference equations and attitude integrator.
- `web/physics.js`: SI-unit browser equations, RK4, Biot–Savart, grid interpolation, field-line tracing.
- `web/worker.js`: background physics/field work.
- `web/player.js`: persistent WebGL scene, camera, sliders, playback clock, CSV export.
- `app.py`: initial-condition form, player embedding/export, initial calculation review.
- `visuals.py`: retained Python Plotly reference visualization.

```bash
.venv/bin/python -m pytest -q
npm test
```

Browser equation tests compare a rotated three-coil trajectory and magnetic field against the Python reference fixture. The optional real-browser check requires `playwright` and Chromium (set `CHROMIUM_PATH` if needed):

```bash
.venv/bin/python -m pip install playwright
.venv/bin/python web/check_browser.py http://127.0.0.1:8501
```

It verifies the native time slider, 10× elapsed time, camera movement during playback, display toggles, pause, and reset in the actual player iframe.
