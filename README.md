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
2. Press **Update initial state**. Form edits alone do not alter the applied model. All coils remain perpendicular; orientation rotates the whole cube. Body +Z normal vectors are normalized; zero is rejected. Angular velocity and external field use world coordinates.
3. Use **Play**, **Pause**, and **Reset** inside the player. Manual coil currents stay fixed in the body frame; when PID is enabled the controller replaces them with dynamic commands; the external field stays constant in the world frame. Reset returns to the applied initial conditions.
4. Drag the **Time scale** slider while playing: 10× means 10 simulated seconds per active real second. Simulation time, active real time, angular speed, magnetic moment, and torque update live. Pauses exclude elapsed wall time.

Mouse drag rotates the camera, wheel/pinch zooms, and right-drag pans. Camera controls run independently of the physics worker. Field lines and field arrows can be toggled independently; hiding arrows also hides magnitude dots. Arrow size and physical-vector display scale are native sliders and do not trigger Streamlit redraws. Applying initial conditions reloads the player. Refreshing the page does not preserve a trajectory.

Coils and their magnetic moment vectors are orange (X), green (Y), and violet (Z). Net magnetic moment is gold, external field is teal, and torque is magenta. Vector lengths scale linearly with magnitude. All moment vectors share a scale; different physical units have separate reference scales: 0.1 A·m², 40 µT, and 4 µN·m each span 60% of the field-view half-width at display scale 1.

The calculations tab and metrics above the player describe **initial conditions**. The player shows live quantities and exports the last 2,000 trajectory samples as CSV (time, quaternion, and angular velocity).

## Equations and approximations

Each moment is m_i = N_i I_i πR² n_i; net moment is their vector sum. Torque is m_world × B_external. Uniform solid-cube inertia about its center of mass is J = ML²/6 on each axis. Mass affects inertia and angular acceleration, not magnetic moment.

Attitude uses quaternion RK4 integration: q̇ = ½[0,ω_world]⊗q and ω̇ = τ/J. Quaternion length is normalized after each step. Step size is limited by angular speed and torque frequency. The worker integrates toward the scaled monotonic wall-clock target, subject to a per-update work budget. It never advances the displayed time without integrating physics. If extreme parameters or speeds exceed available CPU, a pending-time message suggests lowering the time scale.

The 3D scene renders via WebGL on browser animation frames. Physics runs in a Web Worker; field geometry refreshes at up to 2 Hz, so it may lag slightly behind attitude. Biot–Savart uses 240 segments per energized circular coil, summed with the external field. Display field lines use midpoint integration of a trilinearly interpolated 9×9×9 field grid from **12 seeds (4 per coil)**. Trace spacing is illustrative, not flux density. Near-wire points within 2 mm are excluded. Display interpolation does not enter attitude dynamics.

Assumptions: uniform solid-cube mass distribution, coincident circular turns, vacuum permeability, and uniform external field. No core saturation, heating, winding inductance/resistance, passive damping, or orbital model. Optional sampled PID feedback is available. Self-fields do not supply net external torque. Magnetorquers cannot produce torque parallel to the external field. This is an engineering exploration model, not a flight qualification model.

## Code and tests

- `physics.py`, `dynamics.py`: Python reference equations and attitude integrator.
- `web/physics.js`: SI-unit browser equations, RK4, sample scheduling, Biot–Savart, grid interpolation, field-line tracing.
- `web/controller.js`: editable PID torque law, magnetic allocation, limits, anti-windup, and all-update peak logging.
- `controller.py`: Python reference for the initial PID commands used in the calculation table.
- `web/math.js`: quaternion, rotation-error, and vector helpers.
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


## PID control and final targets

Enable **PID current control** in the sidebar, set targets/gains/limits, then click **Update initial state**. PID takes over all three currents at simulation time zero. Manual current entries apply only when PID is off. The initial vectors and calculation table use the initial controlled currents when PID is enabled.

- Set target world angular velocities ωx/ωy/ωz in degrees per second.
- When all three target rates are zero, set final roll/pitch/yaw in degrees. The target attitude is Rz(yaw) Ry(pitch) Rx(roll). Control uses a shortest-path quaternion rotation error rather than subtracting Euler angles, avoiding wraparound at ±180°.
- When any target rate is nonzero, control uses rate PID and ignores the final attitude inputs.
- Set Kp, Ki, Kd, each coil's absolute current limit, the simulated control sample period, and the derivative filter time. A common set of gains applies to the three world axes. The default Ki is zero; increase it only as needed.

In zero-rate attitude mode, desired torque is Kp e_angle + Ki ∫e_angle dt + Kd(ω_target − ω). In rate mode it is Kp e_rate + Ki ∫e_rate dt + Kd filtered(de_rate/dt). Radians and rad/s are used internally, and gains output N·m. Gains have different units between modes: the attitude P/I/D inputs are rad, rad·s, and rad/s; the rate inputs are rad/s, rad, and rad/s². Retune when changing modes.

Magnetic allocation uses m_world = (B × τ_desired)/|B|², rotates that into the body frame, then computes I_i = m_body_i/(N_i πR²). This produces only the torque perpendicular to B; each current is clipped to its configured limit. The integral accumulates only error perpendicular to B and freezes while currents saturate. With zero external field the command is zero and the requested torque remains unmet. A constant-field uniform cube cannot change the angular-velocity component along B; arbitrary target velocities/attitudes are not guaranteed reachable. Live error and unmet-torque readouts expose this limitation.

To implement your own controller, edit **`desiredTorque()` in `web/controller.js`**, then run `npm run build`. It receives current Euler angles, quaternion, angular velocity, angle/rate errors, target rate, integral, filtered derivative, mode, and gains. Return a world-frame torque vector in N·m. If you change the torque law, update the initial-command reference in `controller.py` too; the Python initial tables otherwise continue to describe the built-in PID law. The allocator, coil limits, and peak logs remain reusable; `web/math.js` provides quaternion helpers.

The player logs per-coil **peak absolute required current**, **peak absolute applied current**, and both corresponding **peak |ΔI/Δt|**, considering every controller sample since Reset. Required values are before clipping, so saturation cannot hide demand. Rates are finite differences over the simulated sample period; the initial ideal turn-on jump is excluded. These are sampled command-change rates, not a winding electrical-rise model or a bound on the continuous physical derivative. Reset clears PID history, integral state, and peak logs.

Use **Export peak current log** for all-run peak values and **Export trajectory CSV** for the most recent 2,000 display samples, including applied/required currents and current-change rates. Display sampling can be slower than control updates; peak statistics still include all updates. Browser controller tests check a reachable final pitch, a rate target, saturation/anti-windup, derivative peaks, frame-cadence independence, and agreement with Python initial commands. Optional end-to-end PID validation:

```bash
.venv/bin/python web/check_pid.py http://127.0.0.1:8501
```

## Approximate coil inductance

Each coil has a **winding bundle diameter (mm)** input: the diameter of the entire round cross-section containing all turns, not an individual wire's diameter. Click Update initial state to apply geometry/turn changes. Approximate inductance appears in mH above the player and in the player’s coil table (including exported standalone players); the calculations tab shows the equation and offers a dedicated inductance CSV with values in henries.

For a thin circular air-core winding, the model uses L ≈ μ₀ N² R [ln(8R/a) − 7/4], where R is mean turn radius and a is half the winding-bundle diameter. It approximates uniformly distributed DC current in a round winding cross-section and requires a/R ≤ 0.1. Turn count scales the estimate as N² when geometry stays fixed. The default 100-turn, 40-mm-radius, 3-mm-bundle coil estimates approximately 1.82 mH.

The model reports **Outside model** instead of a number when the thin-bundle condition fails. Flat or rectangular winding packs, core materials, conductive mounting structures, and inter-coil mutual inductance are not modeled. Inductance is a design estimate only; it does not add winding electrical dynamics or change the ideal PID current response. `approximate_inductance()` in `physics.py` contains the reviewable SI equation and geometry checks.
