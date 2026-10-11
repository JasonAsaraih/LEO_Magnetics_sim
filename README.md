# CubeSat Magnetorquer Lab

Interactive 3D magnetic-field and torque calculator for an air-core circular magnetorquer. Adjust current, turns, radius, coil tilt/azimuth, and external field components. Review equations and vector calculations in the second tab and export results as CSV.

## Run

Python 3.12 is validated.

```bash
python -m venv /workspace/leo-magnetics-env/venv
/workspace/leo-magnetics-env/venv/bin/python -m pip install -r requirements.txt
/workspace/leo-magnetics-env/venv/bin/python -m streamlit run app.py --server.headless=true --browser.gatherUsageStats=false
```

Run those commands from this checkout. Streamlit normally listens on port 8501. Cloud onboarding validates locally; it does not provide a web preview. To view the UI, run locally or use your development platform's supported port access.

## Structure

- `physics.py`: pure SI-unit equations, coil geometry, Biot–Savart quadrature, exact axial field, and external-field torque.
- `visuals.py`: Plotly scene construction, independent of UI controls.
- `app.py`: Streamlit controls and calculation display.
- `test_physics.py`: exact-solution, torque, sign, and singularity checks.

```bash
/workspace/leo-magnetics-env/venv/bin/python -m pytest -q
```

The coil normal uses polar tilt from +z and azimuth from +x toward +y. Positive current follows the right-hand rule. The field is computed from 240 straight segments; grid points within 2 mm of the ideal wire are omitted. Arrow lengths are normalized; marker color carries magnitude. The axial chart uses the exact analytical solution.

This model assumes coincident circular turns and a uniform external field. It does not model core permeability/saturation, thermal limits, resistance, orbit propagation, induced electric fields, or attitude dynamics. The coil's own field is excluded from net torque. A magnetorquer cannot generate torque along the external magnetic field.

Connected field lines are enabled by default and can be toggled under Display. They integrate dr/ds = B/|B| with the midpoint method, using the same Biot–Savart field as the arrows (plus the external field when selected). Curves stop near the wire, at nulls, at the view boundary, upon returning to a seed, or after 360 steps per direction. They are finite traces, not forcibly closed loops; their seed spacing does not represent flux density.

Under Coil orientation, choose Angles or Normal vector. In Normal vector mode, enter x/y/z components for the vector perpendicular to the coil plane. Any nonzero length is accepted and normalized; a zero vector displays an error. The derived tilt and azimuth are displayed for review. Show field arrows toggles both the small arrows and the colored field-magnitude dots/color scale; connected field lines and the external-field/torque vectors remain independent.
