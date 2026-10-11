import time
import numpy as np
import plotly.graph_objects as go
import streamlit as st
from physics import Coil, CubeSat, torque
from dynamics import Simulation, PlaybackClock
from visuals import scene

st.set_page_config(page_title='CubeSat Magnetorquer Lab', layout='wide')
st.title('CubeSat Magnetorquer Lab')
st.caption('Set initial conditions, then play the magnetorquer attitude simulation.')


def load_initial(cube, field, omega):
    st.session_state.simulation = Simulation(cube, field, omega)
    st.session_state.clock = PlaybackClock(speed=st.session_state.get('time_scale', 1.0))
    st.session_state.history = [(0.0, float(np.rad2deg(np.linalg.norm(omega))))]


if 'simulation' not in st.session_state:
    load_initial(CubeSat(0.1, 1.0), np.array([0,0,40e-6]), np.zeros(3))

with st.sidebar:
    st.header('Initial conditions')
    st.caption('Changes take effect when you press Apply initial conditions / Reset. Display and playback controls apply live.')
    with st.form('initial-conditions'):
        side_mm = st.number_input('Cube side length (mm)', min_value=20.0, max_value=1000.0, value=100.0, step=10.0)
        mass = st.number_input('Cube mass (kg)', min_value=0.01, max_value=1000.0, value=1.0, step=0.1)
        fill = st.slider('Coil radius (% of cube side)', 10, 49, 40)
        currents, turns = [], []
        for label in ('X', 'Y', 'Z'):
            with st.expander(f'{label} coil', expanded=True):
                currents.append(st.number_input(f'{label} current (A)', min_value=-10.0, max_value=10.0, value=0.1, step=0.01))
                turns.append(st.number_input(f'{label} turns', min_value=1, max_value=10000, value=100, step=1))
        orientation = st.radio('Initial cube orientation', ['Angles', 'Normal vector'])
        with st.expander('Initial orientation values', expanded=True):
            tilt = st.slider('Body +Z tilt from world +z (°)', 0, 180, 0)
            azimuth = st.slider('Body +Z azimuth (°)', -180, 180, 0)
            nx = st.number_input('Body +Z normal x', value=0.0, step=0.1)
            ny = st.number_input('Body +Z normal y', value=0.0, step=0.1)
            nz = st.number_input('Body +Z normal z', value=1.0, step=0.1)
            st.caption('Use the angle or normal-vector inputs selected above; all three coils rotate together.')
        st.subheader('Initial angular velocity')
        initial_rates = [st.number_input(f'Initial ω{axis} (°/s)', min_value=-360.0, max_value=360.0, value=0.0, step=0.1) for axis in ('x','y','z')]
        st.caption('Angular velocity components use the fixed world frame.')
        st.subheader('External field (world frame)')
        bx = st.number_input('Bx (µT)', value=0.0)
        by = st.number_input('By (µT)', value=0.0)
        bz = st.number_input('Bz (µT)', value=40.0)
        submitted = st.form_submit_button('Apply initial conditions / Reset')
    if submitted:
        try:
            if orientation == 'Normal vector':
                oriented = Coil.from_normal(1,1,1,[nx,ny,nz])
                tilt, azimuth = oriented.tilt_deg, oriented.azimuth_deg
            cube = CubeSat(side_mm/1000, mass, tuple(currents), tuple(turns), fill/100, tilt, azimuth)
            field = np.array([bx,by,bz])*1e-6
            if not np.isfinite(field).all():
                raise ValueError('External field must be finite.')
            load_initial(cube, field, np.deg2rad(initial_rates))
        except ValueError as error:
            st.error(str(error))
    st.header('Playback')
    time_scale = st.slider('Time scale (sim seconds / real second)', 0.1, 100.0, 1.0, 0.1, key='time_scale')
    st.session_state.clock.set_speed(time_scale, time.monotonic())
    st.header('Display')
    show_lines = st.checkbox('Show connected field lines', value=True)
    show_arrows = st.checkbox('Show field arrows', value=True, help='Shows or hides both field arrows and colored magnitude dots.')
    arrow_scale = st.slider('Field arrow size', 0.05, 2.0, 0.35, 0.05)
    vector_scale = st.slider('Moment / torque / external vector scale', 0.01, 2.0, 1.0, 0.01)


def toggle_play():
    clock = st.session_state.clock
    clock.set_playing(not clock.playing, time.monotonic())


def reset_playback():
    old = st.session_state.simulation
    load_initial(old.initial_cube, old.external_field, old.initial_omega)


play, reset = st.columns([1,1])
play.button('Pause' if st.session_state.clock.playing else 'Play', on_click=toggle_play, width='stretch')
reset.button('Reset to applied initial conditions', on_click=reset_playback, width='stretch')


@st.fragment(run_every=0.25 if st.session_state.clock.playing else None)
def render_simulation():
    simulation = st.session_state.simulation
    clock = st.session_state.clock
    clock.tick(time.monotonic())
    advanced = simulation.advance(max(0.0, clock.target_time-simulation.time))
    if advanced > 0:
        st.session_state.history.append((simulation.time, float(np.rad2deg(np.linalg.norm(simulation.omega)))))
        st.session_state.history = st.session_state.history[-2000:]
    elapsed, real, speed, status = st.columns(4)
    elapsed.metric('Simulation time', f'{simulation.time:.3f} s')
    real.metric('Active real time', f'{clock.wall_elapsed:.3f} s')
    speed.metric('Time scale', f'{clock.speed:g}×')
    status.metric('Playback', 'Playing' if clock.playing else 'Paused')
    if clock.target_time-simulation.time > 0.1:
        st.warning(f'Integration is catching up: {clock.target_time-simulation.time:.2f} simulated seconds pending. Reduce time scale for smoother playback.')
    cube = simulation.cube
    earth = simulation.external_field
    tau = torque(cube, earth)
    alpha = cube.angular_acceleration(earth)
    a,b,c,d = st.columns(4)
    a.metric('Net magnetic moment', f'{np.linalg.norm(cube.moment):.5g} A·m²')
    b.metric('Net torque', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
    c.metric('Inertia per axis', f'{cube.inertia[0,0]:.5g} kg·m²')
    d.metric('Angular acceleration', f'{np.linalg.norm(alpha):.5g} rad/s²')
    visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
    with visual:
        st.plotly_chart(scene(cube, earth, field_arrow_scale=arrow_scale, show_lines=show_lines, show_arrows=show_arrows, vector_scale=vector_scale, live=clock.playing), width='stretch', key='field-scene')
        st.caption('Orange: X coil · Green: Y coil · Violet: Z coil. Curves follow the combined field of all three coils, plus the external field. Line spacing is illustrative. Small arrows show direction; dots show magnitude. Coil-colored vectors: individual magnetic moments · Gold: net magnetic moment · Teal: external field · Magenta: net torque. Vector lengths scale linearly with magnitude, using separate scales for different units; labels give actual values. Reduce the vector scale if a tip extends beyond the view. Field traces exclude points within 2 mm of energized wires.')
        st.caption('While playing, field-line curves use interpolated grid values for responsiveness. Paused curves use direct Biot–Savart tracing. Dynamics always use the exact magnetic moment and external-field torque.')
        history = st.session_state.history
        if len(history) > 1:
            chart = go.Figure(go.Scatter(x=[h[0] for h in history], y=[h[1] for h in history]))
            chart.update_layout(title='Angular speed over simulated time', xaxis_title='Simulation time (s)', yaxis_title='Angular speed (°/s)', height=250)
            st.plotly_chart(chart, width='stretch', key='speed-history')
    with calculations:
        st.latex(r'R=fL,\quad A=\pi R^2,\quad \mathbf m_i=N_i I_i A\hat{\mathbf n}_i,\quad \mathbf m_{net}=\sum_i\mathbf m_i')
        st.latex(r'\mathbf B_{coils}=\sum_i\mathbf B_i,\quad \boldsymbol\tau=\mathbf m_{net}\times\mathbf B_{external}')
        st.latex(r'\dot{\mathbf R}=[\boldsymbol\omega]_{\times}\mathbf R,\quad \dot{\boldsymbol\omega}=\mathbf J^{-1}(\mathbf m_{world}\times\mathbf B_{external})')
        st.latex(r'J_x=J_y=J_z=\frac{ML^2}{6},\quad \mathbf J=\frac{ML^2}{6}\mathbf 1,\quad \boldsymbol\alpha=\mathbf J^{-1}\boldsymbol\tau')
        st.latex(r'\mathbf B_i(\mathbf r)=\frac{\mu_0 N_i I_i}{4\pi}\oint\frac{d\boldsymbol\ell\times(\mathbf r-\mathbf r\prime)}{|\mathbf r-\mathbf r\prime|^3},\quad \frac{d\mathbf r}{ds}=\frac{\mathbf B}{|\mathbf B|}')
        vectors = [(f'{label} coil moment (A·m²)', c.moment) for label,c in zip(('X','Y','Z'),cube.coils)]
        vectors += [('Net moment (A·m²)',cube.moment),('External B (T)',earth),('Net torque (N·m)',tau),('Angular acceleration (rad/s²)',alpha),('Angular velocity (rad/s)',simulation.omega)]
        st.dataframe({'Quantity':[name for name,_ in vectors],**{axis:[v[i] for _,v in vectors] for i,axis in enumerate(('x','y','z'))}},hide_index=True)
        st.caption('At vector scale 1, a vector spans 60% of the field-view half-width for 0.1 A·m², 40 µT, or 4 µN·m. All moment vectors share one scale; unlike units cannot be compared by length.')
        st.write('Body axes expressed in world coordinates (columns X, Y, Z):')
        st.dataframe(cube.rotation)
        st.write(f'Each coil radius: {cube.side_m*cube.radius_fraction*1000:.3g} mm. Face centers: +L/2 along each positive body axis. Inertia is about the cube center of mass.')
        st.download_button('Download calculation (CSV)',data='quantity,x,y,z\n'+'\n'.join(','.join([name]+[str(v) for v in vector]) for name,vector in vectors),file_name='magnetorquer.csv')
        st.info('Assumptions: uniform solid cube mass distribution, coincident circular turns, vacuum permeability, and uniform external field. Mass affects inertia and angular acceleration, not magnetic moment. Attitude evolves using quaternion RK4 integration with constant body-frame currents and a constant world-frame external field. No core saturation, heating, resistance, damping, feedback controller, or orbital field model. The coils’ own fields do not supply net external torque. Torque along the external field cannot be produced.')


render_simulation()
