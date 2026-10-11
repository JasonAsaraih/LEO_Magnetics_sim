import json
from dataclasses import replace
from pathlib import Path
import streamlit.components.v1 as components
import numpy as np
import streamlit as st
from physics import Coil, CubeSat, torque
from dynamics import Simulation, quaternion
from controller import euler_quaternion_degrees, initial_pid_currents

st.set_page_config(page_title='CubeSat Magnetorquer Lab', layout='wide')
st.title('CubeSat Magnetorquer Lab')
st.caption('Set initial conditions, then play the magnetorquer attitude simulation.')


def load_initial(cube, field, omega, controller=None):
    st.session_state.simulation = Simulation(cube, field, omega)
    st.session_state.controller_config = controller or {"enabled": False}
    st.session_state.initial_revision = st.session_state.get('initial_revision', 0) + 1


if 'simulation' not in st.session_state:
    load_initial(CubeSat(0.1, 1.0), np.array([0,0,40e-6]), np.zeros(3))

with st.sidebar:
    st.header('Initial conditions')
    st.caption('Change the values below, then press Update initial state to preview them at time zero. Display and playback controls apply live.')
    with st.form('initial-conditions'):
        submitted = st.form_submit_button('Update initial state', type='primary', width='stretch')
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
        with st.expander('PID controller and final targets', expanded=True):
            enable_pid = st.checkbox('Enable PID current control', value=False)
            st.caption('PID overrides manual coil currents at time zero. Targets and gains take effect when you click Update initial state.')
            target_rates = [st.number_input(f'Target ω{axis} (°/s)', min_value=-360.0, max_value=360.0, value=0.0, step=0.1) for axis in ('x','y','z')]
            target_angles = [st.number_input(f'Final {axis} (°)', min_value=-180.0, max_value=180.0, value=0.0, step=1.0) for axis in ('roll','pitch','yaw')]
            st.caption('Final roll/pitch/yaw applies only when all target angular velocities are zero. Targets use world coordinates, with R = Rz(yaw) Ry(pitch) Rx(roll). A fixed field cannot control torque along its own direction.')
            kp = st.number_input('PID Kp', min_value=0.0, value=1e-5, step=1e-6, format='%.3e')
            ki = st.number_input('PID Ki', min_value=0.0, value=0.0, step=1e-8, format='%.3e')
            kd = st.number_input('PID Kd', min_value=0.0, value=2e-4, step=1e-5, format='%.3e')
            st.caption('Zero-rate mode: Kp × angle error + Ki × integrated angle error + Kd × velocity error. Nonzero-rate mode: PID on velocity error. Gains output torque in N·m from radians/rad·s⁻¹.')
            current_limits = [st.number_input(f'{axis} current limit (A)', min_value=0.001, max_value=10.0, value=1.0, step=0.01) for axis in ('X','Y','Z')]
            control_period = st.number_input('Controller sample period (s)', min_value=0.01, max_value=1.0, value=0.05, step=0.01)
            derivative_filter = st.number_input('Rate derivative filter time (s)', min_value=0.0, max_value=10.0, value=0.1, step=0.01)
        st.subheader('External field (world frame)')
        bx = st.number_input('Bx (µT)', value=0.0)
        by = st.number_input('By (µT)', value=0.0)
        bz = st.number_input('Bz (µT)', value=40.0)
    if submitted:
        try:
            if orientation == 'Normal vector':
                oriented = Coil.from_normal(1,1,1,[nx,ny,nz])
                tilt, azimuth = oriented.tilt_deg, oriented.azimuth_deg
            cube = CubeSat(side_mm/1000, mass, tuple(currents), tuple(turns), fill/100, tilt, azimuth)
            field = np.array([bx,by,bz])*1e-6
            if not np.isfinite(field).all():
                raise ValueError('External field must be finite.')
            controller = dict(enabled=enable_pid, targetRate=np.deg2rad(target_rates).tolist(),
                              targetQuaternion=euler_quaternion_degrees(target_angles).tolist(),
                              targetEulerDegrees=target_angles, kp=kp, ki=ki, kd=kd,
                              currentLimits=current_limits, period=control_period,
                              derivativeFilter=derivative_filter)
            if not np.isfinite(target_rates + target_angles + current_limits + [kp,ki,kd,control_period,derivative_filter]).all():
                raise ValueError('PID settings and targets must be finite.')
            load_initial(cube, field, np.deg2rad(initial_rates), controller)
            st.toast('Initial state updated. The simulation is paused at time zero.')
        except ValueError as error:
            st.error(str(error))


def render_simulation():
    simulation = st.session_state.simulation
    cube = simulation.initial_cube
    earth = simulation.external_field
    controller = st.session_state.get('controller_config', {'enabled': False})
    if controller['enabled']:
        cube = replace(cube, currents_a=tuple(initial_pid_currents(cube, earth, simulation.initial_omega, controller)))
    tau = torque(cube, earth)
    alpha = cube.angular_acceleration(earth)
    a,b,c,d = st.columns(4)
    a.metric('Initial net magnetic moment', f'{np.linalg.norm(cube.moment):.5g} A·m²')
    b.metric('Initial net torque', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
    c.metric('Inertia per axis', f'{cube.inertia[0,0]:.5g} kg·m²')
    d.metric('Initial angular acceleration', f'{np.linalg.norm(alpha):.5g} rad/s²')
    visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
    with visual:
        config = dict(revision=st.session_state.get('initial_revision', 0), side=cube.side_m, mass=cube.mass_kg, currents=cube.currents_a,
                      turns=cube.turns, radiusFraction=cube.radius_fraction,
                      quaternion=quaternion(cube.rotation).tolist(),
                      omega=simulation.initial_omega.tolist(), field=earth.tolist(), controller=controller)
        player = Path(__file__).parent / 'web' / 'dist' / 'player.html'
        if not player.exists():
            st.error('Browser player is missing. Run npm ci && npm run build from this checkout.')
            return
        html = player.read_text().replace('__CONFIG_JSON__', json.dumps(config).replace('</', r'<\/'))
        st.download_button('Download standalone player (HTML)', data=html, file_name='cubesat-player.html', mime='text/html', on_click='ignore')
        components.html(html, height=1450 if controller['enabled'] else 1100, scrolling=True)
        st.caption('Live playback runs entirely in your browser. Dragging the camera, changing time scale, and toggling display controls do not trigger Streamlit redraws. Applying new initial conditions reloads the player and resets time. The external field is always included.')
    with calculations:
        st.caption('The table below shows applied initial conditions. Live vectors and trajectory data are displayed/exported in the browser player.')
        st.latex(r'R=fL,\quad A=\pi R^2,\quad \mathbf m_i=N_i I_i A\hat{\mathbf n}_i,\quad \mathbf m_{net}=\sum_i\mathbf m_i')
        st.latex(r'\mathbf B_{coils}=\sum_i\mathbf B_i,\quad \boldsymbol\tau=\mathbf m_{net}\times\mathbf B_{external}')
        st.latex(r'\mathbf m_{cmd}=\frac{\mathbf B\times\boldsymbol\tau_{desired}}{|\mathbf B|^2},\quad I_i=\frac{(\mathbf R^T\mathbf m_{cmd})_i}{N_i\pi R_{coil}^2}')
        st.caption('PID requests torque, projects it onto the plane perpendicular to B through magnetic-moment allocation, then clips each current to its configured limit. The integral is projected onto that plane and frozen during saturation.')
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
        st.info('Assumptions: uniform solid cube mass distribution, coincident circular turns, vacuum permeability, and uniform external field. Mass affects inertia and angular acceleration, not magnetic moment. Attitude evolves using quaternion RK4 integration with manual body-frame currents or sampled PID current commands, and a constant world-frame external field. No core saturation, heating, resistance, passive damping or orbital field model. The coils’ own fields do not supply net external torque. Torque along the external field cannot be produced.')


render_simulation()
