import json
from pathlib import Path
import streamlit.components.v1 as components
import numpy as np
import streamlit as st
from physics import Coil, CubeSat, torque
from dynamics import Simulation, quaternion

st.set_page_config(page_title='CubeSat Magnetorquer Lab', layout='wide')
st.title('CubeSat Magnetorquer Lab')
st.caption('Set initial conditions, then play the magnetorquer attitude simulation.')


def load_initial(cube, field, omega):
    st.session_state.simulation = Simulation(cube, field, omega)


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


def render_simulation():
    simulation = st.session_state.simulation
    cube = simulation.initial_cube
    earth = simulation.external_field
    tau = torque(cube, earth)
    alpha = cube.angular_acceleration(earth)
    a,b,c,d = st.columns(4)
    a.metric('Initial net magnetic moment', f'{np.linalg.norm(cube.moment):.5g} A·m²')
    b.metric('Initial net torque', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
    c.metric('Inertia per axis', f'{cube.inertia[0,0]:.5g} kg·m²')
    d.metric('Initial angular acceleration', f'{np.linalg.norm(alpha):.5g} rad/s²')
    visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
    with visual:
        config = dict(side=cube.side_m, mass=cube.mass_kg, currents=cube.currents_a,
                      turns=cube.turns, radiusFraction=cube.radius_fraction,
                      quaternion=quaternion(cube.rotation).tolist(),
                      omega=simulation.initial_omega.tolist(), field=earth.tolist())
        player = Path(__file__).parent / 'web' / 'dist' / 'player.html'
        if not player.exists():
            st.error('Browser player is missing. Run npm ci && npm run build from this checkout.')
            return
        html = player.read_text().replace('__CONFIG_JSON__', json.dumps(config).replace('</', r'<\/'))
        st.download_button('Download standalone player (HTML)', data=html, file_name='cubesat-player.html', mime='text/html', on_click='ignore')
        components.html(html, height=1030, scrolling=True)
        st.caption('Live playback runs entirely in your browser. Dragging the camera, changing time scale, and toggling display controls do not trigger Streamlit redraws. Applying new initial conditions reloads the player and resets time. The external field is always included.')
    with calculations:
        st.caption('The table below shows applied initial conditions. Live vectors and trajectory data are displayed/exported in the browser player.')
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
