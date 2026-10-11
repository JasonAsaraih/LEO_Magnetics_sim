import numpy as np
import plotly.graph_objects as go
import streamlit as st
from physics import Coil, CubeSat, torque
from visuals import scene

st.set_page_config(page_title='CubeSat Magnetorquer Lab', layout='wide')
st.title('CubeSat Magnetorquer Lab')
st.caption('Three orthogonal air-core coils on a cube. Review magnetic moments, torque, and rotational inertia in SI units.')
with st.sidebar:
    st.header('Cube')
    side_mm = st.number_input('Cube side length (mm)', min_value=20.0, max_value=1000.0, value=100.0, step=10.0)
    mass = st.number_input('Cube mass (kg)', min_value=0.01, max_value=1000.0, value=1.0, step=0.1)
    fill = st.slider('Coil radius (% of cube side)', 10, 49, 40)
    st.caption('Circular coils are centered on the +X, +Y, and +Z faces. Their normals stay perpendicular to one another.')
    currents, turns = [], []
    for label in ('X', 'Y', 'Z'):
        with st.expander(f'{label} coil', expanded=True):
            currents.append(st.number_input(f'{label} current (A)', min_value=-10.0, max_value=10.0, value=0.1, step=0.01))
            turns.append(st.number_input(f'{label} turns', min_value=1, max_value=10000, value=100, step=1))
    orientation = st.radio('Cube orientation', ['Angles', 'Normal vector'])
    if orientation == 'Angles':
        tilt = st.slider('Body +Z tilt from world +z (°)', 0, 180, 0)
        azimuth = st.slider('Body +Z azimuth (°)', -180, 180, 0)
    else:
        nx = st.number_input('Body +Z normal x', value=0.0, step=0.1)
        ny = st.number_input('Body +Z normal y', value=0.0, step=0.1)
        nz = st.number_input('Body +Z normal z', value=1.0, step=0.1)
        try:
            orientation_coil = Coil.from_normal(1, 1, 1, [nx,ny,nz])
        except ValueError as error:
            st.error(str(error))
            st.stop()
        tilt, azimuth = orientation_coil.tilt_deg, orientation_coil.azimuth_deg
        st.caption(f'Tilt: {tilt:.2f}° · Azimuth: {azimuth:.2f}°. All coils rotate together; body X/Y use the deterministic basis shown in the calculations.')
    st.header('External field')
    bx = st.number_input('Bx (µT)', value=0.0)
    by = st.number_input('By (µT)', value=0.0)
    bz = st.number_input('Bz (µT)', value=40.0)
    total = st.checkbox('Show coil + external field', value=False)
    st.header('Display')
    show_lines = st.checkbox('Show connected field lines', value=True)
    show_arrows = st.checkbox('Show field arrows', value=True, help='Shows or hides both field arrows and colored magnitude dots.')
    arrow_scale = st.slider('Field arrow size', 0.05, 2.0, 0.35, 0.05)
cube = CubeSat(side_mm/1000, mass, tuple(currents), tuple(turns), fill/100, tilt, azimuth)
earth = np.array([bx,by,bz])*1e-6
tau = torque(cube, earth)
alpha = cube.angular_acceleration(earth)
a,b,c,d = st.columns(4)
a.metric('Net magnetic moment', f'{np.linalg.norm(cube.moment):.5g} A·m²')
b.metric('Net torque', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
c.metric('Inertia per axis', f'{cube.inertia[0,0]:.5g} kg·m²')
d.metric('Angular acceleration', f'{np.linalg.norm(alpha):.5g} rad/s²')
visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
with visual:
    st.plotly_chart(scene(cube, earth, total, field_arrow_scale=arrow_scale, show_lines=show_lines, show_arrows=show_arrows), width='stretch')
    st.caption('Orange: X coil · Green: Y coil · Violet: Z coil. Curves follow the combined field of all three coils, plus the external field when selected. Line spacing is illustrative. Small arrows show direction; dots show magnitude. Teal: external field · Magenta: net torque. Large vector lengths are normalized; labels give actual magnitudes. Field traces exclude points within 2 mm of energized wires.')
    angles = np.linspace(-10,10,201)
    # A useful current sweep keeps the remaining two coils at their selected values.
    moments = np.array([Coil.from_normal(value,cube.turns[0],cube.side_m*cube.radius_fraction,cube.rotation[:,0]).moment+sum((c.moment for c in cube.coils[1:]),start=np.zeros(3)) for value in angles])
    chart=go.Figure(go.Scatter(x=angles,y=np.linalg.norm(np.cross(moments,earth),axis=1)*1e6))
    chart.update_layout(title='Net torque vs X-coil current (Y/Z fixed)',xaxis_title='X current (A)',yaxis_title='Net torque (µN·m)',height=300)
    st.plotly_chart(chart,width='stretch')
with calculations:
    st.latex(r'R=fL,\quad A=\pi R^2,\quad \mathbf m_i=N_i I_i A\hat{\mathbf n}_i,\quad \mathbf m_{net}=\sum_i\mathbf m_i')
    st.latex(r'\mathbf B_{coils}=\sum_i\mathbf B_i,\quad \boldsymbol\tau=\mathbf m_{net}\times\mathbf B_{external}')
    st.latex(r'J_x=J_y=J_z=\frac{ML^2}{6},\quad \mathbf J=\frac{ML^2}{6}\mathbf 1,\quad \boldsymbol\alpha=\mathbf J^{-1}\boldsymbol\tau')
    st.latex(r'\mathbf B_i(\mathbf r)=\frac{\mu_0 N_i I_i}{4\pi}\oint\frac{d\boldsymbol\ell\times(\mathbf r-\mathbf r\prime)}{|\mathbf r-\mathbf r\prime|^3},\quad \frac{d\mathbf r}{ds}=\frac{\mathbf B}{|\mathbf B|}')
    vectors = [(f'{label} coil moment (A·m²)', c.moment) for label,c in zip(('X','Y','Z'),cube.coils)]
    vectors += [('Net moment (A·m²)',cube.moment),('External B (T)',earth),('Net torque (N·m)',tau),('Angular acceleration (rad/s²)',alpha)]
    st.dataframe({'Quantity':[name for name,_ in vectors],**{axis:[v[i] for _,v in vectors] for i,axis in enumerate(('x','y','z'))}},hide_index=True)
    st.write('Body axes expressed in world coordinates (columns X, Y, Z):')
    st.dataframe(cube.rotation)
    st.write(f'Each coil radius: {cube.side_m*cube.radius_fraction*1000:.3g} mm. Face centers: +L/2 along each positive body axis. Inertia is about the cube center of mass.')
    st.download_button('Download calculation (CSV)',data='quantity,x,y,z\n'+'\n'.join(','.join([name]+[str(v) for v in vector]) for name,vector in vectors),file_name='magnetorquer.csv')
    st.info('Assumptions: uniform solid cube mass distribution, coincident circular turns, vacuum permeability, and uniform external field. Mass affects inertia and angular acceleration, not magnetic moment. This is an instantaneous field/torque calculation; no attitude evolution, core saturation, heating, resistance, or orbital field model. The coils’ own fields do not supply net external torque. Torque along the external field cannot be produced.')
