import numpy as np
import plotly.graph_objects as go
import streamlit as st
from physics import Coil, axial_field, torque
from visuals import scene

st.set_page_config(page_title='CubeSat Magnetorquer Lab', layout='wide')
st.title('CubeSat Magnetorquer Lab')
st.caption('Explore an air-core circular coil in a uniform external magnetic field. All calculations use SI units.')
with st.sidebar:
    st.header('Coil')
    current = st.slider('Current (A)', -2.0, 2.0, 0.1, 0.01)
    turns = st.number_input('Number of turns', 1, 10000, 100)
    radius = st.slider('Coil radius (mm)', 5.0, 100.0, 40.0, 1.0)
    orientation = st.radio('Coil orientation', ['Angles', 'Normal vector'])
    if orientation == 'Angles':
        tilt = st.slider('Normal tilt from +z (°)', 0, 180, 45)
        azimuth = st.slider('Normal azimuth from +x toward +y (°)', -180, 180, 0)
        coil = Coil(current, int(turns), radius/1000, tilt, azimuth)
    else:
        nx = st.number_input('Coil normal x', value=1.0, step=0.1)
        ny = st.number_input('Coil normal y', value=0.0, step=0.1)
        nz = st.number_input('Coil normal z', value=1.0, step=0.1)
        st.caption('The normal is perpendicular to the coil plane. Only its direction matters; components are normalized automatically.')
        try:
            coil = Coil.from_normal(current, int(turns), radius/1000, [nx, ny, nz])
        except ValueError as error:
            st.error(str(error))
            st.stop()
        st.caption(f'Tilt: {coil.tilt_deg:.2f}° · Azimuth: {coil.azimuth_deg:.2f}°')
    st.header('External field')
    bx = st.number_input('Bx (µT)', value=0.0)
    by = st.number_input('By (µT)', value=0.0)
    bz = st.number_input('Bz (µT)', value=40.0)
    total = st.checkbox('Show coil + external field', value=False)
    st.header('Display')
    show_lines = st.checkbox('Show connected field lines', value=True)
    show_arrows = st.checkbox('Show field arrows', value=True, help='Shows or hides both field arrows and colored magnitude dots.')
    arrow_scale = st.slider('Field arrow size', 0.05, 2.0, 0.35, 0.05, help='Adjusts field arrows only; torque arrow and calculated values stay the same.')
earth = np.array([bx,by,bz])*1e-6
tau = torque(coil, earth)
a,b,c = st.columns(3)
a.metric('Magnetic moment', f'{np.linalg.norm(coil.moment):.5g} A·m²')
b.metric('Torque magnitude', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
c.metric('Coil center field', f'{abs(float(axial_field(coil,0)))*1e6:.5g} µT')
visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
with visual:
    st.plotly_chart(scene(coil, earth, total, field_arrow_scale=arrow_scale, show_lines=show_lines, show_arrows=show_arrows), width='stretch')
    st.caption('Rotate and zoom the 3D view. Connected curves follow the selected magnetic field; line spacing is illustrative, not a measure of strength. Small arrows show field direction; point colors show field magnitude. The teal arrow shows the external magnetic field; the magenta arrow shows torque. Both use normalized lengths, with actual magnitudes in their labels. Points within 2 mm of the ideal wire are omitted.')
    z = np.linspace(-0.3,0.3,301)
    chart = go.Figure(go.Scatter(x=z*1000, y=axial_field(coil,z)*1e6))
    chart.update_layout(title='Coil field along its normal', xaxis_title='Distance from center (mm)', yaxis_title='Signed axial B (µT)', height=300)
    st.plotly_chart(chart, width='stretch')
with calculations:
    st.latex(r'A=\pi R^2,\quad \mathbf m=NI A\hat{\mathbf n},\quad \boldsymbol\tau=\mathbf m\times\mathbf B_{external}')
    st.latex(r'\mathbf B(\mathbf r)=\frac{\mu_0 NI}{4\pi}\oint\frac{d\boldsymbol\ell\times(\mathbf r-\mathbf r\prime)}{|\mathbf r-\mathbf r\prime|^3}')
    st.latex(r'B_{axis}(z)=\frac{\mu_0 NIR^2}{2(R^2+z^2)^{3/2}}')
    st.latex(r'\frac{d\mathbf r}{ds}=\frac{\mathbf B(\mathbf r)}{|\mathbf B(\mathbf r)|}')
    st.write('Field lines use midpoint integration in both directions from seeds inside the coil. They stop at the view boundary, near the wire, at a field null, on return to a seed, or at the integration limit.')
    st.write('Vector components are in the fixed x, y, z frame. Positive current follows the right-hand rule about the displayed positive normal.')
    st.dataframe({'Quantity':['Normal','Moment (A·m²)','External B (T)','Torque (N·m)'], 'x':[coil.basis[2][0],coil.moment[0],earth[0],tau[0]], 'y':[coil.basis[2][1],coil.moment[1],earth[1],tau[1]], 'z':[coil.basis[2][2],coil.moment[2],earth[2],tau[2]]}, hide_index=True)
    st.download_button('Download calculation (CSV)', data='quantity,x,y,z\n'+ '\n'.join(','.join([name]+[str(v) for v in vector]) for name,vector in [('normal',coil.basis[2]),('moment_A_m2',coil.moment),('external_B_T',earth),('torque_N_m',tau)]), file_name='magnetorquer.csv')
    st.info('Assumptions: coincident circular turns, vacuum permeability, steady current, uniform external field. No ferromagnetic core, saturation, winding resistance/heating, orbital field model, or attitude dynamics. This is a field and torque calculator, not a flight qualification model. Torque parallel to the external field cannot be produced.')
