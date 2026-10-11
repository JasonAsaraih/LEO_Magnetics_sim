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
    tilt = st.slider('Normal tilt from +z (°)', 0, 180, 45)
    azimuth = st.slider('Normal azimuth from +x toward +y (°)', -180, 180, 0)
    st.header('External field')
    bx = st.number_input('Bx (µT)', value=0.0)
    by = st.number_input('By (µT)', value=0.0)
    bz = st.number_input('Bz (µT)', value=40.0)
    total = st.checkbox('Show coil + external field', value=False)
coil = Coil(current, int(turns), radius/1000, tilt, azimuth)
earth = np.array([bx,by,bz])*1e-6
tau = torque(coil, earth)
a,b,c = st.columns(3)
a.metric('Magnetic moment', f'{np.linalg.norm(coil.moment):.5g} A·m²')
b.metric('Torque magnitude', f'{np.linalg.norm(tau)*1e6:.5g} µN·m')
c.metric('Coil center field', f'{abs(float(axial_field(coil,0)))*1e6:.5g} µT')
visual, calculations = st.tabs(['Field explorer', 'Equations & calculations'])
with visual:
    st.plotly_chart(scene(coil, earth, total), width='stretch')
    st.caption('Rotate and zoom the 3D view. Arrow lengths are normalized; point colors show field magnitude. Points within 2 mm of the ideal wire are omitted.')
    z = np.linspace(-0.3,0.3,301)
    chart = go.Figure(go.Scatter(x=z*1000, y=axial_field(coil,z)*1e6))
    chart.update_layout(title='Coil field along its normal', xaxis_title='Distance from center (mm)', yaxis_title='Signed axial B (µT)', height=300)
    st.plotly_chart(chart, width='stretch')
with calculations:
    st.latex(r'A=\pi R^2,\quad \mathbf m=NI A\hat{\mathbf n},\quad \boldsymbol\tau=\mathbf m\times\mathbf B_{external}')
    st.latex(r'\mathbf B(\mathbf r)=\frac{\mu_0 NI}{4\pi}\oint\frac{d\boldsymbol\ell\times(\mathbf r-\mathbf r\prime)}{|\mathbf r-\mathbf r\prime|^3}')
    st.latex(r'B_{axis}(z)=\frac{\mu_0 NIR^2}{2(R^2+z^2)^{3/2}}')
    st.write('Vector components are in the fixed x, y, z frame. Positive current follows the right-hand rule about the displayed positive normal.')
    st.dataframe({'Quantity':['Normal','Moment (A·m²)','External B (T)','Torque (N·m)'], 'x':[coil.basis[2][0],coil.moment[0],earth[0],tau[0]], 'y':[coil.basis[2][1],coil.moment[1],earth[1],tau[1]], 'z':[coil.basis[2][2],coil.moment[2],earth[2],tau[2]]}, hide_index=True)
    st.download_button('Download calculation (CSV)', data='quantity,x,y,z\n'+ '\n'.join(','.join([name]+[str(v) for v in vector]) for name,vector in [('normal',coil.basis[2]),('moment_A_m2',coil.moment),('external_B_T',earth),('torque_N_m',tau)]), file_name='magnetorquer.csv')
    st.info('Assumptions: coincident circular turns, vacuum permeability, steady current, uniform external field. No ferromagnetic core, saturation, winding resistance/heating, orbital field model, or attitude dynamics. This is a field and torque calculator, not a flight qualification model. Torque parallel to the external field cannot be produced.')
