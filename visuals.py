"""Plot construction only: consumes SI-unit model outputs."""
import numpy as np
import plotly.graph_objects as go
from physics import coil_field, torque


def scene(coil, earth_t, show_total=False, field_arrow_scale=0.35):
    extent = max(0.12, coil.radius_m*3)
    axis = np.linspace(-extent, extent, 9)
    points = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1).reshape(-1, 3)
    field = coil_field(coil, points)
    if show_total:
        field = field + earth_t
    magnitude = np.linalg.norm(field, axis=1)
    valid = np.isfinite(magnitude) & (magnitude > 0)
    p, b, strength = points[valid], field[valid], magnitude[valid]
    # Use raw cone sizing: actual scene length is controlled by the slider.
    # Plotly absolute/scaled modes apply an additional grid-dependent normalization.
    # Normalize field vectors only; magnitude is encoded separately.
    arrows = b/strength[:, None]*(extent*0.05*field_arrow_scale) if len(b) else b
    fig = go.Figure(go.Cone(x=p[:,0], y=p[:,1], z=p[:,2], u=arrows[:,0], v=arrows[:,1], w=arrows[:,2], sizemode='raw', sizeref=1, anchor='tail', showscale=False, colorscale='Blues', name='Field direction', hoverinfo='skip'))
    fig.add_trace(go.Scatter3d(x=p[:,0], y=p[:,1], z=p[:,2], mode='markers', marker=dict(size=2, color=strength*1e6, colorscale='Viridis', colorbar=dict(title='|B| µT')), customdata=strength*1e6, hovertemplate='|B|=%{customdata:.3g} µT<extra></extra>', name='Field magnitude'))
    wire = coil.wire()
    fig.add_trace(go.Scatter3d(x=wire[:,0], y=wire[:,1], z=wire[:,2], mode='lines', line=dict(color='#ef8b2c', width=8), name='Coil'))
    normal = coil.basis[2]*extent*0.8
    fig.add_trace(go.Scatter3d(x=[0,normal[0]], y=[0,normal[1]], z=[0,normal[2]], mode='lines', line=dict(color='#e85d75', width=5), name='Positive coil normal'))
    tau = torque(coil, earth_t)
    tau_magnitude = np.linalg.norm(tau)
    # Suppress roundoff torque for parallel vectors; normalize only for display.
    torque_scale = np.linalg.norm(coil.moment)*np.linalg.norm(earth_t)
    if tau_magnitude > torque_scale*1e-12:
        tip = tau/tau_magnitude*extent*0.85
        color = '#d528b8'
        label = f'τ = {tau_magnitude*1e6:.3g} µN·m'
        fig.add_trace(go.Scatter3d(x=[0,tip[0]], y=[0,tip[1]], z=[0,tip[2]], mode='lines+text', text=['',label], textposition='top center', textfont=dict(color=color, size=16), line=dict(color=color, width=10), name='Torque τ', hovertemplate=label+'<extra>Torque direction</extra>'))
        head = tip*0.22
        fig.add_trace(go.Cone(x=[tip[0]], y=[tip[1]], z=[tip[2]], u=[head[0]], v=[head[1]], w=[head[2]], anchor='tip', sizemode='absolute', sizeref=extent*0.16, colorscale=[[0,color],[1,color]], showscale=False, showlegend=False, hoverinfo='skip'))
    else:
        fig.add_annotation(text='τ = 0 · no torque direction', x=0.02, y=0.98, xref='paper', yref='paper', showarrow=False, font=dict(color='#d528b8', size=16))
    fig.update_layout(height=580, margin=dict(l=0,r=0,t=0,b=0), scene=dict(aspectmode='cube', xaxis_title='x (m)', yaxis_title='y (m)', zaxis_title='z (m)', xaxis=dict(range=[-extent,extent]), yaxis=dict(range=[-extent,extent]), zaxis=dict(range=[-extent,extent])), legend=dict(orientation='h'))
    return fig
