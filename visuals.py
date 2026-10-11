"""Plot construction only: consumes SI-unit model outputs."""
from functools import lru_cache
import numpy as np
import plotly.graph_objects as go
from physics import torque


def scene(coil, earth_t, field_arrow_scale=0.35, show_lines=True, show_arrows=True, vector_scale=1.0, live=False):
    extent = coil.side_m*2
    axis = np.linspace(-extent, extent, 9)
    points = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1).reshape(-1, 3)
    field = coil.field(points) + np.asarray(earth_t)
    magnitude = np.linalg.norm(field, axis=1)
    valid = np.isfinite(magnitude) & (magnitude > 0)
    p, b, strength = points[valid], field[valid], magnitude[valid]
    # Use raw cone sizing: actual scene length is controlled by the slider.
    # Plotly absolute/scaled modes apply an additional grid-dependent normalization.
    # Normalize field vectors only; magnitude is encoded separately.
    arrows = b/strength[:, None]*(extent*0.05*field_arrow_scale) if len(b) else b
    fig = go.Figure()
    if show_arrows:
        fig.add_trace(go.Cone(x=p[:,0], y=p[:,1], z=p[:,2], u=arrows[:,0], v=arrows[:,1], w=arrows[:,2], sizemode='raw', sizeref=1, anchor='tail', showscale=False, colorscale='Blues', name='Field direction', hoverinfo='skip'))
        fig.add_trace(go.Scatter3d(x=p[:,0], y=p[:,1], z=p[:,2], mode='markers', marker=dict(size=2, color=strength*1e6, colorscale='Viridis', colorbar=dict(title='|B| µT')), customdata=strength*1e6, hovertemplate='|B|=%{customdata:.3g} µT<extra></extra>', name='Field magnitude'))
    if show_lines:
        paths = coil.field_lines(earth_t, extent=extent, field_function=grid_interpolator(field.reshape(9,9,9,3), extent)) if live else cached_field_lines(coil, tuple(earth_t), extent)
        # None separators keep independently integrated paths disconnected.
        coordinates = [[], [], []]
        for path in paths:
            for axis_index in range(3):
                coordinates[axis_index].extend(path[:,axis_index].tolist()+[None])
        fig.add_trace(go.Scatter3d(x=coordinates[0], y=coordinates[1], z=coordinates[2], mode='lines', line=dict(color='#458acc', width=3), name='Magnetic field lines', hoverinfo='skip'))
    # Rotate the cube and all three face-mounted coils together.
    corners = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*coil.side_m/2
    corners = corners@coil.rotation.T
    fig.add_trace(go.Mesh3d(x=corners[:,0], y=corners[:,1], z=corners[:,2], alphahull=0, color='#8c9bb1', opacity=0.12, name='Cube', hoverinfo='skip', showlegend=True))
    for i, start in enumerate(corners):
        for j in range(i+1, len(corners)):
            if (i^j) in (1,2,4):
                end = corners[j]
                fig.add_trace(go.Scatter3d(x=[start[0],end[0]], y=[start[1],end[1]], z=[start[2],end[2]], mode='lines', line=dict(color='#8c9bb1', width=3), showlegend=False, hoverinfo='skip'))
    for label, color, winding in zip(('X','Y','Z'), ('#ef8b2c','#4caf70','#8168ed'), coil.coils):
        wire = winding.wire()
        fig.add_trace(go.Scatter3d(x=wire[:,0], y=wire[:,1], z=wire[:,2], mode='lines', line=dict(color=color, width=8), name=f'{label} coil'))
        add_vector(fig, winding.center_m, winding.moment, extent*0.6/0.1*vector_scale,
                   f'm {label}', color, 'A·m²')
    add_vector(fig, (0,0,0), coil.moment, extent*0.6/0.1*vector_scale,
               'm net', '#d3a300', 'A·m²', width=10)
    add_vector(fig, (0,0,0), np.asarray(earth_t), extent*0.6/40e-6*vector_scale,
               'B external', '#00a6a6', 'µT', unit_factor=1e6)
    tau = torque(coil, earth_t)
    if np.linalg.norm(tau) <= np.linalg.norm(coil.moment)*np.linalg.norm(earth_t)*1e-12:
        tau = np.zeros(3)
    add_vector(fig, (0,0,0), tau, extent*0.6/4e-6*vector_scale,
               'Torque τ', '#d528b8', 'µN·m', unit_factor=1e6, width=10)
    fig.update_layout(uirevision='attitude-view', height=580, margin=dict(l=0,r=0,t=0,b=0), scene=dict(aspectmode='cube', xaxis_title='x (m)', yaxis_title='y (m)', zaxis_title='z (m)', xaxis=dict(range=[-extent,extent]), yaxis=dict(range=[-extent,extent]), zaxis=dict(range=[-extent,extent])), legend=dict(orientation='h'))
    return fig



def add_vector(fig, origin, vector, gain, name, color, unit, unit_factor=1, width=8):
    """Display displacement = gain * physical vector, with no normalization.

    Gains are fixed per quantity type so doubling magnitude doubles length.
    Each magnetic moment uses the same gain for direct comparison.
    """
    origin, vector = np.asarray(origin), np.asarray(vector)
    magnitude = np.linalg.norm(vector)
    label = f'{name} = {magnitude*unit_factor:.3g} {unit}'
    tip = origin+gain*vector
    fig.add_trace(go.Scatter3d(x=[origin[0],tip[0]], y=[origin[1],tip[1]], z=[origin[2],tip[2]], mode='lines+text', text=['',label], textposition='top center', textfont=dict(color=color,size=14), line=dict(color=color,width=width), name=name, hovertemplate=label+'<extra></extra>'))
    if magnitude > 0:
        head = gain*vector*0.18
        fig.add_trace(go.Cone(x=[tip[0]], y=[tip[1]], z=[tip[2]], u=[head[0]], v=[head[1]], w=[head[2]], anchor='tip', sizemode='raw', sizeref=1, colorscale=[[0,color],[1,color]], showscale=False, showlegend=False, hoverinfo='skip'))


@lru_cache(maxsize=8)
def cached_field_lines(cube, external_field, extent):
    """Reuse expensive traces when only display controls change."""
    return cube.field_lines(external_field, extent=extent)



def grid_interpolator(field, extent):
    """Trilinear field sampling for responsive live traces (display only)."""
    def sample(points):
        coordinates=(np.asarray(points)+extent)/(2*extent)*(field.shape[0]-1)
        inside=((coordinates>=0)&(coordinates<=field.shape[0]-1)).all(axis=1)
        coordinates=np.clip(coordinates,0,field.shape[0]-1)
        base=np.minimum(np.floor(coordinates).astype(int),field.shape[0]-2)
        fraction=coordinates-base
        result=np.zeros((len(points),3))
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    offsets=np.array([x,y,z])
                    index=base+offsets
                    weights=np.prod(np.where(offsets,fraction,1-fraction),axis=1)
                    values=field[index[:,0],index[:,1],index[:,2]]
                    result+=values*weights[:,None]
        result[~inside]=np.nan
        return result
    return sample
