import numpy as np
from streamlit.testing.v1 import AppTest


def test_initial_conditions_playback_pause_and_reset():
    app=AppTest.from_file('app.py').run(timeout=60)
    assert not app.exception
    next(c for c in app.checkbox if c.label=='Show connected field lines').set_value(False).run(timeout=30)
    next(n for n in app.number_input if n.label=='Cube mass (kg)').set_value(2.0)
    # The draft widget does not mutate the applied physical model.
    assert app.session_state['simulation'].initial_cube.mass_kg==1
    next(b for b in app.button if b.label=='Apply initial conditions / Reset').click().run(timeout=30)
    assert app.session_state['simulation'].initial_cube.mass_kg==2
    assert app.session_state['simulation'].time==0
    next(s for s in app.slider if s.label=='Time scale (sim seconds / real second)').set_value(10.0).run(timeout=30)
    next(b for b in app.button if b.label=='Play').click().run(timeout=30)
    # Advance the monotonic-clock input without sleeps or browser timers.
    app.session_state['clock'].last_wall-=1
    app.run(timeout=30)
    assert app.session_state['simulation'].time>=10
    np.testing.assert_allclose(app.session_state['simulation'].time,10*app.session_state['clock'].wall_elapsed,atol=1e-8)
    next(b for b in app.button if b.label=='Pause').click().run(timeout=30)
    paused=app.session_state['simulation'].time
    app.run(timeout=30)
    assert app.session_state['simulation'].time==paused
    next(b for b in app.button if b.label=='Reset to applied initial conditions').click().run(timeout=30)
    assert app.session_state['simulation'].time==0
    assert app.session_state['simulation'].initial_cube.mass_kg==2
    assert not app.session_state['clock'].playing
    assert not app.exception
