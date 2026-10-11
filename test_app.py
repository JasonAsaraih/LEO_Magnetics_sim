from streamlit.testing.v1 import AppTest


def test_initial_conditions_apply_only_on_submit():
    app=AppTest.from_file('app.py').run(timeout=30)
    assert not app.exception
    next(n for n in app.number_input if n.label=='Cube mass (kg)').set_value(2.0)
    assert app.session_state['simulation'].initial_cube.mass_kg==1
    next(b for b in app.button if b.label=='Apply initial conditions / Reset').click().run(timeout=30)
    assert app.session_state['simulation'].initial_cube.mass_kg==2
    assert app.session_state['simulation'].time==0
    # Live playback now belongs to the browser, not Streamlit buttons/timers.
    assert not any(b.label in ('Play','Pause') for b in app.button)
    assert not app.exception


def test_invalid_initial_normal_does_not_replace_applied_model():
    app=AppTest.from_file('app.py').run(timeout=30)
    app.radio[0].set_value('Normal vector')
    next(n for n in app.number_input if n.label=='Body +Z normal z').set_value(0.0)
    next(b for b in app.button if b.label=='Apply initial conditions / Reset').click().run(timeout=30)
    assert app.error
    assert not app.exception
    assert app.session_state['simulation'].initial_cube.mass_kg==1
