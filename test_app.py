from streamlit.testing.v1 import AppTest


def test_initial_conditions_apply_only_on_submit():
    app=AppTest.from_file('app.py').run(timeout=30)
    assert not app.exception
    next(n for n in app.number_input if n.label=='Cube mass (kg)').set_value(2.0)
    assert app.session_state['simulation'].initial_cube.mass_kg==1
    next(b for b in app.button if b.label=='Update initial state').click().run(timeout=30)
    assert app.session_state['simulation'].initial_cube.mass_kg==2
    assert app.session_state['simulation'].time==0
    # Live playback now belongs to the browser, not Streamlit buttons/timers.
    assert not any(b.label in ('Play','Pause') for b in app.button)
    assert not app.exception


def test_invalid_initial_normal_does_not_replace_applied_model():
    app=AppTest.from_file('app.py').run(timeout=30)
    app.radio[0].set_value('Normal vector')
    next(n for n in app.number_input if n.label=='Body +Z normal z').set_value(0.0)
    next(b for b in app.button if b.label=='Update initial state').click().run(timeout=30)
    assert app.error
    assert not app.exception
    assert app.session_state['simulation'].initial_cube.mass_kg==1


def test_update_previews_orientation_and_resets_even_for_same_inputs():
    import numpy as np
    app=AppTest.from_file('app.py').run(timeout=30)
    revision=app.session_state['initial_revision']
    next(s for s in app.slider if s.label=='Body +Z tilt from world +z (°)').set_value(90)
    next(b for b in app.button if b.label=='Update initial state').click().run(timeout=30)
    np.testing.assert_allclose(app.session_state['simulation'].initial_cube.rotation[:,2],[1,0,0],atol=1e-15)
    assert app.session_state['simulation'].time==0
    assert app.session_state['initial_revision']==revision+1
    next(b for b in app.button if b.label=='Update initial state').click().run(timeout=30)
    assert app.session_state['initial_revision']==revision+2
    assert not app.exception


def test_pid_settings_and_targets_apply_with_initial_state():
    import numpy as np
    app=AppTest.from_file('app.py').run(timeout=30)
    next(c for c in app.checkbox if c.label=='Enable PID current control').set_value(True)
    next(n for n in app.number_input if n.label=='Final pitch (°)').set_value(30.0)
    next(n for n in app.number_input if n.label=='X current limit (A)').set_value(.01)
    next(b for b in app.button if b.label=='Update initial state').click().run(timeout=30)
    settings=app.session_state['controller_config']
    assert settings['enabled']
    assert settings['targetEulerDegrees']==[0,30,0]
    np.testing.assert_allclose(settings['targetRate'],[0,0,0])
    assert settings['currentLimits'][0]==.01
    assert not app.exception
