"""Optional real-browser smoke check. Run against Streamlit or player.html.

Install playwright into your test venv and provide Chromium via CHROMIUM_PATH
(default: /usr/bin/chromium). No browser download is required on this cloud host.
"""
import os
import sys
from playwright.sync_api import sync_playwright

with sync_playwright() as p:
    browser=p.chromium.launch(executable_path=os.environ.get('CHROMIUM_PATH','/usr/bin/chromium'),headless=True,args=['--no-sandbox','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1400,'height':1100})
    errors=[]
    page.on('pageerror',lambda error: errors.append(str(error)))
    page.goto(sys.argv[1] if len(sys.argv)>1 else 'http://127.0.0.1:8501',wait_until='domcontentloaded')
    frame=None
    for _ in range(300):
        frame=next((f for f in page.frames if f.query_selector('#play')),None)
        if frame:
            break
        page.wait_for_timeout(100)
    assert frame is not None,'Browser player iframe not found'
    frame.wait_for_function('window.playerDiagnostics && !playerDiagnostics().busy')
    assert frame.locator('#speed').get_attribute('type')=='range'
    frame.locator('#speed').evaluate('(e)=>{e.value=10;e.dispatchEvent(new Event("input"))}')
    frame.locator('#play').click()
    frame.wait_for_function('playerDiagnostics().time>5')
    frame.locator('#view canvas').scroll_into_view_if_needed()
    start=frame.evaluate('playerDiagnostics()')
    rect=frame.locator('#view canvas').bounding_box()
    x,y=rect['x']+rect['width']*.45,rect['y']+rect['height']*.4
    page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+140,y+80,steps=20);page.mouse.up()
    after=frame.evaluate('playerDiagnostics()')
    assert start['camera']!=after['camera'],'Camera did not move during playback'
    assert after['playing']
    # Display changes must not restart the player or reset simulated time.
    frame.locator('#arrows').uncheck()
    frame.locator('#lines').uncheck()
    assert frame.evaluate('playerDiagnostics().generation')==start['generation']
    frame.locator('#play').click()
    frame.wait_for_function('!playerDiagnostics().busy && Math.abs(playerDiagnostics().target-playerDiagnostics().time)<1e-8')
    paused=frame.evaluate('playerDiagnostics()')
    assert abs(paused['time']-10*paused['realTime'])<1e-7
    page.wait_for_timeout(300)
    assert frame.evaluate('playerDiagnostics().time')==paused['time']
    frame.locator('#reset').click()
    frame.wait_for_function('!playerDiagnostics().busy')
    assert frame.evaluate('playerDiagnostics().time')==0
    assert not errors,errors
    print('Browser: 10× time slider, camera dragging during playback, independent toggles, pause, and reset passed')
    browser.close()
