"""Optional browser check of controlled currents and exported peak logs."""
import csv
import os
import sys
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright

with sync_playwright() as p:
    browser=p.chromium.launch(executable_path=os.environ.get('CHROMIUM_PATH','/usr/bin/chromium'),headless=True,args=['--no-sandbox','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1600,'height':1200})
    errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(sys.argv[1] if len(sys.argv)>1 else 'http://127.0.0.1:8501')
    page.get_by_text('Enable PID current control',exact=True).click()
    page.get_by_label('Final pitch (°)',exact=True).fill('30')
    page.get_by_label('X current limit (A)',exact=True).fill('0.01')
    page.get_by_role('button',name='Update initial state',exact=True).click()
    frame=None
    for _ in range(200):
        try:
            frame=next((f for f in page.frames if f.query_selector('#play')),None)
            if frame and frame.evaluate('window.playerDiagnostics && playerDiagnostics().controller && !playerDiagnostics().busy'):
                break
        except Exception:
            pass
        page.wait_for_timeout(100)
    assert frame is not None
    initial=frame.evaluate('playerDiagnostics()')
    assert initial['controller']['mode']=='attitude'
    assert max(initial['controller']['peaks']['required'])>.01
    frame.locator('#speed').evaluate('(e)=>{e.value=20;e.dispatchEvent(new Event("input"))}')
    frame.locator('#play').click();frame.wait_for_function('playerDiagnostics().time>5')
    frame.locator('#play').click()
    frame.wait_for_function('!playerDiagnostics().busy && Math.abs(playerDiagnostics().target-playerDiagnostics().time)<1e-8')
    final=frame.evaluate('playerDiagnostics()')
    assert final['currents']!=initial['currents']
    assert max(final['controller']['peaks']['requiredDerivative'])>0
    assert abs(final['currents'][0])<=.01000001
    with page.expect_download() as download:
        frame.locator('#export-peaks').click()
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'peaks.csv';download.value.save_as(path)
        with path.open() as file:
            rows=list(csv.DictReader(file))
        assert len(rows)==3 and float(rows[0]['max_required_A'])>.01
    assert not errors,errors
    print('Browser PID: dynamic currents, clipping, required/current-derivative peaks, and three-coil CSV export passed')
    browser.close()
