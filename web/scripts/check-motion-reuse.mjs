// Verify the published cache inventory, seek controls, and small-screen layout.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root=fileURLToPath(new URL('../..',import.meta.url));
const base=process.env.PEN_ART_VIEWER_URL??'http://127.0.0.1:8765';
const browser=await chromium.launch({
  executablePath:process.env.PEN_ART_BROWSER_EXECUTABLE??(existsSync('/opt/google/chrome/chrome')?'/opt/google/chrome/chrome':undefined),
  headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
const results=[];
try {
  for(const id of ['phase1_cache_cold','phase1_cache_warm','phase2_diffusion_cold']) {
    const page=await browser.newPage({viewport:{width:1500,height:1100}});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    const data=await (await fetch(`${base}/api/runs/${id}/artifacts/viewer_motion_reuse.json`)).json();
    await page.goto(`${base}/?workbench=1&run=${id}`);
    await page.locator('#reuse-strokes').waitFor();
    assert.equal(await page.locator('#run-scope').inputValue(),'reuse');
    assert.equal(await page.locator('#run-select option').count(),3);
    assert.equal(await page.locator('[data-reuse-stroke]').count(),9);
    assert.equal(await page.locator('#badge').innerText(),'One-arm geometric replay passed');
    assert.equal(await page.locator('#reuse-benchmark tbody tr').count(),3);
    assert.match(await page.locator('#motion-reuse-panel').innerText(),/not 15 objects or builds/);
    for(const index of [0,8]) {
      await page.locator(`[data-reuse-stroke="${index}"]`).click();
      assert(Math.abs(Number(await page.locator('#dual-seek').inputValue())-data.strokes[index].draw_start_s)<1e-6);
      assert.match(await page.locator('#reuse-selected').innerText(),new RegExp(data.strokes[index].section));
    }
    await page.locator('#motion-reuse-panel').screenshot({path:`${root}/runs/${id}/cache_inventory.png`});
    await page.setViewportSize({width:390,height:844});
    await page.waitForTimeout(200);
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth+1),'Page overflows mobile width');
    assert.equal(errors.length,0,errors.join('\n'));
    results.push({id,strokes:9,library_records:data.library_entries.length,errors});
    await page.close();
  }
  await fs.writeFile(`${root}/runs/trajectory_reuse_implementation/viewer_check.json`,JSON.stringify({status:'PASS',results},null,2)+'\n');
  console.log(JSON.stringify({status:'PASS',results}));
} finally {await browser.close();}
