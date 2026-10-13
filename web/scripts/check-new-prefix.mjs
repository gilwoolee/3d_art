// Verify a new, explicitly partial robot playback without treating it as a full build.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { chromium } from 'playwright';

const run=process.argv[2];
assert(run&&/^[A-Za-z0-9_-]+$/.test(run));
const root=new URL('../..',import.meta.url).pathname;
const plan=JSON.parse(await fs.readFile(`${root}/runs/${run}/offline_dual_arm_plan.json`,'utf8'));
assert.equal(plan.full_schedule,false);
const browser=await chromium.launch({executablePath:'/opt/google/chrome/chrome',headless:true,
  args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try {
  const page=await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];page.on('pageerror',error=>errors.push(error.message));
  const started=Date.now();
  await page.goto(`http://127.0.0.1:8765/?run=${run}&stage=motion`);
  const play=page.locator('#robot-play');
  await play.waitFor({state:'visible'});
  await page.waitForFunction(()=>!document.querySelector('#robot-play').disabled);
  assert.equal(await page.locator('#run-select').inputValue(),run);
  assert.match(await play.textContent(),/prefix/i);
  const badge=await page.locator('#badge').textContent();
  assert.match(badge,/Verified prefix/i);
  assert.match(badge,new RegExp(`${plan.completed_strokes}/${plan.requested_strokes}`));
  const loadedMs=Date.now()-started;
  await play.click();
  await page.waitForTimeout(1000);
  const time=Number(await page.locator('#dual-seek').inputValue());
  assert(time>0,'Robot playback must advance');
  await play.click();
  const screenshot=`${root}/runs/${run}/new_prefix_playback.png`;
  await page.screenshot({path:screenshot,fullPage:true});
  assert.deepEqual(errors,[]);
  const result={run,loaded_ms:loadedMs,observed_time_s:time,badge,
    completed_strokes:plan.completed_strokes,requested_strokes:plan.requested_strokes,
    screenshot,errors,status:'VERIFIED_PARTIAL_BROWSER_PLAYBACK'};
  await fs.writeFile(`${root}/runs/${run}/new_prefix_browser_check.json`,JSON.stringify(result,null,2));
  console.log(JSON.stringify(result));
} finally {await browser.close();}
