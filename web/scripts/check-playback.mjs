// Check the viewer as a consumer of a separately verified, immutable schedule.
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import fs from 'node:fs/promises';
import {existsSync} from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {chromium} from 'playwright';

const root=fileURLToPath(new URL('../..',import.meta.url));
const run=process.argv[2];
assert(run&&/^[A-Za-z0-9_-]+$/.test(run),'Provide one saved run ID');
const directory=path.join(root,'runs',run);
const base=process.env.PEN_ART_VIEWER_URL??'http://127.0.0.1:8765';
const digest=data=>createHash('sha256').update(data).digest('hex');
const files=['offline_dual_arm_plan.json','offline_dual_arm_validation.json'];
const before=await Promise.all(files.map(name=>fs.readFile(path.join(directory,name))));
const plan=JSON.parse(before[0]);
const validation=JSON.parse(before[1]);
assert(plan.requires_final_verification);
assert.equal(validation.source_plan_sha256,digest(before[0]));
assert.equal(validation.status,'CERTIFIED_GEOMETRIC_REPLAY');
assert.equal(validation.collision_scope,'full_robot');
const browser=await chromium.launch({
  executablePath:process.env.PEN_ART_BROWSER_EXECUTABLE??
    (existsSync('/opt/google/chrome/chrome')?'/opt/google/chrome/chrome':undefined),
  headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
try{
  const page=await browser.newPage({viewport:{width:1500,height:1100}});
  const errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  // An absent matching verdict must withhold animation, even for a full plan.
  await page.route('**/offline_dual_arm_validation.json',route=>route.fulfill({status:404}));
  await page.goto(`${base}/?run=${run}&stage=motion`);
  await page.waitForFunction(()=>document.querySelector('#badge')?.textContent===
    'Awaiting full-robot verification',null,{timeout:120000});
  assert.equal(await page.locator('#dual-play').count(),0);
  assert(await page.locator('#robot-play').isHidden());
  await page.unroute('**/offline_dual_arm_validation.json');
  await page.reload();
  await page.locator('#dual-play').waitFor({timeout:120000});
  assert.equal(await page.locator('#badge').innerText(),'One-arm geometric replay passed');
  assert.equal(Number(await page.locator('#dual-seek').getAttribute('max')),
    plan.modeled_duration_s);
  const requests=[];
  page.on('request',request=>requests.push({method:request.method(),url:request.url()}));
  await page.locator('#dual-play').click();
  await page.waitForFunction(()=>Number(document.querySelector('#dual-seek').value)>0);
  assert.equal(await page.locator('#dual-play').innerText(),'Pause one-arm build');
  await page.locator('#dual-play').click();
  await page.locator('#dual-seek').evaluate(element=>{
    element.value=element.max;
    element.dispatchEvent(new Event('input',{bubbles:true}));
  });
  assert.equal(Number(await page.locator('#dual-seek').inputValue()),plan.modeled_duration_s);
  assert.equal(await page.locator('#dual-play').innerText(),'Play full one-arm build');
  assert.equal(errors.length,0,errors.join('\n'));
  assert(requests.every(request=>request.method==='GET'),'Playback must not submit backend work');
  const after=await Promise.all(files.map(name=>fs.readFile(path.join(directory,name))));
  assert.deepEqual(after.map(digest),before.map(digest),'Playback must leave saved evidence intact');
  await page.screenshot({path:path.join(directory,'modular_playback_completed.png')});
  const receipt={status:'PASSED',run,source_plan_sha256:digest(before[0]),
    source_validation_sha256:digest(before[1]),unverified_playback_blocked:true,
    verified_playback_started:true,complete_horizon_seek_passed:true,
    backend_requests_during_playback:requests,artifacts_unchanged:true,errors};
  await fs.writeFile(path.join(directory,'modular_browser_receipt.json'),JSON.stringify(receipt,null,2));
  console.log(JSON.stringify(receipt));
}finally{
  await browser.close();
}
