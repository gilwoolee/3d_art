// Exercise visible progress with the full dense-cube artifacts over a slow connection.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root=fileURLToPath(new URL('../..',import.meta.url));
const base=process.env.PEN_ART_VIEWER_URL??'http://127.0.0.1:8765';
const run='cube_stl_forward_turntable_v1';
const browser=await chromium.launch({
  executablePath:process.env.PEN_ART_BROWSER_EXECUTABLE??(existsSync('/opt/google/chrome/chrome')?'/opt/google/chrome/chrome':undefined),
  headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
try {
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[];page.on('pageerror',error=>errors.push(error.message));
  await page.addInitScript(()=>{
    window.playbackStatuses=[];
    new MutationObserver(()=>{
      const element=document.querySelector('#playback-load');
      const value=element?.textContent;
      if(value&&!element.classList.contains('hidden')&&window.playbackStatuses.at(-1)!==value)
        window.playbackStatuses.push(value);
    }).observe(document,{childList:true,subtree:true,characterData:true,attributes:true,attributeFilter:['class']});
  });
  const connection=await page.context().newCDPSession(page);
  await connection.send('Network.enable');
  await connection.send('Network.emulateNetworkConditions',{
    offline:false,latency:30,downloadThroughput:10_000_000,uploadThroughput:1_000_000,
  });
  await page.goto(`${base}/?workbench=1&run=${run}&stage=motion`);
  await page.waitForFunction(()=>document.querySelector('#playback-load')?.textContent.includes('Downloading motion plan:'),null,{timeout:60000});
  assert(await page.locator('#robot-play').isVisible());
  assert(await page.locator('#robot-play').isDisabled());
  await page.screenshot({path:`${root}/runs/${run}/download_progress.png`});
  await page.waitForFunction(()=>{
    const button=document.querySelector('#robot-play');
    return button&&!button.disabled&&button.textContent.includes('Play full');
  },null,{timeout:120000});
  const statuses=await page.evaluate(()=>window.playbackStatuses);
  for(const label of ['motion plan','validation report']) {
    assert(statuses.some(value=>value.includes(`Downloading ${label}:`)&&/\([1-9]\d?%\)/.test(value)),`${label} needs intermediate progress`);
    assert(statuses.some(value=>value===`Downloaded ${label}. Checking saved evidence…`));
  }
  assert.equal(await page.locator('#badge').innerText(),'One-arm geometric replay passed');
  await page.locator('#robot-play').click();await page.waitForTimeout(1000);
  await page.locator('#robot-play').click();
  assert(Number(await page.locator('#dual-seek').inputValue())>0);
  assert.equal(errors.length,0,errors.join('\n'));
  const receipt={status:'PASSED_DOWNLOAD_PROGRESS',run,download_bytes_per_second:10_000_000,statuses,errors};
  await fs.writeFile(`${root}/runs/${run}/download_progress_check.json`,JSON.stringify(receipt,null,2)+'\n');
  console.log(JSON.stringify({status:receipt.status,progress_updates:statuses.length,errors}));
} finally {await browser.close();}
