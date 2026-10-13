// Open the side-by-side viewer on one saved run, play it, and save a screenshot.
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {chromium} from 'playwright';

const run=process.argv[2], output=process.argv[3]??'simple-viewer.png';
assert(run&&/^[A-Za-z0-9_-]+$/.test(run),'Provide one saved run ID');
const base=process.env.PEN_ART_VIEWER_URL??'http://127.0.0.1:8765';
const browser=await chromium.launch({
  executablePath:process.env.PEN_ART_BROWSER_EXECUTABLE??
    (existsSync('/opt/google/chrome/chrome')?'/opt/google/chrome/chrome':undefined),
  headless:true,args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
try{
  const page=await browser.newPage({viewport:{width:1800,height:900}});
  const errors=[];
  page.on('pageerror',error=>errors.push(String(error)));
  page.on('console',message=>{if(message.type()==='error'&&!message.text().includes('404'))errors.push(message.text());});
  page.on('response',response=>{if(response.status()>=400&&!response.url().endsWith('/favicon.ico'))errors.push(`${response.status()} ${response.url()}`);});
  await page.goto(`${base}/?run=${run}`);
  await page.waitForFunction(()=>!document.querySelector('#simple-play')?.disabled,null,{timeout:180000});
  await page.waitForFunction(()=>document.querySelector('#simple-strokes-note')?.textContent.includes('strokes'),null,{timeout:60000});
  assert.equal(await page.locator('.simple-view canvas').count(),3);
  const notes=await page.locator('.simple-panels p').allTextContents();
  const before=await page.locator('#simple-clock').textContent();
  assert.equal(await page.locator('#simple-rate').inputValue(),'30');
  await page.click('#simple-play');
  await page.waitForTimeout(6000);
  await page.click('#simple-play');
  const after=await page.locator('#simple-clock').textContent();
  assert.notEqual(before,after,'Playback did not advance');
  await page.screenshot({path:output});
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({run,notes,before,after,status:await page.locator('#simple-status').textContent(),output}));
}finally{await browser.close();}
