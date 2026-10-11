import {readFileSync} from 'node:fs';
import {stripTypeScriptTypes} from 'node:module';
import test from 'node:test';
import assert from 'node:assert/strict';
const code=stripTypeScriptTypes(readFileSync(new URL('../src/timing.ts',import.meta.url),'utf8'),{mode:'transform'});
const {timedPose,interval,coordinatedPose,replayProgressLabel}=await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);

test('timeline evaluates the quadratic clock and cubic joint path instead of interpolating samples',()=>{
  const op={start_s:10,duration_s:2,path_clock:{s:[0,1],t_s:[0,2],speeds:[0,1]},
    geometric_path:{knots:[0,1],coefficients:[[[0,0]],[[0,1]],[[1,0]],[[0,0]]]}};
  assert.deepEqual(timedPose(op,10),[0,0]);
  assert.deepEqual(timedPose(op,11),[0.25,0.0625]);
  assert.deepEqual(timedPose(op,12),[1,1]);
  assert.deepEqual(timedPose(op,99),[1,1]);
  assert.deepEqual(timedPose({hold_positions:[1,2,3]},0),[1,2,3]);
  assert.equal(interval([0,1,2],2),1);
  const action={start_s:10,trajectory:{duration_s:2,edges:[
    {start_s:0,end_s:2,q0:[0],q1:[1],coefficients:[[0,0,1]]}
  ]}};
  assert.deepEqual(coordinatedPose(action,11),[0.25]);
  assert.match(replayProgressLabel({replay_in_progress:true,replay:{status:'RUNNING',phase:'PATH_CHECKS',checked_actions:4,requested_actions:66,checked_edges:100,requested_edges:1000}}), /4\/66 actions.*100\/1000 intervals/);
  assert.equal(replayProgressLabel({replay:{status:'REPLAY_TIME_BUDGET'}}), 'Compute budget reached · verification incomplete');
  assert.equal(replayProgressLabel({replay_in_progress:true,replay:{status:'REPLAY_REJECTED'}}), 'Running; previous result: replay rejected');
});
