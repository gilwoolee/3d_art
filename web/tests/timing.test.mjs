import {readFileSync} from 'node:fs';
import {stripTypeScriptTypes} from 'node:module';
import test from 'node:test';
import assert from 'node:assert/strict';
const code=stripTypeScriptTypes(readFileSync(new URL('../src/timing.ts',import.meta.url),'utf8'),{mode:'transform'});
const {timedPose,interval,coordinatedPose,replayProgressLabel,replayState,playbackReady}=await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);

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


test('a verified partial replay never becomes a complete build label',()=>{
  const partial={status:'PARTIAL_SECTION_SCHEDULE',full_schedule:false,
    completed_strokes:2,requested_strokes:39,replay:{status:'CERTIFIED_PREFIX_REPLAY'}};
  assert.equal(replayState(partial),'prefix');
  assert.equal(replayState({...partial,replay:{status:'CERTIFIED_GEOMETRIC_REPLAY'}}),'prefix');
  assert.equal(replayState({...partial,replay_in_progress:true}),'unverified');
  assert.equal(replayState({...partial,full_schedule:true,completed_strokes:39,
    replay:{status:'CERTIFIED_GEOMETRIC_REPLAY'}}),'full');
  assert.match(replayProgressLabel(partial),/Verified prefix.*complete build pending/);
  const saved={...partial,full_schedule:true,completed_strokes:39,
    requires_final_verification:true,replay:undefined};
  assert.equal(playbackReady(saved),false);
  for(const status of ['RUNNING','REPLAY_REJECTED','REPLAY_ERROR','CERTIFIED_PREFIX_REPLAY']){
    assert.equal(playbackReady({...saved,replay:{status,collision_scope:'full_robot'}}),false);
  }
  const passed={...saved,replay:{status:'CERTIFIED_GEOMETRIC_REPLAY',collision_scope:'full_robot'}};
  assert.equal(playbackReady(passed),true);
  assert.equal(playbackReady({...passed,replay_in_progress:true}),false);
  assert.equal(playbackReady({...passed,replay:{...passed.replay,collision_scope:'pen_candidate'}}),false);
  assert.equal(playbackReady(partial),true); // Historical diagnostic prefixes remain viewable.
});
