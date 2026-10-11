import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { stripTypeScriptTypes } from 'node:module';
import { test } from 'node:test';
import * as THREE from 'three';

// Exercise the actual viewer methods without a WebGL context. Resolve Three.js
// imports for the transpiled module; no production test hooks are necessary.
const source = await readFile(new URL('../src/viewer.ts', import.meta.url), 'utf8');
const code = stripTypeScriptTypes(source).replace(/from '(three[^']*)'/g,
  (_, name) => `from '${import.meta.resolve(name)}'`) + '\n//# sourceURL=viewer-test.js';
const { Viewer } = await import(`data:text/javascript;base64,${Buffer.from(code).toString('base64')}`);

test('tool primitives and mesh retain dimensions and follow the flange frame', () => {
  const viewer = Object.assign(Object.create(Viewer.prototype), {
    toolGeometry: new THREE.Group(), tool: new THREE.Group(), source: new THREE.Group(),
  });
  const origin = new THREE.Matrix4().makeTranslation(.2, 0, 0);
  const rows = matrix => Array.from({ length: 4 }, (_, i) =>
    Array.from({ length: 4 }, (_, j) => matrix.elements[4 * j + i]));
  const shapes = [
    { name: 'box', type: 'box', size_m: [.04, .03, .02] },
    { name: 'sphere', type: 'sphere', radius_m: .02 },
    { name: 'cylinder', type: 'cylinder', radius_m: .02, length_m: .1 },
    { name: 'mesh', type: 'mesh', vertices: [[0,0,0],[.01,0,0],[0,.02,0],[0,0,.03]],
      faces: [[0,2,1],[0,1,3],[0,3,2],[1,2,3]] },
  ];
  viewer.setToolModel({ configured: true, visual_bodies: shapes.map(s => ({ ...s, origin: rows(origin) })) });
  assert.equal(viewer.toolGeometry.children.length, 4);
  const cylinder = viewer.toolGeometry.children[2].geometry;
  cylinder.computeBoundingBox();
  const size = cylinder.boundingBox.getSize(new THREE.Vector3());
  assert.ok(size.distanceTo(new THREE.Vector3(.04,.04,.1)) < 1e-7, 'cylinder axis must be local Z');
  const rig = {kinematic_root:'base',kinematic_tip:'flange',visual_nodes:[],kinematic_chain:[
    {parent:'base',child:'flange',origin:rows(new THREE.Matrix4()),axis:[0,0,1],index:0}
  ]};
  viewer.setRobotPose([Math.PI/2],rig,rows(new THREE.Matrix4().makeTranslation(.1,0,0)));
  viewer.toolGeometry.updateMatrixWorld(true);
  const center = viewer.toolGeometry.children[0].getWorldPosition(new THREE.Vector3());
  assert.ok(center.distanceTo(new THREE.Vector3(0,.2,0)) < 1e-10);
  viewer.setToolModel();
  assert.equal(viewer.toolGeometry.children.length, 0, 'unconfigured runs must not inherit a previous tool');
});

test('independent robot instances use their own base and joint transforms', () => {
  const viewer=Object.assign(Object.create(Viewer.prototype),{toolGeometry:new THREE.Group(),tool:new THREE.Group()});
  const rows=matrix=>Array.from({length:4},(_,i)=>Array.from({length:4},(_,j)=>matrix.elements[4*j+i]));
  const rig={kinematic_root:'base',kinematic_tip:'flange',visual_nodes:[{name:'visual',link:'flange',origin:rows(new THREE.Matrix4().makeTranslation(.2,0,0))}],kinematic_chain:[
    {parent:'base',child:'flange',origin:rows(new THREE.Matrix4()),axis:[0,0,1],index:0}
  ]};
  const context=transform=>{const source=new THREE.Group();const mesh=new THREE.Mesh(new THREE.SphereGeometry(.01));mesh.name='visual';source.add(mesh);return {source,tool:new THREE.Group(),toolGeometry:new THREE.Group(),world_from_base:rows(transform),color:0x327aa0};};
  const left=context(new THREE.Matrix4());
  const right=context(new THREE.Matrix4().makeTranslation(1,0,0).multiply(new THREE.Matrix4().makeRotationZ(Math.PI)));
  const offset=rows(new THREE.Matrix4().makeTranslation(.1,0,0));
  const tipLeft=viewer.setRobotPose([Math.PI/2],rig,offset,left);
  const tipRight=viewer.setRobotPose([0],rig,offset,right);
  assert.ok(new THREE.Vector3(...tipLeft).distanceTo(new THREE.Vector3(0,.1,0))<1e-10);
  assert.ok(new THREE.Vector3(...tipRight).distanceTo(new THREE.Vector3(.9,0,0))<1e-10);
  left.source.updateMatrixWorld(true);right.source.updateMatrixWorld(true);
  assert.ok(left.source.getObjectByName('visual').getWorldPosition(new THREE.Vector3()).distanceTo(new THREE.Vector3(0,.2,0))<1e-10);
  assert.ok(right.source.getObjectByName('visual').getWorldPosition(new THREE.Vector3()).distanceTo(new THREE.Vector3(.8,0,0))<1e-10);
});
