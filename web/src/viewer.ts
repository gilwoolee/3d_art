import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

export interface Curve { id: string; segments: number[][][]; role?: string; stage?:string; color?:number; diameter_m?: number; length_m: number; closed?: boolean }
const curveColors:Record<string,number>={framework:0x52745a,artistic:0xb77a3f,connector:0x537b8a,brace:0x8c695d,base:0x52745a,support:0x52745a};
function displayRole(curve:Curve):string {
  if(curve.role&&['connector','brace','base','support'].includes(curve.role))return curve.role;
  return curve.stage==='framework'?'framework':'artistic';
}
interface ToolBody {
  name:string; origin:number[][]; type:'box'|'sphere'|'cylinder'|'mesh';
  size_m?:number[]; radius_m?:number; length_m?:number; vertices?:number[][]; faces?:number[][];
}
interface ToolModel {configured:boolean; visual_bodies:ToolBody[]}

export interface RobotRig {
  kinematic_root:string; kinematic_tip:string;
  kinematic_chain:{parent:string;child:string;origin:number[][];axis:number[]|null;index:number|null}[];
  visual_nodes:{name:string;link:string;origin:number[][]}[];
}

export interface CellArmView {
  id:string; rig:RobotRig; url:string; world_from_base:number[][];
  flange_to_tcp:number[][]; initial_positions:number[]; tool_model:ToolModel;
}

// One three.js view: a model, strokes, or the robot cell with its motion.
export class Viewer {
  renderer: THREE.WebGLRenderer;
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(35, 1, .001, 10);
  controls: OrbitControls;
  model = new THREE.Group();
  curves = new THREE.Group();
  platform = new THREE.Group();
  deposited = new THREE.Group();
  cell = new THREE.Group();
  grid: THREE.GridHelper;
  private loader = new GLTFLoader();
  private loadGeneration = 0;
  private cellGeneration = 0;
  private cellReference: THREE.Object3D | null = null;
  private cellArms = new Map<string,{view:CellArmView;source:THREE.Object3D;tool:THREE.Group;toolGeometry:THREE.Group;color:number}>();
  private extent = .15;
  private center = new THREE.Vector3(0, 0, .045);

  constructor(container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor('#f8faf4');
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.3;
    container.prepend(this.renderer.domElement);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = .02;
    this.controls.maxDistance = 1.4;
    this.controls.autoRotateSpeed = .8;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xa4b29c, 2.5));
    const light = new THREE.DirectionalLight(0xffefd9, 3);
    light.position.set(.3, -.4, .5);
    this.scene.add(light);
    const fill = new THREE.DirectionalLight(0xe4ecff, 1.4);
    fill.position.set(-.4, .1, .3);
    this.scene.add(fill);
    this.grid = new THREE.GridHelper(.28, 14, 0xcbd3c6, 0xe2e8dc);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -.0003;
    this.scene.add(this.grid, this.model, this.curves, this.platform, this.deposited, this.cell);
    const observer = new ResizeObserver(() => {
      const { width, height } = container.getBoundingClientRect();
      this.renderer.setSize(width, height);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
    });
    observer.observe(container);
    let down = { x: 0, y: 0 };
    this.renderer.domElement.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY }; });
    // A click on a stroke colours it.
    this.renderer.domElement.addEventListener('pointerup', e => {
      if (Math.hypot(e.clientX - down.x, e.clientY - down.y) > 4) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      ray.setFromCamera(new THREE.Vector2((e.clientX - rect.left) / rect.width * 2 - 1, -(e.clientY - rect.top) / rect.height * 2 + 1), this.camera);
      const hits = ray.intersectObjects(this.curves.children.filter(c => c.visible));
      this.select(hits.length ? hits[0].object.userData.index as number : -1);
    });
    this.view();
    const draw = () => {
      requestAnimationFrame(draw);
      this.controls.update();
      const width = container.clientWidth, height = container.clientHeight;
      if (width && height) {
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
      }
      this.renderer.setViewport(0, 0, width, height);
      this.renderer.render(this.scene, this.camera);
    };
    draw();
  }

  async load(url: string, onProgress: (n: number) => void) {
    const generation = ++this.loadGeneration;
    const gltf = await this.loader.loadAsync(url, event => onProgress(event.total ? event.loaded / event.total : 0));
    if (generation !== this.loadGeneration) { this.clear(gltf.scene); return; }
    this.clear(this.model);
    this.model.add(gltf.scene);
    const box = new THREE.Box3().setFromObject(gltf.scene);
    box.getCenter(this.center);
    const size = box.getSize(new THREE.Vector3());
    this.extent = Math.max(size.x, size.y, size.z);
    this.controls.minDistance = this.extent * .08;
    this.controls.maxDistance = this.extent * 10;
    this.grid.scale.setScalar(this.extent / .15);
    this.grid.position.set(0,0,-.0003);
    this.view();
  }

  clear(group: THREE.Group) {
    group.traverse(object => {
      if (object instanceof THREE.Mesh || object instanceof THREE.Line) {
        object.geometry.dispose();
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          for (const value of Object.values(material)) if (value instanceof THREE.Texture) value.dispose();
          material.dispose();
        }
      }
    });
    group.clear();
  }

  private setToolModel(tool:ToolModel, target:THREE.Group) {
    this.clear(target);
    if (!tool.configured) return;
    for (const body of tool.visual_bodies) {
      let geometry:THREE.BufferGeometry;
      if (body.type==='box') geometry=new THREE.BoxGeometry(...body.size_m as [number,number,number]);
      else if (body.type==='sphere') geometry=new THREE.SphereGeometry(body.radius_m,24,16);
      else if (body.type==='cylinder') {
        geometry=new THREE.CylinderGeometry(body.radius_m,body.radius_m,body.length_m,32);
        geometry.rotateX(Math.PI/2); // Collision cylinders use local Z, Three.js uses Y.
      } else {
        geometry=new THREE.BufferGeometry();
        geometry.setAttribute('position',new THREE.Float32BufferAttribute(body.vertices!.flat(),3));
        geometry.setIndex(body.faces!.flat()); geometry.computeVertexNormals();
      }
      const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0x5b7c94,roughness:.65}));
      mesh.name=`tool/${body.name}`;
      mesh.matrixAutoUpdate=false;
      mesh.matrix.fromArray(body.origin.flat()).transpose();
      target.add(mesh);
    }
  }

  private setRobotPose(q:number[], rig:RobotRig, flangeToTcp:number[][], context:{
    source:THREE.Object3D;tool:THREE.Group;toolGeometry:THREE.Group;world_from_base:number[][];color:number;
  }) {
    const matrix = (values:number[][]) => new THREE.Matrix4().fromArray(values.flat()).transpose();
    const frames = new Map<string,THREE.Matrix4>([[rig.kinematic_root,matrix(context.world_from_base)]]);
    const {source, tool, toolGeometry, color} = context;
    for (const joint of rig.kinematic_chain) {
      const frame = frames.get(joint.parent)!.clone().multiply(matrix(joint.origin));
      if (joint.index !== null && joint.axis) {
        frame.multiply(new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...joint.axis as [number,number,number]).normalize(),q[joint.index]));
      }
      frames.set(joint.child,frame);
    }
    for (const visual of rig.visual_nodes) {
      const node = source.getObjectByName(visual.name);
      if (!node) throw new Error(`Robot visual node is missing: ${visual.name}`);
      node.matrixAutoUpdate = false;
      node.matrix.copy(frames.get(visual.link)!).multiply(matrix(visual.origin));
      node.matrixWorldNeedsUpdate = true;
    }
    this.clear(tool);
    const flange = frames.get(rig.kinematic_tip)!;
    toolGeometry.matrixAutoUpdate=false;
    toolGeometry.matrix.copy(flange);
    toolGeometry.matrixWorldNeedsUpdate=true;
    const tcp = flange.clone().multiply(matrix(flangeToTcp));
    const tip = new THREE.Vector3().setFromMatrixPosition(tcp);
    tool.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3().setFromMatrixPosition(flange), tip
    ]),new THREE.LineBasicMaterial({color})));
    const marker = new THREE.Mesh(new THREE.SphereGeometry(.002,10,8),new THREE.MeshStandardMaterial({color}));
    marker.position.copy(tip); tool.add(marker);
    return tip.toArray();
  }

  clearCell() {
    ++this.cellGeneration;
    this.cellArms.clear();this.clear(this.cell);
    this.cellReference=null;
  }

  async loadCell(arms:CellArmView[], reference?:{url:string;world_from_build:number[][]}) {
    ++this.loadGeneration; // An older model download must not reset this cell's camera.
    this.clearCell();const generation=this.cellGeneration;
    const loaded=await Promise.all(arms.map(async view=>({view,gltf:await this.loader.loadAsync(view.url)})));
    if(generation!==this.cellGeneration){for(const {gltf} of loaded)this.clear(gltf.scene);return;}
    for(const [index,{view,gltf}] of loaded.entries()) {
      const tool=new THREE.Group(),toolGeometry=new THREE.Group();
      const color=index===0?0x327aa0:0xc0813f;
      this.cell.add(gltf.scene,tool,toolGeometry);
      this.cellArms.set(view.id,{view,source:gltf.scene,tool,toolGeometry,color});
      this.setToolModel(view.tool_model,toolGeometry);
      this.setCellArmPose(view.id,view.initial_positions);
    }
    if(reference){
      const gltf=await this.loader.loadAsync(reference.url);
      if(generation!==this.cellGeneration){this.clear(gltf.scene);return;}
      gltf.scene.matrixAutoUpdate=false;
      gltf.scene.matrix.fromArray(reference.world_from_build.flat()).transpose();
      gltf.scene.traverse(object=>{
        if(object instanceof THREE.Mesh){
          object.material=new THREE.MeshStandardMaterial({color:0xb2b9a9,transparent:true,opacity:.55,roughness:.8});
        }
      });
      this.cell.add(gltf.scene);
      this.cellReference=gltf.scene;
    }
    const box=new THREE.Box3().setFromObject(this.cell);box.getCenter(this.center);
    const size=box.getSize(new THREE.Vector3());this.extent=Math.max(size.x,size.y,size.z);
    this.grid.scale.setScalar(this.extent/.15);this.grid.position.x=this.center.x;this.grid.position.y=this.center.y;
    this.controls.minDistance=this.extent*.08;this.controls.maxDistance=this.extent*10;
    this.view();
  }

  setCellArmPose(identity:string,q:number[]) {
    const row=this.cellArms.get(identity);if(!row)throw new Error(`Missing cell arm ${identity}`);
    return this.setRobotPose(q,row.view.rig,row.view.flange_to_tcp,{...row,world_from_base:row.view.world_from_base});
  }

  setCellReferenceVisible(visible:boolean) {
    if(this.cellReference)this.cellReference.visible=visible;
  }

  focus(subject:'cell'|'curves') {
    this.frameBox(new THREE.Box3().setFromObject(subject==='cell'?this.cell:this.curves));
  }

  focusPoints(points:number[][]) {
    this.frameBox(new THREE.Box3().setFromPoints(points.map(p=>new THREE.Vector3(p[0],p[1],p[2]))));
  }

  private frameBox(box:THREE.Box3) {
    if(box.isEmpty())return;
    box.getCenter(this.center);
    const size=box.getSize(new THREE.Vector3());
    this.extent=Math.max(.05,size.x,size.y,size.z);
    this.controls.minDistance=this.extent*.08;this.controls.maxDistance=this.extent*10;
    this.view();
  }

  // The build plate and its stand, one block from the table up to the plate top.
  setPlatform(center: number[], top: number, table: number, width: number, depth: number) {
    this.clear(this.platform);
    const height = Math.max(.006, top-table);
    const base = new THREE.Mesh(new THREE.BoxGeometry(width,depth,height),
      new THREE.MeshStandardMaterial({color:0xa6b5a3, roughness:.9, transparent:true, opacity:.65}));
    base.position.set(center[0],center[1],top-height/2);
    this.platform.add(base);
  }

  clearSequence() { this.clear(this.deposited); this.depositedTubes=undefined; }

  // Deposited wire as 1 mm tubes, like the stroke view: one instanced
  // cylinder per segment, reused across frames as the wire grows and turns.
  private depositedTubes?: THREE.InstancedMesh;
  setDepositedPaths(paths:number[][][]) {
    const segments=paths.reduce((count,points)=>count+Math.max(0,points.length-1),0);
    let tubes=this.depositedTubes;
    if (!tubes || tubes.instanceMatrix.count < segments) {
      this.clear(this.deposited);
      const capacity=Math.max(1024, 2 ** Math.ceil(Math.log2(Math.max(1, segments))));
      tubes=new THREE.InstancedMesh(
        new THREE.CylinderGeometry(.0005,.0005,1,8,1,false),
        new THREE.MeshStandardMaterial({color:0xb77a3f,roughness:.6}),
        capacity,
      );
      tubes.frustumCulled=false;
      this.deposited.add(tubes);
      this.depositedTubes=tubes;
    }
    const up=new THREE.Vector3(0,1,0), a=new THREE.Vector3(), b=new THREE.Vector3();
    const direction=new THREE.Vector3(), turn=new THREE.Quaternion(), matrix=new THREE.Matrix4();
    const scale=new THREE.Vector3(1,1,1);
    let index=0;
    for (const points of paths) {
      for (let i=1;i<points.length;i++) {
        a.set(points[i-1][0],points[i-1][1],points[i-1][2]);
        b.set(points[i][0],points[i][1],points[i][2]);
        direction.subVectors(b,a);
        const length=direction.length();
        if (length < 1e-9) continue;
        turn.setFromUnitVectors(up,direction.divideScalar(length));
        scale.set(1,length+.0005,1);
        matrix.compose(a.add(b).multiplyScalar(.5),turn,scale);
        tubes.setMatrixAt(index++,matrix);
      }
    }
    tubes.count=index;
    tubes.instanceMatrix.needsUpdate=true;
  }

  setCurves(curves: Curve[]) {
    this.clear(this.curves);
    for (const [index, curve] of curves.entries()) {
      const path = new THREE.CurvePath<THREE.Vector3>();
      for (const segment of curve.segments) {
        const points = segment.map(p => new THREE.Vector3(...p as [number, number, number]));
        path.add(new THREE.CubicBezierCurve3(points[0], points[1], points[2], points[3]));
      }
      const geometry = new THREE.TubeGeometry(path, Math.min(700, Math.max(24, curve.segments.length * 10)), (curve.diameter_m ?? .001) / 2, 6, curve.closed ?? false);
      const color = curve.color ?? curveColors[displayRole(curve)];
      const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: .65 }));
      mesh.userData = { index, baseColor: color };
      this.curves.add(mesh);
    }
  }

  private select(index: number) {
    for (const mesh of this.curves.children as THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>[]) {
      mesh.material.color.setHex(mesh.userData.index === index ? 0x295d48 : mesh.userData.baseColor);
    }
  }

  setSource(visible: boolean, faded = false) {
    this.model.visible = visible;
    this.model.traverse(object => {
      if (object instanceof THREE.Mesh) {
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          material.transparent = faded;
          material.opacity = faded ? .12 : 1;
          material.depthWrite = !faded;
        }
      }
    });
  }

  private view() {
    const distance = this.extent * 2.4;
    const direction = new THREE.Vector3(1, -1.4, .9).normalize();
    this.camera.position.copy(this.center).addScaledVector(direction, distance);
    this.controls.target.copy(this.center);
    this.controls.update();
  }
}
