import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

export interface Curve { id: string; segments: number[][][]; role?: string; color?:number; diameter_m?: number; length_m: number; closed?: boolean }
export interface Junction { id: string; position: number[]; ground: boolean }
export interface Deformation { id: string; positions_m: number[][]; displacements_m: number[][] }
export interface ToolBody {
  name:string; origin:number[][]; type:'box'|'sphere'|'cylinder'|'mesh';
  size_m?:number[]; radius_m?:number; length_m?:number; vertices?:number[][]; faces?:number[][];
}
export interface ToolModel {configured:boolean; visual_bodies:ToolBody[]; tip_radius_m?:number; calibration_declared?:boolean}

export interface RobotRig {
  kinematic_root:string; kinematic_tip:string;
  kinematic_chain:{parent:string;child:string;origin:number[][];axis:number[]|null;index:number|null}[];
  visual_nodes:{name:string;link:string;origin:number[][]}[];
}

export interface CellArmView {
  id:string; rig:RobotRig; url:string; world_from_base:number[][];
  flange_to_tcp:number[][]; initial_positions:number[]; tool_model:ToolModel;
}

export class Viewer {
  renderer: THREE.WebGLRenderer;
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(35, 1, .001, 10);
  controls: OrbitControls;
  model = new THREE.Group();
  curves = new THREE.Group();
  junctions = new THREE.Group();
  deformations = new THREE.Group();
  platform = new THREE.Group();
  tool = new THREE.Group();
  toolGeometry = new THREE.Group();
  deposited = new THREE.Group();
  activePath = new THREE.Group();
  contactRegion = new THREE.Group();
  cell = new THREE.Group();
  grid: THREE.GridHelper;
  private loader = new GLTFLoader();
  private source: THREE.Object3D | null = null;
  private loadGeneration = 0;
  private cellGeneration = 0;
  private cellReference: THREE.Object3D | null = null;
  private cellArms = new Map<string,{view:CellArmView;source:THREE.Object3D;tool:THREE.Group;toolGeometry:THREE.Group;color:number}>();
  private onSelect: (curve: Curve | null) => void;
  private disposed = false;
  private limit = 10000;
  private curveData: Curve[] = [];
  private hiddenRoles = new Set<string>();
  private highlighted = new Set<string>();
  private extent = .15;
  private center = new THREE.Vector3(0, 0, .045);
  compareSource = false;

  constructor(container: HTMLElement, onSelect: (curve: Curve | null) => void) {
    this.onSelect = onSelect;
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
    this.scene.add(this.grid, this.model, this.curves, this.junctions, this.deformations, this.platform, this.tool, this.toolGeometry, this.deposited, this.activePath, this.contactRegion, this.cell);
    const observer = new ResizeObserver(() => {
      const { width, height } = container.getBoundingClientRect();
      this.renderer.setSize(width, height);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
    });
    observer.observe(container);
    let down = { x: 0, y: 0 };
    this.renderer.domElement.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY }; });
    this.renderer.domElement.addEventListener('pointerup', e => {
      if (Math.hypot(e.clientX - down.x, e.clientY - down.y) > 4) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      const ray = new THREE.Raycaster();
      const paneWidth = rect.width / (this.compareSource ? 2 : 1);
      const x = e.clientX - rect.left;
      if(this.compareSource && x < paneWidth) { this.select(-1); return; }
      ray.setFromCamera(new THREE.Vector2((x % paneWidth) / paneWidth * 2 - 1, -(e.clientY - rect.top) / rect.height * 2 + 1), this.camera);
      const hits = ray.intersectObjects(this.curves.children.filter(c => c.visible));
      const index = hits.length ? hits[0].object.userData.index as number : -1;
      this.select(index);
    });
    this.view('perspective');
    const draw = () => {
      if (this.disposed) return;
      requestAnimationFrame(draw);
      this.controls.update();
      const width = container.clientWidth, height = container.clientHeight;
      if (width && height) {
        this.camera.aspect = width / height / (this.compareSource ? 2 : 1);
        this.camera.updateProjectionMatrix();
      }
      if (this.compareSource) {
        const sourceVisible = this.model.visible, curvesVisible = this.curves.visible, jointsVisible = this.junctions.visible;
        this.renderer.setScissorTest(true);
        this.renderer.setViewport(0, 0, width / 2, height);
        this.renderer.setScissor(0, 0, width / 2, height);
        this.model.visible = true; this.curves.visible = false; this.junctions.visible = false;
        this.renderer.render(this.scene, this.camera);
        this.renderer.setViewport(width / 2, 0, width / 2, height);
        this.renderer.setScissor(width / 2, 0, width / 2, height);
        this.model.visible = false; this.curves.visible = curvesVisible; this.junctions.visible = jointsVisible;
        this.renderer.render(this.scene, this.camera);
        this.model.visible = sourceVisible;
        this.renderer.setScissorTest(false);
      } else {
        this.renderer.setViewport(0, 0, width, height);
        this.renderer.render(this.scene, this.camera);
      }
    };
    draw();
  }

  async load(url: string, onProgress: (n: number) => void) {
    const generation = ++this.loadGeneration;
    const gltf = await this.loader.loadAsync(url, event => onProgress(event.total ? event.loaded / event.total : 0));
    if (generation !== this.loadGeneration) { this.clear(gltf.scene); return; }
    this.clear(this.model);
    this.source = gltf.scene;
    this.model.add(this.source);
    const box = new THREE.Box3().setFromObject(this.source);
    box.getCenter(this.center);
    const size = box.getSize(new THREE.Vector3());
    this.extent = Math.max(size.x, size.y, size.z);
    this.controls.minDistance = this.extent * .08;
    this.controls.maxDistance = this.extent * 10;
    this.grid.scale.setScalar(this.extent / .15);
    this.grid.position.set(0,0,-.0003);
    this.view('perspective');
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

  setToolModel(tool?:ToolModel, target=this.toolGeometry) {
    this.clear(target);
    if (!tool?.configured) return;
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

  setRobotPose(q:number[], rig:RobotRig, flangeToTcp:number[][], context?:{
    source:THREE.Object3D;tool:THREE.Group;toolGeometry:THREE.Group;world_from_base:number[][];color:number;
  }) {
    const matrix = (values:number[][]) => new THREE.Matrix4().fromArray(values.flat()).transpose();
    const frames = new Map<string,THREE.Matrix4>([[rig.kinematic_root,context?matrix(context.world_from_base):new THREE.Matrix4()]]);
    const source=context?.source??this.source, tool=context?.tool??this.tool;
    const toolGeometry=context?.toolGeometry??this.toolGeometry, color=context?.color??0x327aa0;
    for (const joint of rig.kinematic_chain) {
      const frame = frames.get(joint.parent)!.clone().multiply(matrix(joint.origin));
      if (joint.index !== null && joint.axis) {
        frame.multiply(new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...joint.axis as [number,number,number]).normalize(),q[joint.index]));
      }
      frames.set(joint.child,frame);
    }
    for (const visual of rig.visual_nodes) {
      const node = source?.getObjectByName(visual.name);
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
    this.view('perspective');
  }

  setCellArmPose(identity:string,q:number[]) {
    const row=this.cellArms.get(identity);if(!row)throw new Error(`Missing cell arm ${identity}`);
    return this.setRobotPose(q,row.view.rig,row.view.flange_to_tcp,{...row,world_from_base:row.view.world_from_base});
  }

  setCellArmBase(identity:string,worldFromBase:number[][]) {
    const row=this.cellArms.get(identity);if(!row)throw new Error(`Missing cell arm ${identity}`);
    row.view={...row.view,world_from_base:worldFromBase};
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
    this.view('perspective');
  }

  highlightCellLinks(identity:string,links:string[]) {
    const arm=this.cellArms.get(identity);
    if(arm)this.highlightRobotLinks(links,arm.view.rig,arm.source);
  }

  highlightRobotLinks(links:string[], rig:RobotRig, source=this.source) {
    for (const visual of rig.visual_nodes) {
      source?.getObjectByName(visual.name)?.traverse(object => {
        if (!(object instanceof THREE.Mesh)) return;
        if (!object.userData.collisionMaterials) {
          object.material = Array.isArray(object.material) ? object.material.map(material => material.clone()) : object.material.clone();
          object.userData.collisionMaterials = true;
        }
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          if (!(material instanceof THREE.MeshStandardMaterial)) continue;
          material.userData.baseEmissive ??= material.emissive.getHex();
          material.emissive.setHex(links.includes(visual.link) ? 0x9f1423 : material.userData.baseEmissive);
        }
      });
    }
  }

  setPlatform(center?: number[], top = 0, table = 0, width = .24, depth = .24) {
    this.clear(this.platform);
    this.clear(this.tool);
    this.setToolModel();
    if (!center) return;
    const height = Math.max(.006, top-table);
    const base = new THREE.Mesh(new THREE.BoxGeometry(width,depth,height),
      new THREE.MeshStandardMaterial({color:0xa6b5a3, roughness:.9, transparent:true, opacity:.65}));
    base.position.set(center[0],center[1],top-height/2);
    this.platform.add(base);
  }

  clearSequence() { this.clear(this.deposited); this.clear(this.activePath); this.clear(this.contactRegion); }

  setContactRegion(center?:number[], radius?:number) {
    this.clear(this.contactRegion);
    if (!center || !radius) return;
    const marker=new THREE.Mesh(new THREE.SphereGeometry(radius,16,12),new THREE.MeshBasicMaterial({color:0x9272b8,wireframe:true,transparent:true,opacity:.45}));
    marker.position.set(...center as [number,number,number]);
    this.contactRegion.add(marker);
  }

  setDepositedPaths(paths:number[][][]) {
    this.clear(this.deposited);
    const segments=paths.reduce((count,points)=>count+Math.max(0,points.length-1),0);
    if(!segments)return;
    const vertices=new Float32Array(segments*6);
    let offset=0;
    for (const points of paths) {
      for(let i=1;i<points.length;i++){
        vertices.set(points[i-1],offset);
        vertices.set(points[i],offset+3);
        offset+=6;
      }
    }
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(vertices,3));
    this.deposited.add(new THREE.LineSegments(geometry,new THREE.LineBasicMaterial({color:0xb77a3f})));
  }

  setActivePath(points:number[][], drawing:boolean) {
    this.clear(this.activePath);
    if (points.length < 2) return;
    const vertices = points.map(p=>new THREE.Vector3(...p as [number,number,number]));
    this.activePath.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(vertices),new THREE.LineBasicMaterial({color:drawing ? 0x295d48 : 0x367ca5})));
  }

  setCurves(curves: Curve[]) {
    this.clear(this.curves);
    this.clear(this.junctions);
    this.clear(this.deformations);
    this.hiddenRoles.clear();
    this.highlighted.clear();
    this.limit = curves.length;
    this.curveData = curves;
    for (const [index, curve] of curves.entries()) {
      const path = new THREE.CurvePath<THREE.Vector3>();
      for (const segment of curve.segments) {
        const points = segment.map(p => new THREE.Vector3(...p as [number, number, number]));
        path.add(new THREE.CubicBezierCurve3(points[0], points[1], points[2], points[3]));
      }
      const geometry = new THREE.TubeGeometry(path, Math.min(700, Math.max(24, curve.segments.length * 10)), (curve.diameter_m ?? .001) / 2, 6, curve.closed ?? false);
      const color = curve.color ?? ({connector:0x537b8a,brace:0x8c695d,base:0x52745a,support:0x52745a} as Record<string,number>)[curve.role ?? ''] ?? 0xb77a3f;
      const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color, roughness: .65 }));
      mesh.userData = { index, baseColor: color };
      this.curves.add(mesh);
    }
    this.setLimit(this.limit);
    this.select(-1);
  }

  selectById(id:string) { this.select(this.curveData.findIndex(curve => curve.id === id)); }

  select(index: number) {
    for (const mesh of this.curves.children as THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>[]) {
      mesh.material.color.setHex(mesh.userData.index === index ? 0x295d48 : this.highlighted.has(this.curveData[mesh.userData.index].id) ? 0xbd4450 : mesh.userData.baseColor);
    }
    this.onSelect(this.curveData[index] ?? null);
  }

  setLimit(count: number) { this.limit = count; this.curves.children.forEach((c, i) => c.visible = i < count && !this.hiddenRoles.has(this.curveData[i].role ?? 'artistic')); }
  setRoleVisible(role: string, visible: boolean) { if (visible) this.hiddenRoles.delete(role); else this.hiddenRoles.add(role); this.setLimit(this.limit); }
  highlight(ids: string[]) { this.highlighted = new Set(ids); this.select(-1); }
  setJunctions(nodes: Junction[], visible: boolean) {
    this.clear(this.junctions);
    const mesh = new THREE.InstancedMesh(new THREE.SphereGeometry(.001, 8, 6), new THREE.MeshStandardMaterial(), nodes.length);
    const matrix = new THREE.Matrix4();
    nodes.forEach((node,index) => {
      matrix.makeTranslation(...node.position as [number,number,number]);
      mesh.setMatrixAt(index,matrix);
      mesh.setColorAt(index,new THREE.Color(node.ground ? 0x2e7854 : 0x43596a));
    });
    this.junctions.add(mesh);
    this.junctions.visible = visible;
  }
  setDeformation(edges: Deformation[], scale: number) {
    this.clear(this.deformations);
    if (scale <= 0) return;
    for (const edge of edges) {
      const points = edge.positions_m.map((p,i) => new THREE.Vector3(...p.map((v,j) => v + edge.displacements_m[i][j]*scale) as [number,number,number]));
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0x6b61a8}));
      this.deformations.add(line);
    }
  }
  setSource(visible: boolean, faded = false) {
    this.model.visible = visible;
    this.model.traverse(object => {
      if (object instanceof THREE.Mesh) {
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of materials) {
          material.transparent = faded && !this.compareSource;
          material.opacity = faded && !this.compareSource ? .12 : 1;
          material.depthWrite = !faded || this.compareSource;
        }
      }
    });
  }

  view(name: string) {
    const distance = this.extent * 2.4;
    const direction = name === 'top' ? new THREE.Vector3(0, -.0001, 1) : name === 'front' ? new THREE.Vector3(0, -1, .03) : name === 'side' ? new THREE.Vector3(1, 0, .03) : new THREE.Vector3(1, -1.4, .9).normalize();
    this.camera.position.copy(this.center).addScaledVector(direction, distance);
    this.controls.target.copy(this.center);
    this.controls.update();
  }
}
