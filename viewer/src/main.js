import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { xmlToViewerResult } from './xml-model.js';
const $=id=>document.getElementById(id), V=(x,y,z)=>[x,y,z], canvas=document.createElement('canvas'),view=$('view'),nodeLabelCanvas=$('node-label-overlay'),nodeLabelContext=nodeLabelCanvas.getContext('2d');
view.appendChild(canvas);
const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:false});renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.15;
const world=new THREE.Scene();world.background=new THREE.Color(0x0b1018);const camera=new THREE.PerspectiveCamera(45,1,.01,100000);camera.up.set(0,0,1);const ortho=new THREE.OrthographicCamera(-50,50,50,-50,.01,100000);ortho.up.set(0,0,1);let activeCamera=camera;
const nodeRaycaster=new THREE.Raycaster(),nodePointer=new THREE.Vector2(),pickPoint=new THREE.Vector3();let selectedNodeId=null,selectedNodeRef=null,selectedElementRef=null,selectionMode='node',pointerStart=null,nodeById=new Map(),elementPickMeshes=[],sourceXmlDoc=null,sourceModelName='',modelHasChanges=false;
const controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.dampingFactor=.08;controls.screenSpacePanning=true;controls.target.set(0,0,0);
world.add(new THREE.HemisphereLight(0xe6f4ff,0x26354a,1.35));const keyLight=new THREE.DirectionalLight(0xffffff,2.1);keyLight.position.set(80,-100,140);world.add(keyLight);const fillLight=new THREE.DirectionalLight(0x72cfff,.85);fillLight.position.set(-100,90,-70);world.add(fillLight);
let modelGroup=new THREE.Group();world.add(modelGroup);let helperGroup=new THREE.Group();world.add(helperGroup);let orthographic=false,displayMode='both',showWaterSurface=true,showAxes=false,showNodes=true,showNodeIds=false,hiddenComponents=new Set(),data=null,playing=false,t=0,last=0,toastTimer,positions=new Map(),scene=null,renderables=new Map(),viewBounds=new THREE.Box3(),nodeInstances=new Map(),nodeMarkerMeshes=[],componentIndex=new Map(),segmentBatches=new Map(),segmentRefsByNode=new Map(),animatedNodeIds=new Set();
function colorFor(role){return role==='beam'?'#a3e635':role==='rope'?'#a78bfa':role==='buoy'?'#f59e0b':role==='joint'?'#dbeafe':'#38bdf8'}
function makeMaterial(color,roughness=.5,metalness=.08){return new THREE.MeshStandardMaterial({color,roughness,metalness})}
function shade(hex,factor){const c=hex.replace('#',''),n=parseInt(c.length===3?c.split('').map(x=>x+x).join(''):c,16);return '#'+[16,8,0].map(s=>Math.round(Math.max(0,Math.min(255,((n>>s)&255)*factor))).toString(16).padStart(2,'0')).join('')}
function makeLine(a,b,radius,color,kind='beam'){const va=new THREE.Vector3(...a),vb=new THREE.Vector3(...b),dir=new THREE.Vector3().subVectors(vb,va),length=dir.length();if(length<1e-8)return null;const geometry=kind==='box'?new THREE.BoxGeometry(radius*2,length,radius*2):new THREE.CylinderGeometry(radius,radius,length,kind==='rope'?8:16);const mesh=new THREE.Mesh(geometry,makeMaterial(color,kind==='rope'?.65:.42,kind==='rope'?.02:.12));mesh.position.copy(va).add(vb).multiplyScalar(.5);mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),dir.normalize());return mesh}
function addRenderable(key,obj){if(!renderables.has(key))renderables.set(key,[]);obj.userData.component_id=key;renderables.get(key).push(obj);modelGroup.add(obj)}
function componentForElement(e){return componentIndex.get(String(e.component_id??e.componentId??e.id))}
function elementNodeRefs(e){if(Array.isArray(e.nodes))return e.nodes;return['nodeA','nodeB','nodeC','nodeD'].map(key=>e[key]).filter(value=>value!==undefined&&value!==null)}
function isMembraneElement(e){const type=String(e.type||e.element_type||'').toLowerCase(),category=String(e.category||'').toLowerCase(),role=String(e.role||'').toLowerCase();return category.includes('membrane')||role.includes('membrane')||type.includes('membrane')||['triangle','tri','quad','surface'].includes(type)||(type==='polygon'&&!category.includes('beam')&&role!=='beam')}
function surfaceFaces(e,refs){
  const faces=e.faces||e.triangles;
  if(!Array.isArray(faces)||!faces.length)return[refs];
  const resolve=value=>{
    const match=refs.find(ref=>String(ref)===String(value));
    if(match!==undefined)return match;
    if(positions.has(value))return value;
    const index=Number(value);
    return Number.isInteger(index)&&index>=0&&index<refs.length?refs[index]:value;
  };
  if(!Array.isArray(faces[0]))return faces.length%3===0?Array.from({length:faces.length/3},(_,i)=>faces.slice(i*3,i*3+3).map(resolve)):[refs];
  return faces.map(face=>(Array.isArray(face)?face:face.nodes||face.nodeIds||['nodeA','nodeB','nodeC','nodeD'].map(key=>face[key]).filter(value=>value!==undefined&&value!==null)).map(resolve));
}
function surfaceTriangleNodeIds(e){
  const refs=elementNodeRefs(e),triangles=[];
  for(const face of surfaceFaces(e,refs)){
    if(face.length<3||face.some(id=>!positions.has(id)))continue;
    for(let i=1;i<face.length-1;i++)triangles.push(face[0],face[i],face[i+1]);
  }
  return triangles;
}
function drawElement(e){
  if(e.hidden)return;
  const refs=elementNodeRefs(e),pts=refs.map(id=>positions.get(id));
  if(pts.length<2||pts.some(point=>!point))return;
  const type=String(e.type||'').toLowerCase(),component=componentForElement(e),category=`${e.category||''} ${component?.category||''}`.toLowerCase(),role=String(e.role||'').toLowerCase(),structural=category.includes('beam')||role==='beam',truss=category.includes('truss')||role==='truss',color=e.color||component?.color||colorFor(e.role),sectionRadius=Number(e.section_radius_m)||Number(component?.section_radius_m)||(structural?.12:.018),key=e.component_id||e.componentId||e.id||('element-'+segmentBatches.size),pairs=[];
  for(let i=1;i<refs.length;i++)pairs.push([refs[i-1],refs[i]]);
  if(type==='polygon'&&refs.length>2)pairs.push([refs.at(-1),refs[0]]);
  const lineRadius=truss?Math.min(.006,Math.max(.002,sectionRadius*.25)):structural?sectionRadius:Math.max(sectionRadius,.008),batchKey=[key,structural?'beam':truss?'truss':'rope',color,lineRadius].join('|');
  if(!segmentBatches.has(batchKey))segmentBatches.set(batchKey,{key,structural,truss,color,radius:lineRadius,segments:[]});
  const batch=segmentBatches.get(batchKey);
  for(let segmentIndex=0;segmentIndex<pairs.length;segmentIndex++){const[a,b]=pairs[segmentIndex],segment={element:e,a,b,index:batch.segments.length,batch};batch.segments.push(segment);for(const id of new Set([a,b])){if(!segmentRefsByNode.has(id))segmentRefsByNode.set(id,[]);segmentRefsByNode.get(id).push(segment)}}
}
function setSegmentMatrix(segment){
  const a=positions.get(segment.a),b=positions.get(segment.b);if(!a||!b)return;
  const start=new THREE.Vector3(...a),end=new THREE.Vector3(...b),direction=end.clone().sub(start),length=direction.length();if(length<1e-9)return;
  const matrix=new THREE.Matrix4(),rotation=new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0,1,0),direction.normalize()),scale=new THREE.Vector3(segment.batch.radius,length,segment.batch.radius);
  matrix.compose(start.add(end).multiplyScalar(.5),rotation,scale);segment.batch.mesh.setMatrixAt(segment.index,matrix);
  if(segment.batch.centerline){const attribute=segment.batch.centerline.geometry.attributes.position;attribute.setXYZ(segment.index*2,...a);attribute.setXYZ(segment.index*2+1,...b);attribute.needsUpdate=true}
}
function finalizeSegmentBatches(){
  for(const batch of segmentBatches.values()){
    const geometry=new THREE.CylinderGeometry(1,1,1,batch.structural?12:6),material=makeMaterial(batch.color,batch.structural?.42:.65,batch.structural?.12:.02),mesh=new THREE.InstancedMesh(geometry,material,batch.segments.length);
    mesh.userData.structural=batch.structural;mesh.userData.sectionMesh=batch.structural||batch.truss;mesh.userData.instanceElements=batch.segments.map(segment=>segment.element);mesh.frustumCulled=false;batch.mesh=mesh;elementPickMeshes.push(mesh);
    batch.segments.forEach(segment=>setSegmentMatrix(segment));mesh.instanceMatrix.needsUpdate=true;addRenderable(batch.key,mesh);
    if(batch.structural||batch.truss){const values=new Float32Array(batch.segments.length*6),lineGeometry=new THREE.BufferGeometry();lineGeometry.setAttribute('position',new THREE.BufferAttribute(values,3));const centerline=new THREE.LineSegments(lineGeometry,new THREE.LineBasicMaterial({color:0xf8fafc}));centerline.userData.centerline=true;centerline.userData.structural=batch.structural;batch.centerline=centerline;batch.segments.forEach(segment=>{const a=positions.get(segment.a),b=positions.get(segment.b);if(a&&b){values.set(a,segment.index*6);values.set(b,segment.index*6+3)}});lineGeometry.attributes.position.needsUpdate=true;lineGeometry.computeBoundingSphere();addRenderable(batch.key,centerline)}
  }
  segmentBatches.clear();
}
function drawMembraneGroups(elements){
  const groups=new Map();
  for(const element of elements){if(element.hidden)continue;const key=element.component_id||element.componentId||element.id||'membranes',component=componentForElement(element),color=element.color||component?.color||'#10b981',groupKey=String(key)+'|'+color;if(!groups.has(groupKey))groups.set(groupKey,{key,color,elements:[]});groups.get(groupKey).elements.push(element)}
  for(const group of groups.values()){
    const nodeIds=[],triangleElements=[];
    for(const element of group.elements){const triangles=surfaceTriangleNodeIds(element);nodeIds.push(...triangles);for(let i=0;i<triangles.length;i+=3)triangleElements.push(element)}
    if(!nodeIds.length)continue;
    const vertices=[];for(const id of nodeIds)vertices.push(...positions.get(id));
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));geometry.computeVertexNormals();
    const material=makeMaterial(group.color,.6,.1);material.side=THREE.DoubleSide;material.transparent=true;material.opacity=.65;material.depthWrite=false;
    const mesh=new THREE.Mesh(geometry,material);mesh.userData.surfaceNodeIds=nodeIds;mesh.userData.triangleElements=triangleElements;mesh.userData.membrane=true;addRenderable(group.key,mesh);elementPickMeshes.push(mesh);
    const wireMaterial=new THREE.MeshBasicMaterial({color:group.color,wireframe:true,transparent:true,opacity:.4,depthWrite:false,side:THREE.DoubleSide});
    const wire=new THREE.Mesh(geometry,wireMaterial);wire.userData.surfaceNodeIds=nodeIds;wire.userData.membrane=true;addRenderable(group.key,wire);
  }
}
function updateMembraneGeometry(){const updated=new Set();for(const objects of renderables.values())for(const object of objects){const ids=object.userData.surfaceNodeIds,geometry=object.geometry;if(!ids||updated.has(geometry))continue;updated.add(geometry);const attribute=geometry.attributes.position;for(let i=0;i<ids.length;i++){const point=positions.get(ids[i]);if(point)attribute.setXYZ(i,...point)}attribute.needsUpdate=true;geometry.computeVertexNormals();geometry.computeBoundingSphere()}}
function nodeMarkerRadii(){
  const attached=new Map();
  for(const element of scene?.elements||[]){
    const component=componentForElement(element),structural=element.role==='beam'||element.category==='beams',membrane=isMembraneElement(element);
    const radiusValue=Number(element.section_radius_m)||Number(component?.section_radius_m),radius=Number.isFinite(radiusValue)&&radiusValue>0?radiusValue:(structural?.12:.018);
    for(const id of elementNodeRefs(element)){if(!attached.has(id))attached.set(id,[]);attached.get(id).push({radius,preferred:!membrane&&!structural})}
  }
  const radii=new Map();
  for(const node of scene?.nodes||[]){const entries=attached.get(node.id)||[],preferred=entries.filter(item=>item.preferred),values=(preferred.length?preferred:entries).map(item=>item.radius);radii.set(node.id,Math.max(.002,(values.length?Math.min(...values):.018)*1.1))}
  return radii;
}
function addNodeMarkers(){
  nodeInstances.clear();nodeMarkerMeshes.length=0;
  const groups=new Map(),radii=nodeMarkerRadii();
  for(const node of scene?.nodes||[]){
    const position=positions.get(node.id);if(!position||node.role==='rope')continue;
    const roller=node.role==='roller',groupKey=roller?'roller':node.role;
    if(!groups.has(groupKey))groups.set(groupKey,{roller,role:node.role,nodes:[]});
    groups.get(groupKey).nodes.push({node,position,radius:radii.get(node.id)||.02});
  }
  for(const [groupKey,group] of groups){
    const geometry=group.roller?new THREE.CylinderGeometry(1,1,2.2,12):new THREE.SphereGeometry(1,8,6);
    const color=group.roller?'#f59e0b':group.role==='winch'?'#fb7185':colorFor(group.role),mesh=new THREE.InstancedMesh(geometry,makeMaterial(color,.4,.1),group.nodes.length);
    mesh.userData.nodeMarker=true;nodeMarkerMeshes.push(mesh);mesh.userData.instanceNodeIds=group.nodes.map(item=>item.node.id);mesh.frustumCulled=false;
    const rotation=group.roller?new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI/2,0,0)):new THREE.Quaternion(),scale=new THREE.Vector3(),matrix=new THREE.Matrix4();
    group.nodes.forEach((item,index)=>{scale.set(item.radius,item.radius,item.radius);matrix.compose(new THREE.Vector3(...item.position),rotation,scale);mesh.setMatrixAt(index,matrix);nodeInstances.set(item.node.id,{mesh,index,radius:item.radius,roller:group.roller})});
    mesh.instanceMatrix.needsUpdate=true;addRenderable('node-markers:'+groupKey,mesh);
  }
}
function updateNodeMarker(id){
  const position=positions.get(id),instance=nodeInstances.get(id);if(!position||!instance)return;
  const rotation=instance.roller?new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI/2,0,0)):new THREE.Quaternion(),matrix=new THREE.Matrix4();
  matrix.compose(new THREE.Vector3(...position),rotation,new THREE.Vector3(instance.radius,instance.radius,instance.radius));instance.mesh.setMatrixAt(instance.index,matrix);
}
function buildHelpers(){
  while(helperGroup.children.length){
    const object=helperGroup.children[0];
    helperGroup.remove(object);
    object.traverse?.(child=>{child.geometry?.dispose?.();child.material?.dispose?.()});
  }
  const extent=scene?.extent||30,waterZ=scene?.waterline_z||0;
  if(showWaterSurface){
    const waterDepth=Math.max(10,extent*1.5);
    const volume=new THREE.Mesh(
      new THREE.BoxGeometry(extent*2,extent*2,waterDepth),
      new THREE.MeshBasicMaterial({color:0x087fae,transparent:true,opacity:.055,side:THREE.DoubleSide,depthWrite:false,toneMapped:false})
    );
    volume.position.set(0,0,waterZ-waterDepth/2-.035);
    helperGroup.add(volume);

    const surfaceGeometry=new THREE.PlaneGeometry(extent*2,extent*2);
    const surface=new THREE.Mesh(
      surfaceGeometry,
      new THREE.MeshBasicMaterial({color:0x168fc4,transparent:true,opacity:.27,side:THREE.DoubleSide,depthWrite:false,toneMapped:false})
    );
    surface.position.z=waterZ+.02;
    helperGroup.add(surface);

    const edge=new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-extent,-extent,waterZ+.035),
        new THREE.Vector3(extent,-extent,waterZ+.035),
        new THREE.Vector3(extent,extent,waterZ+.035),
        new THREE.Vector3(-extent,extent,waterZ+.035),
      ]),
      new THREE.LineBasicMaterial({color:0x65d8ff,transparent:true,opacity:.55,depthWrite:false,toneMapped:false})
    );
    helperGroup.add(edge);
  }
  if(showAxes){
    const axes=new THREE.AxesHelper(Math.max(4,extent*.18));
    axes.position.z=waterZ;
    helperGroup.add(axes);
  }
}
function buildScene(){for(const child of [...modelGroup.children]){modelGroup.remove(child);child.traverse?.(o=>{o.geometry?.dispose?.();if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material?.dispose?.()})}renderables.clear();segmentBatches.clear();segmentRefsByNode.clear();nodeInstances.clear();elementPickMeshes.length=0;componentIndex=new Map((scene?.components||[]).map(component=>[String(component.id),component]));if(!scene)return;const membraneElements=[];for(const e of scene.elements||[]){if(isMembraneElement(e))membraneElements.push(e);else drawElement(e)}finalizeSegmentBatches();drawMembraneGroups(membraneElements);for(const p of scene.primitives||[]){if(p.type==='cylinder'&&p.center){const [x,y,z]=p.center,g=new THREE.CylinderGeometry(p.radius,p.radius,p.height,24),m=new THREE.Mesh(g,makeMaterial(p.color||colorFor(p.role),.38,.16));m.rotation.x=Math.PI/2;m.position.set(x,y,z+p.height/2);addRenderable(p.component_id||p.id,m)}}addNodeMarkers();buildHelpers();syncVisibility();viewBounds.setFromObject(modelGroup)}
function syncVisibility(){for(const [key,objects] of renderables)for(const obj of objects){obj.visible=obj.userData.nodeMarker?showNodes:!hiddenComponents.has(key);if(obj.userData.centerline)obj.visible=obj.visible&&displayMode!=='sections';else if(obj.isMesh&&obj.userData.sectionMesh)obj.visible=obj.visible&&displayMode!=='centerlines'}}
function draw(){const w=view.clientWidth||innerWidth,h=view.clientHeight||innerHeight;if(!w||!h)return;renderer.setSize(w,h,false);activeCamera.aspect=w/h;activeCamera.updateProjectionMatrix();world.background.set(document.body.classList.contains('light')?0xe8edf3:0x0b1018);renderer.render(world,activeCamera);drawNodeLabels();updateSelectedNodeInfo()}
function updateSelectedNodeInfo(){
  if(selectedNodeRef){
    const position=positions.get(selectedNodeId);if(!position)return;
    const id=selectedNodeRef.node_id??String(selectedNodeRef.id).replace(/^n-/,'');$('node-info-title').textContent='Node '+id;
    $('node-coordinate-rows').hidden=false;$('element-info').hidden=true;$('node-current-role').textContent=selectedNodeRef.source_role||'none';
    for(let axis=0;axis<3;axis++)$('node-'+['x','y','z'][axis]).value=Number(position[axis]).toFixed(3);
    return;
  }
  if(!selectedElementRef)return;
  const element=selectedElementRef,component=componentForElement(element),nodeRefs=elementNodeRefs(element),category=String(element.category||component?.category||'').toLowerCase();
  const elementType=category==='beams'?'Beam':category==='trusses'?'Truss':category==='membranes'?'Membrane':element.element_type||element.type||element.role||'Unknown';
  const componentName=element.component_name||component?.label||component?.name||component?.id||'—',elementName=element.name||element.id||'—';
  const id=element.id??elementName;$('node-info-title').textContent='Element '+id;
  $('node-coordinate-rows').hidden=true;$('element-info').hidden=false;
  $('element-type').textContent=String(elementType);$('element-component').textContent=String(componentName);$('element-name').textContent=String(elementName);
  $('element-nodes').textContent=nodeRefs.map(ref=>{const node=nodeById.get(ref);return String(node?.node_id??String(ref).replace(/^n-/,''))}).join(', ')||'—';
}
function selectNodeAt(event,force=false){
  if((!showNodes&&!force)||!nodeMarkerMeshes.length)return;
  const rect=canvas.getBoundingClientRect();activeCamera.updateMatrixWorld();nodePointer.x=((event.clientX-rect.left)/rect.width)*2-1;nodePointer.y=-((event.clientY-rect.top)/rect.height)*2+1;
  nodeRaycaster.setFromCamera(nodePointer,activeCamera);
  const hit=nodeRaycaster.intersectObjects(nodeMarkerMeshes,false)[0];let id=hit?.instanceId===undefined?null:hit.object.userData.instanceNodeIds[hit.instanceId];
  if(id===null){let nearest=14*14,front=Infinity;for(const [nodeId] of nodeInstances){const position=positions.get(nodeId);if(!position)continue;pickPoint.set(...position).project(activeCamera);if(pickPoint.z < -1||pickPoint.z > 1)continue;const x=(pickPoint.x+1)*rect.width/2,y=(1-pickPoint.y)*rect.height/2,dx=x-(event.clientX-rect.left),dy=y-(event.clientY-rect.top),distance=dx*dx+dy*dy;if(distance<nearest||(distance<=nearest+1&&pickPoint.z<front)){nearest=distance;front=pickPoint.z;id=nodeId}}}
  if(id===null)return;selectedNodeId=id;selectedNodeRef=nodeById.get(id);selectedElementRef=null;$('node-info').classList.add('open');updateSelectedNodeInfo();drawNodeLabels();
}
function selectElementAt(event){
  if(!elementPickMeshes.length)return;
  const rect=canvas.getBoundingClientRect();activeCamera.updateMatrixWorld();nodePointer.x=((event.clientX-rect.left)/rect.width)*2-1;nodePointer.y=-((event.clientY-rect.top)/rect.height)*2+1;
  nodeRaycaster.setFromCamera(nodePointer,activeCamera);
  const hit=nodeRaycaster.intersectObjects(elementPickMeshes,false)[0];let element=hit?.object.userData.instanceElements?.[hit.instanceId]||hit?.object.userData.triangleElements?.[hit.faceIndex];
  if(!element){
    let nearest=12*12;const mouseX=event.clientX-rect.left,mouseY=event.clientY-rect.top;
    for(const candidate of scene?.elements||[]){if(candidate.hidden||isMembraneElement(candidate))continue;const componentKey=candidate.component_id||candidate.componentId||candidate.id;if(hiddenComponents.has(componentKey))continue;const refs=elementNodeRefs(candidate),pairs=[];for(let i=1;i<refs.length;i++)pairs.push([refs[i-1],refs[i]]);if(String(candidate.type||'').toLowerCase()==='polygon'&&refs.length>2)pairs.push([refs.at(-1),refs[0]]);
      for(const[a,b]of pairs){const pa=positions.get(a),pb=positions.get(b);if(!pa||!pb)continue;const va=pickPoint.set(...pa).project(activeCamera);if(va.z < -1||va.z > 1)continue;const ax=(va.x+1)*rect.width/2,ay=(1-va.y)*rect.height/2;const vb=new THREE.Vector3(...pb).project(activeCamera);if(vb.z < -1||vb.z > 1)continue;const bx=(vb.x+1)*rect.width/2,by=(1-vb.y)*rect.height/2,dx=bx-ax,dy=by-ay,length=dx*dx+dy*dy,t=length?Math.max(0,Math.min(1,((mouseX-ax)*dx+(mouseY-ay)*dy)/length)):0,ex=mouseX-(ax+t*dx),ey=mouseY-(ay+t*dy),distance=ex*ex+ey*ey;if(distance<nearest){nearest=distance;element=candidate}}
    }
  }
  if(!element)return;selectedElementRef=element;selectedNodeRef=null;selectedNodeId=null;$('node-info').classList.add('open');updateSelectedNodeInfo();drawNodeLabels();
}
function selectAt(event){if(event.button===2){openNodeContextMenu(event);return}$('node-context-menu').hidden=true;if(selectionMode==='node')selectNodeAt(event);else selectElementAt(event)}
function openNodeContextMenu(event){
  event.preventDefault();selectedNodeId=null;selectedNodeRef=null;selectedElementRef=null;
  selectNodeAt(event,true);
  const menu=$('node-context-menu');if(!selectedNodeRef){menu.hidden=true;return}
  if(!sourceXmlDoc){menu.hidden=true;toast('Load an XML or .amodel source model to assign solver roles.');return}
  const label=String(selectedNodeRef.node_id??selectedNodeRef.id).replace(/^n-/,'');$('node-context-title').textContent='Node '+label;
  menu.hidden=false;const bounds=menu.getBoundingClientRect();menu.style.left=Math.max(8,Math.min(event.clientX,innerWidth-bounds.width-8))+'px';menu.style.top=Math.max(8,Math.min(event.clientY,innerHeight-bounds.height-8))+'px';document.addEventListener('click',()=>{menu.hidden=true},{once:true});
}
function rebuildNodeMarkers(){
  for(const mesh of nodeMarkerMeshes){modelGroup.remove(mesh);mesh.geometry.dispose();mesh.material.dispose()}
  for(const key of [...renderables.keys()])if(String(key).startsWith('node-markers:'))renderables.delete(key);
  addNodeMarkers();syncVisibility();
}
function setSelectedNodeRole(role){
  if(!sourceXmlDoc||!selectedNodeRef)return;
  const sourceId=String(selectedNodeRef.node_id),currentRole=selectedNodeRef.source_role||'',protectedRoles=new Set(['upper_joint','lower_joint','buoy_top']);
  if(currentRole===role){toast(`Node ${sourceId} already has the ${role} role.`);return}
  if(['roller','winch'].includes(currentRole)){toast(`Node ${sourceId} already has the ${currentRole} role. Add another role on a different node.`);return}
  if(protectedRoles.has(currentRole)&&currentRole!==role){toast(`Cannot replace the required ${currentRole} role; the solver needs that node.`);return}
  const xmlNodes=[...sourceXmlDoc.querySelectorAll('node')],target=xmlNodes.find(node=>node.getAttribute('id')===sourceId);
  if(!target){toast('Could not find this node in the source model.');return}
  target.setAttribute('role',role);selectedNodeRef.source_role=role;selectedNodeRef.role=role;
  modelHasChanges=true;$('save-model').disabled=false;
  let message=`Added ${role} role to node ${sourceId}. Save the model to download the changes.`;
  if(role==='roller'&&!([...sourceXmlDoc.querySelectorAll('element[role="roller_body"]')].some(element=>['StartNode_ID','startNodeId','EndNode_ID','endNodeId'].some(attr=>element.getAttribute(attr)===sourceId))))message+=' A solver-ready roller also needs a roller_body beam and two connected truss legs.';
  if(role==='winch'&&!([...sourceXmlDoc.querySelectorAll('element[role="main_rope"]')].some(element=>['StartNode_ID','startNodeId','EndNode_ID','endNodeId'].some(attr=>element.getAttribute(attr)===sourceId))))message+=' Ensure a main_rope line connects to this node.';
  $('node-context-menu').hidden=true;toast(message);
  rebuildNodeMarkers();
  updateSelectedNodeInfo();draw();
}
function saveModifiedModel(){
  if(!sourceXmlDoc||!modelHasChanges)return;
  const xml=new XMLSerializer().serializeToString(sourceXmlDoc),blob=new Blob([xml],{type:'application/xml;charset=utf-8'}),url=URL.createObjectURL(blob),link=document.createElement('a');
  link.href=url;link.download=(sourceModelName.replace(/\.(xml|amodel)$/i,'')||'winchsim_model')+'_updated.xml';document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function drawNodeLabels(){const rect=view.getBoundingClientRect(),ratio=Math.min(devicePixelRatio||1,2),width=Math.max(1,Math.round(rect.width*ratio)),height=Math.max(1,Math.round(rect.height*ratio));if(nodeLabelCanvas.width!==width||nodeLabelCanvas.height!==height){nodeLabelCanvas.width=width;nodeLabelCanvas.height=height}const ctx=nodeLabelContext;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,rect.width,rect.height);if((!showNodeIds&&!selectedNodeRef)||!scene)return;activeCamera.updateMatrixWorld();ctx.font='11px system-ui, sans-serif';ctx.textAlign='left';ctx.textBaseline='middle';ctx.lineWidth=3;ctx.strokeStyle='rgba(5,12,20,.9)';ctx.fillStyle=document.body.classList.contains('light')?'#172334':'#f8fafc';if(showNodeIds)for(const node of scene.nodes||[]){const position=positions.get(node.id);if(!position)continue;const point=new THREE.Vector3(...position).project(activeCamera);if(point.z < -1||point.z > 1||point.x < -1||point.x > 1||point.y < -1||point.y > 1)continue;const id=node.node_id??String(node.id).replace(/^n-/,'');const label=String(id);const x=(point.x+1)*rect.width/2+5,y=(1-point.y)*rect.height/2;ctx.strokeText(label,x,y);ctx.fillText(label,x,y)}if(selectedNodeRef){const position=positions.get(selectedNodeId);if(position){const point=pickPoint.set(...position).project(activeCamera);if(point.z>=-1&&point.z<=1){ctx.beginPath();ctx.arc((point.x+1)*rect.width/2,(1-point.y)*rect.height/2,8,0,Math.PI*2);ctx.strokeStyle='#fbbf24';ctx.lineWidth=2;ctx.stroke()}}}}
function toast(msg){$('toast').textContent=msg;$('toast').classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('show'),4500)}function validate(d){if(!d||!Array.isArray(d.results)||!d.results.length)throw Error('This file contains no results array.');for(let i=0;i<d.results.length;i++){const row=d.results[i];if(!row||typeof row!=='object')throw Error('Result row '+(i+1)+' is invalid.');if(i&&Number(row.time_s)<Number(d.results[i-1].time_s))throw Error('Result times must be ascending.')}if(d.scene&&(!Array.isArray(d.scene.nodes)||!Array.isArray(d.scene.elements)))throw Error('The scene description is invalid.');return d}
function inferScene(d){if(d.scene)return d.scene;const g=d.geometry||{},first=d.results[0],side=Number(g.beam_side_m)||10,c=g.beam_corner_xy_m||[[-side/2,-side/2],[side/2,-side/2],[side/2,side/2],[-side/2,side/2]],nodes=c.map((p,i)=>({id:'beam-'+i,position:V(p[0],p[1],Number(first.beam_z_m)||0),role:'beam'}));nodes.push({id:'upper',position:V(0,0,Number(first.joint_z_m)||0),role:'joint'});const w=g.fixed_winch_xyz_m||V(0,0,20),r=g.roller_xyz_m,elements=[{type:'polygon',nodes:c.map((_,i)=>'beam-'+i),role:'beam'}];nodes.push({id:'winch',position:w,role:'anchor'});if(r){nodes.push({id:'roller',position:r,role:'anchor'});elements.push({type:'segment',nodes:['winch','roller'],role:'rope'},{type:'segment',nodes:['roller','upper'],role:'rope'})}else elements.push({type:'segment',nodes:['winch','upper'],role:'rope'});c.forEach((_,i)=>elements.push({type:'segment',nodes:['upper','beam-'+i],role:'rope'}));if(Number.isFinite(Number(first.lower_joint_z_m))){nodes.push({id:'lower',position:V(0,0,Number(first.lower_joint_z_m)),role:'joint'});c.forEach((_,i)=>elements.push({type:'segment',nodes:['lower','beam-'+i],role:'rope'}))}const primitives=[];if(Number.isFinite(Number(first.buoy_bottom_z_m)))primitives.push({id:'buoy',type:'cylinder',center:V(0,0,Number(first.buoy_bottom_z_m)),radius:Number(g.buoy_diameter_m||.35)/2,height:Number(g.buoy_height_m||.5),role:'buoy'});return{nodes,elements,primitives,waterline_z:Number(g.waterline_z_m)||0,animation:{node_z_fields:{beam_z_m:c.map((_,i)=>'beam-'+i),joint_z_m:['upper'],lower_joint_z_m:['lower']},primitive_z_fields:{buoy_bottom_z_m:['buoy']}}}}
function initializeScene(d){scene=inferScene(d);nodeById=new Map((scene.nodes||[]).map(node=>[node.id,node]));selectedNodeId=null;selectedNodeRef=null;selectedElementRef=null;$('node-info').classList.remove('open');positions=new Map((scene.nodes||[]).map(n=>[n.id,n.position.map(Number)]));animatedNodeIds=new Set([...Object.values(scene.animation?.node_z_fields||{}).flat(),...Object.keys(scene.animation?.node_position_fields||{})]);fitScene()}
function fitScene(){const all=[...positions.values(),...(scene?.primitives||[]).flatMap(p=>p.center?[p.center]:[])].map(p=>new THREE.Vector3(...p));if(!all.length)return;viewBounds.setFromPoints(all);const size=viewBounds.getSize(new THREE.Vector3()),center=viewBounds.getCenter(new THREE.Vector3()),span=Math.max(12,size.length());scene.extent=Math.max(25,span*1.5);const direction=new THREE.Vector3(1,-1.2,.8).normalize(),distance=span*1.7;camera.position.copy(center).add(direction.clone().multiplyScalar(distance));camera.near=Math.max(.01,span/10000);camera.far=span*100;camera.lookAt(center);camera.updateProjectionMatrix();ortho.position.copy(camera.position);ortho.lookAt(center);ortho.left=-span;ortho.right=span;ortho.top=span;ortho.bottom=-span;ortho.updateProjectionMatrix();controls.target.copy(center);controls.update();buildScene();viewBounds.setFromObject(modelGroup);draw()}
function applyFrame(row){if(scene){for(const [field,ids] of Object.entries(scene.animation?.node_z_fields||{})){if(!Number.isFinite(Number(row[field])))continue;for(const id of ids){const p=positions.get(id);if(p)p[2]=Number(row[field])}}for(const [id,fields] of Object.entries(scene.animation?.node_position_fields||{})){const p=positions.get(id);if(!p)continue;for(const [axis,field] of Object.entries(fields)){const idx=axis==='x'?0:axis==='y'?1:2,value=Number(row[field]);if(Number.isFinite(value))p[idx]=value}}for(const [field,ids] of Object.entries(scene.animation?.primitive_z_fields||{})){if(!Number.isFinite(Number(row[field])))continue;for(const id of ids){const primitive=(scene.primitives||[]).find(p=>p.id===id);if(primitive?.center)primitive.center[2]=Number(row[field])}}for(const [id,fields] of Object.entries(scene.animation?.primitive_position_fields||{})){const primitive=(scene.primitives||[]).find(p=>p.id===id);if(!primitive?.center)continue;for(const [axis,field] of Object.entries(fields)){const idx=axis==='x'?0:axis==='y'?1:2,value=Number(row[field]);if(Number.isFinite(value))primitive.center[idx]=value}}for(const id of animatedNodeIds)updateNodeMarker(id);for(const mesh of nodeMarkerMeshes)mesh.instanceMatrix.needsUpdate=true;updateMembraneGeometry();const changedSegments=new Set();for(const id of animatedNodeIds)for(const segment of segmentRefsByNode.get(id)||[])changedSegments.add(segment);const changedBatches=new Set();for(const segment of changedSegments){setSegmentMatrix(segment);changedBatches.add(segment.batch)}for(const batch of changedBatches){batch.mesh.instanceMatrix.needsUpdate=true;if(batch.centerline)batch.centerline.geometry.attributes.position.needsUpdate=true}for(const p of scene.primitives||[]){const obj=renderables.get(p.component_id||p.id)?.find(o=>o.userData.element_id===p.id)||renderables.get(p.component_id||p.id)?.[0];if(obj&&p.center)obj.position.set(p.center[0],p.center[1],p.center[2]+(p.height||0)/2)}}time.textContent=Number(row.time_s).toFixed(2)+' s';$('scrub').value=row.time_s;draw()}
function renderTime(tt){if(!data)return;const rows=data.results;let lo=0,hi=rows.length-1;while(lo<hi){const mid=(lo+hi)>>1;if(Number(rows[mid].time_s)<tt)lo=mid+1;else hi=mid}const i=Math.max(0,lo-1),a=rows[i],b=rows[Math.min(i+1,rows.length-1)],f=Number(a.time_s)===Number(b.time_s)?0:(tt-Number(a.time_s))/(Number(b.time_s)-Number(a.time_s)),r={...a};for(const key of Object.keys(a)){if(typeof a[key]==='number'&&typeof b[key]==='number')r[key]=a[key]+(b[key]-a[key])*f}applyFrame(r);t=tt}
function useResults(value,name){data=validate(value);hiddenComponents.clear();initializeScene(data);renderComponents();renderStatistics(name);$("statistics").classList.add("open");const preview=data.model_status==='model_input_preview',playback=data.results.length>1;document.body.classList.toggle('model-preview',preview);$('state').textContent=preview?'Model preview':'Results loaded';$('state').classList.add('ready');$('filename').textContent=name;const first=Number(data.results[0].time_s),end=Number(data.results.at(-1).time_s);$('scrub').min=first;$('scrub').max=end;$('scrub').step='any';$('start').textContent=first+' s';$('end').textContent=end+' s';$('scrub').disabled=!playback;$('play').disabled=$('reset').disabled=!playback;playing=false;$('play').textContent='Play';renderTime(first)}
canvas.style.touchAction='none';canvas.style.cursor='grab';canvas.addEventListener('pointerdown',event=>{pointerStart={x:event.clientX,y:event.clientY};canvas.style.cursor='grabbing'});canvas.addEventListener('pointerup',event=>{canvas.style.cursor='grab';if(pointerStart&&Math.hypot(event.clientX-pointerStart.x,event.clientY-pointerStart.y)<4)selectAt(event);pointerStart=null});canvas.addEventListener('pointercancel',()=>{pointerStart=null;canvas.style.cursor='grab'});canvas.addEventListener('pointerleave',()=>canvas.style.cursor='grab');$('load').addEventListener('click',()=>$('file').click());const loadBox=$('load-box');for(const type of ['dragenter','dragover'])loadBox.addEventListener(type,e=>{e.preventDefault();e.stopPropagation();loadBox.classList.add('drag-over')});for(const type of ['dragleave','dragend','drop'])loadBox.addEventListener(type,e=>{e.preventDefault();e.stopPropagation();loadBox.classList.remove('drag-over')});loadBox.addEventListener('drop',e=>{const f=e.dataTransfer?.files?.[0];if(f)loadFile(f)});function parseBinResults(buf,name){if(buf.byteLength<10)throw Error('Invalid WinchSim binary: file is shorter than its header');const dv=new DataView(buf);const magic=String.fromCharCode(dv.getUint8(0),dv.getUint8(1),dv.getUint8(2),dv.getUint8(3));if(magic!=='WSIM')throw Error('Invalid WinchSim binary: expected WSIM magic, got '+magic);const version=dv.getUint16(4,true),headerLen=dv.getUint32(6,true);if(version!==1)throw Error('Unsupported WinchSim binary version: '+version);if(headerLen>buf.byteLength-10)throw Error('Invalid WinchSim binary: truncated metadata header');const meta=JSON.parse(new TextDecoder().decode(new Uint8Array(buf,10,headerLen)));const shape=meta.shape,fields=meta.fields;if(!Array.isArray(shape)||shape.length!==2||!Number.isSafeInteger(shape[0])||shape[0]<1||!Number.isSafeInteger(shape[1])||shape[1]<1||!Array.isArray(fields)||fields.length!==shape[1]||fields.some(f=>typeof f!=='string'))throw Error('Invalid WinchSim binary: invalid shape or fields metadata');const [numRows,numCols]=shape,dataOffset=10+headerLen,expectedBytes=numRows*numCols*4;if(!Number.isSafeInteger(expectedBytes)||dataOffset+expectedBytes!==buf.byteLength)throw Error('Invalid WinchSim binary: payload size does not match its metadata');const rows=[];for(let i=0;i<numRows;i++){const row={};for(let j=0;j<numCols;j++)row[fields[j]]=dv.getFloat32(dataOffset+(i*numCols+j)*4,true);rows.push(row)}meta.results=rows;meta.results_count=numRows;return meta}
async function loadFile(f){if(!f)return;try{let parsed,xmlDoc=null;if(f.name.endsWith('.bin')){const buf=await f.arrayBuffer();parsed=parseBinResults(buf,f.name)}else{const txt=await f.text();if(f.name.endsWith('.xml')||f.name.endsWith('.amodel')||txt.trim().startsWith('<')){parsed=xmlToViewerResult(txt,f.name);xmlDoc=new DOMParser().parseFromString(txt,'application/xml')}else{parsed=JSON.parse(txt)}}sourceXmlDoc=xmlDoc;sourceModelName=xmlDoc?f.name:'';modelHasChanges=false;$('save-model').disabled=true;$('node-role-status').textContent='';useResults(parsed,f.name)}catch(err){toast('Unable to load results: '+err.message)}} $('file').addEventListener('change',e=>{const f=e.target.files?.[0];if(f)loadFile(f);e.target.value='' });$('scrub').addEventListener('input',e=>{playing=false;$('play').textContent='▶ Play';renderTime(Number(e.target.value))});$('play').addEventListener('click',()=>{playing=!playing;$('play').textContent=playing?'Ⅱ Pause':'▶ Play'});$('reset').addEventListener('click',()=>{playing=false;$('play').textContent='▶ Play';if(data)renderTime(Number(data.results[0].time_s))});$("camera").addEventListener('click',()=>{if(scene)fitScene();else draw()});window.addEventListener('resize',draw);const resizeObserver=new ResizeObserver(draw);resizeObserver.observe(view);function animate(now){requestAnimationFrame(animate);if(playing&&data){t+=Math.min((now-last)/1000,.05)*2;const end=Number(data.results.at(-1).time_s);if(t>=end){t=end;playing=false;$("play").textContent='▶ Play'}renderTime(t)}last=now;controls.update();renderer.render(world,activeCamera);drawNodeLabels();updateSelectedNodeInfo()}requestAnimationFrame(animate);draw();$('filename').textContent='No file loaded';$('state').textContent='Select results file';
try{if(localStorage.getItem('winchsim-theme')==='light')document.body.classList.add('light')}catch{}if(innerWidth<=700)document.body.classList.add('sidebar-collapsed');
function setCamera(kind){const center=viewBounds.isEmpty()?new THREE.Vector3():viewBounds.getCenter(new THREE.Vector3()),span=Math.max(12,viewBounds.getSize(new THREE.Vector3()).length());let direction;if(kind==='xp')direction=new THREE.Vector3(1,0,0);else if(kind==='yp')direction=new THREE.Vector3(0,1,0);else if(kind==='zp')direction=new THREE.Vector3(0,0,1);else direction=new THREE.Vector3(1,-1.2,.8).normalize();activeCamera.position.copy(center).add(direction.multiplyScalar(span*1.8));activeCamera.up.set(0,0,1);activeCamera.lookAt(center);controls.target.copy(center);controls.update();draw()}
function componentRecords(){return scene?.components?.length?scene.components:[...(scene?.elements||[]).map((e,i)=>({id:e.component_id||e.id||`element-${i+1}`,label:e.component_name||e.name||e.id||`Element ${i+1}`,category:e.category||(e.role==='beam'?'beams':e.role==='membrane'?'membranes':'trusses')})),...(scene?.primitives||[]).map((p,i)=>({id:p.component_id||p.id||`primitive-${i+1}`,label:p.name||p.id||`Primitive ${i+1}`,category:p.category||p.role||'other'}))]}
function renderComponents(){const host=$('components');if(!host)return;host.replaceChildren();const groups=new Map();for(const item of componentRecords()){if(!groups.has(item.category))groups.set(item.category,[]);groups.get(item.category).push(item)}for(const cat of ['beams','trusses','membranes'])if(!groups.has(cat))groups.set(cat,[]);for(const [category,items] of groups){const section=document.createElement('div');section.className='component-category';const head=document.createElement('div');head.className='category-head';const all=document.createElement('input');all.type='checkbox';all.checked=items.every(item=>!hiddenComponents.has(item.id));all.addEventListener('change',()=>{for(const item of items)all.checked?hiddenComponents.delete(item.id):hiddenComponents.add(item.id);renderComponents()});const name=document.createElement('span');name.className='category-name';name.textContent=`${category} (${items.length})`;const expand=document.createElement('button');expand.className='expand-list';expand.textContent='?';const list=document.createElement('div');list.className='component-list';expand.addEventListener('click',()=>{list.classList.toggle('closed');expand.textContent=list.classList.contains('closed')?'▸':'▾'});head.append(all,name,expand);section.append(head,list);for(const item of items){const label=document.createElement('label');label.className='component-item';const checkbox=document.createElement('input');checkbox.type='checkbox';checkbox.checked=!hiddenComponents.has(item.id);checkbox.addEventListener('change',()=>{checkbox.checked?hiddenComponents.delete(item.id):hiddenComponents.add(item.id);renderComponents()});const text=document.createElement('span');text.textContent=item.label||item.id;label.append(checkbox,text);list.append(label)}host.append(section)}draw()}
function renderStatistics(filename){const nodes=scene?.nodes||[],elements=scene?.elements||[],primitives=scene?.primitives||[],ids=new Set(nodes.map(n=>n.id)),invalid=elements.reduce((n,e)=>n+(e.nodes||[]).filter(id=>!ids.has(id)).length,0),all=[...nodes.map(n=>n.position),...primitives.flatMap(p=>p.center?[p.center]:[])].filter(p=>Array.isArray(p)&&p.length===3),bounds=[0,1,2].map(i=>all.length?[Math.min(...all.map(p=>Number(p[i]))),Math.max(...all.map(p=>Number(p[i])))]:[0,0]),categories={};for(const item of componentRecords())categories[item.category]=(categories[item.category]||0)+1;const rows=[['Filename',filename],['Nodes',nodes.length],['Elements',elements.length+primitives.length],['Beams',categories.beams||0],['Trusses',categories.trusses||0],['Membranes',categories.membranes||0],['Invalid references',invalid],['Bounds X (m)',`${bounds[0][0].toFixed(2)} to ${bounds[0][1].toFixed(2)}`],['Bounds Y (m)',`${bounds[1][0].toFixed(2)} to ${bounds[1][1].toFixed(2)}`],['Bounds Z (m)',`${bounds[2][0].toFixed(2)} to ${bounds[2][1].toFixed(2)}`]],box=$('statistics');box.replaceChildren();for(const [label,value] of rows){const row=document.createElement('div');row.className='stat-row';const a=document.createElement('span'),b=document.createElement('span');a.textContent=label;b.textContent=value;row.append(a,b);box.append(row)}}
$("display-mode").addEventListener('change',e=>{displayMode=e.target.value;syncVisibility();draw()});$('node-toggle').addEventListener('change',e=>{showNodes=e.target.checked;syncVisibility();draw()});$('node-id-toggle').addEventListener('change',e=>{showNodeIds=e.target.checked;drawNodeLabels()});$("water-toggle").addEventListener('change',e=>{showWaterSurface=e.target.checked;buildHelpers();draw()});$("axes-toggle").addEventListener('change',e=>{showAxes=e.target.checked;buildHelpers();draw()});$("projection").addEventListener('click',()=>{orthographic=!orthographic;if(orthographic){ortho.position.copy(camera.position);ortho.quaternion.copy(camera.quaternion);ortho.up.copy(camera.up);activeCamera=ortho;controls.object=ortho}else{camera.position.copy(ortho.position);camera.quaternion.copy(ortho.quaternion);activeCamera=camera;controls.object=camera}$("projection").textContent=orthographic?'Orthographic':'Perspective';controls.update();draw()});$('theme').addEventListener('click',()=>{const light=document.body.classList.toggle('light');try{localStorage.setItem('winchsim-theme',light?'light':'dark')}catch{}draw()});$('sidebar-toggle').addEventListener('click',()=>{const hidden=document.body.classList.toggle('sidebar-collapsed');$('sidebar-toggle').setAttribute('aria-label',hidden?'Show model information':'Hide model information')});document.querySelectorAll('[data-view]').forEach(button=>button.addEventListener('click',()=>setCamera(button.dataset.view)));$('stats-toggle').addEventListener('click',()=>$('statistics').classList.toggle('open'));$('node-info-close').addEventListener('click',()=>{$('node-info').classList.remove('open');selectedNodeId=null;selectedNodeRef=null;selectedElementRef=null});$('set-node-roller').addEventListener('click',()=>setSelectedNodeRole('roller'));$('set-node-winch').addEventListener('click',()=>setSelectedNodeRole('winch'));$('save-model').addEventListener('click',saveModifiedModel);$('selection-mode').addEventListener('change',event=>{selectionMode=event.target.value;selectedNodeId=null;selectedNodeRef=null;selectedElementRef=null;$('node-info').classList.remove('open');drawNodeLabels()});
