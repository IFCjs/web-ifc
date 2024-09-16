import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import * as THREE from "three";

let scene;
let camera;
let renderer;
let controls;

function Init3DView() {
    scene =  new THREE.Scene();
    renderer =  new THREE.WebGLRenderer({ antialias: true });
    let obj = document.getElementById("3dcontainer") as any;
    obj.appendChild(renderer.domElement);
    camera =  new THREE.PerspectiveCamera(45,obj.clientWidth / obj.clientHeight,0.1,1000);;
  renderer.setSize( obj.clientWidth - 20, obj.clientHeight - 20);
  window.addEventListener('resize', onWindowResize, false );

  controls = new OrbitControls(camera, renderer.domElement);


  scene.background = new THREE.Color(0x8cc7de);

  InitBasicScene();

  camera.position.z = 5;

  AnimationLoop();
}

export function ClearScene() {
  while(scene.children.length > 0) scene.remove(scene.children[0]); 
}

export function InitBasicScene()
{
  const directionalLight1 = new THREE.DirectionalLight(0xffeeff, 0.8);
  directionalLight1.position.set(1, 1, 1);
  scene.add(directionalLight1);

  const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight2.position.set(-1, 0.5, -1);
  scene.add(directionalLight2);

  const ambientLight = new THREE.AmbientLight(0xffffee, 0.25);
  scene.add(ambientLight);
}


function onWindowResize(){
    let obj = document.getElementById("3dcontainer") as any;
    camera.aspect = renderer.domElement.innerWidth / renderer.domElement.innerWidth;
    camera.updateProjectionMatrix();

    renderer.setSize( obj.clientWidth - 20, obj.clientHeight - 20);

}

function AnimationLoop() {
  requestAnimationFrame(AnimationLoop);
  controls.update();
  renderer.render(scene, camera);
}

export { scene, Init3DView };
