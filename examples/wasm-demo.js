import * as THREE from 'https://unpkg.com/three@0.178.0/build/three.module.js';
import loadCCIK from '../lib/ccik-wasm.js';

const container = document.getElementById( 'scene' );
const renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setSize( container.clientWidth, container.clientHeight );
container.appendChild( renderer.domElement );

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera( 50, container.clientWidth / container.clientHeight, 0.1, 10 );
camera.position.set( 3, 3, 3 );
camera.lookAt( 0, 0, 0 );

scene.add( new THREE.AmbientLight( 0xffffff, 1 ) );

const targetMesh = new THREE.Mesh( new THREE.SphereGeometry( 0.05 ), new THREE.MeshStandardMaterial( { color: 0xff3366 } ) );
scene.add( targetMesh );

const lineMaterial = new THREE.LineBasicMaterial( { color: 0x00ccff, linewidth: 3 } );
const lineGeometry = new THREE.BufferGeometry();
const line = new THREE.Line( lineGeometry, lineMaterial );
scene.add( line );

const ccik = await loadCCIK();
const jointSpec = {
	axis: { x: 0, y: 0, z: 1 },
	length: 1,
	mode: ccik.JointMode.Rotation,
	minLimit: -Math.PI,
	maxLimit: Math.PI,
	value: 0,
};

const solver = new ccik.IKSolver( new ccik.Chain( [ jointSpec, { ...jointSpec } ], { x: 0, y: 0, z: 0 } ) );
solver.setTolerance( ccik.CCIK_TOLERANCE );

function updateSolver( target ) {
	solver.setTarget( { x: target.x, y: target.y, z: target.z } );
	solver.solve( 24 );

	const positions = solver.getPositions();
	const verts = new Float32Array( positions.length * 3 );
	positions.forEach( ( p, i ) => {
		verts[ i * 3 + 0 ] = p.x;
		verts[ i * 3 + 1 ] = p.y;
		verts[ i * 3 + 2 ] = p.z;
	} );
	lineGeometry.setAttribute( 'position', new THREE.BufferAttribute( verts, 3 ) );
	lineGeometry.computeBoundingSphere();
}

const target = new THREE.Vector3( 0.8, 0.6, 0 );
targetMesh.position.copy( target );
updateSolver( target );

function animate() {
	requestAnimationFrame( animate );
	renderer.render( scene, camera );
}

renderer.domElement.addEventListener( 'pointermove', e => {
	const rect = renderer.domElement.getBoundingClientRect();
	const nx = ( ( e.clientX - rect.left ) / rect.width ) * 2 - 1;
	const ny = ( ( e.clientY - rect.top ) / rect.height ) * 2 - 1;
	target.set( nx, ny, 0 );
	targetMesh.position.copy( target );
	updateSolver( target );
} );

animate();
