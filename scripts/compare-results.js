import { performance } from 'perf_hooks';
import { Solver, Joint, Link, Goal, DOF } from '../src/index.js';
import { loadCCIK } from '../lib/ccik-wasm.js';

const ERROR_THRESHOLD = Number( process.env.CCIK_MAX_ERROR || '1e-4' );
const SAMPLE_COUNT = Number( process.env.CCIK_SAMPLES || '8' );
const LCG_MOD = 2147483647;
const LCG_MULT = 16807;
const LCG_NORM = 2147483646;

function seededRandomGenerator( seed = 12345 ) {
	let value = seed % LCG_MOD;
	return () => {
		value = ( value * LCG_MULT ) % LCG_MOD;
		return ( value - 1 ) / LCG_NORM;
	};
}

const rand = seededRandomGenerator();

function distance( a, b ) {
	const dx = a[ 0 ] - b[ 0 ];
	const dy = a[ 1 ] - b[ 1 ];
	const dz = a[ 2 ] - b[ 2 ];
	return Math.sqrt( dx * dx + dy * dy + dz * dz );
}

function solveWithJS( target ) {
	const root = new Link();

	const joint1 = new Joint();
	joint1.setDoF( DOF.EZ );
	joint1.setPosition( 0, 0, 0 );

	const link1 = new Link();
	link1.setPosition( 0, 0, 1 );

	const joint2 = new Joint();
	joint2.setDoF( DOF.EZ );
	joint2.setPosition( 0, 0, 1 );

	const link2 = new Link();
	link2.setPosition( 0, 0, 1 );

	const goal = new Goal();
	link2.getWorldPosition( goal.position );
	link2.getWorldQuaternion( goal.quaternion );
	goal.makeClosure( link2 );

	root.addChild( joint1 );
	joint1.addChild( link1 );
	link1.addChild( joint2 );
	joint2.addChild( link2 );

	goal.setPosition( ...target );

	const solver = new Solver( root );
	solver.maxIterations = 50;
	solver.solve();

	const pos = new Float32Array( 3 );
	link2.getWorldPosition( pos );
	return [ pos[ 0 ], pos[ 1 ], pos[ 2 ] ];
}

function solveWithWasm( target, ccik ) {
	const jointSpec = {
		axis: { x: 0, y: 0, z: 1 },
		length: 1,
		mode: ccik.JointMode.Rotation,
		minLimit: -Math.PI,
		maxLimit: Math.PI,
		value: 0,
	};

	const chain = new ccik.Chain( [ jointSpec, { ...jointSpec } ], { x: 0, y: 0, z: 0 } );
	const solver = new ccik.IKSolver( chain );
	solver.setTolerance( ccik.CCIK_TOLERANCE );
	solver.setTarget( { x: target[ 0 ], y: target[ 1 ], z: target[ 2 ] } );
	solver.solve( 50 );

	const positions = solver.getPositions();
	const end = positions[ positions.length - 1 ];
	return [ end.x, end.y, end.z ];
}

async function main() {
	const ccik = await loadCCIK();
	let maxError = 0;
	let totalJs = 0;
	let totalWasm = 0;

	for ( let i = 0; i < SAMPLE_COUNT; i ++ ) {
		const target = [
			( rand() * 2 - 1 ) * 1.5,
			( rand() * 2 - 1 ) * 1.5,
			0,
		];

		const jsStart = performance.now();
		const jsResult = solveWithJS( target );
		totalJs += performance.now() - jsStart;

		const wasmStart = performance.now();
		const wasmResult = solveWithWasm( target, ccik );
		totalWasm += performance.now() - wasmStart;

		const err = distance( jsResult, wasmResult );
		maxError = Math.max( maxError, err );
	}

	console.log( `Compared ${ SAMPLE_COUNT } samples` );
	console.log( `JS solve avg: ${( totalJs / SAMPLE_COUNT ).toFixed( 3 )} ms` );
	console.log( `WASM solve avg: ${( totalWasm / SAMPLE_COUNT ).toFixed( 3 )} ms` );
	console.log( `Max absolute error: ${ maxError } (threshold ${ ERROR_THRESHOLD })` );

	if ( maxError > ERROR_THRESHOLD ) {
		console.error( 'WASM / JS parity failed' );
		process.exit( 1 );
	}
}

main().catch( err => {
	console.error( err );
	process.exit( 1 );
} );
