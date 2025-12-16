import { performance } from 'perf_hooks';
import { Solver, Joint, Link, Goal } from '../src/index.js';
import { axisToDof } from './axis-to-dof.js';
import { loadCCIK } from '../lib/ccik-wasm.js';

const ERROR_THRESHOLD = Number( process.env.CCIK_MAX_ERROR || '5e-5' );
const SAMPLE_COUNT = Number( process.env.CCIK_SAMPLES || '12' );
const LCG_MOD = 2147483647;
const LCG_MULT = 16807;
const LCG_NORM = 2147483646;
const ORIGIN = [ 0, 0, 0 ];

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

const PROFILES = [
	{
		name: 'planar-zz',
		axes: [ [ 0, 0, 1 ], [ 0, 0, 1 ] ],
		lengths: [ 1, 1 ],
		target: () => {
			const r = 1.6 * rand();
			const theta = rand() * Math.PI * 2;
			return [ Math.cos( theta ) * r, Math.sin( theta ) * r, 2 ];
		},
	},
	{
		name: 'spatial-yz',
		axes: [ [ 0, 1, 0 ], [ 0, 0, 1 ], [ 1, 0, 0 ] ],
		lengths: [ 0.8, 0.7, 0.5 ],
		target: () => {
			const r = 1.2 + rand() * 0.4;
			const theta = rand() * Math.PI * 2;
			const z = 0.6 + rand() * 0.6;
			return [ Math.cos( theta ) * r, Math.sin( theta ) * r, z ];
		},
	},
];

function solveWithJS( profile, target ) {
	const { axes, lengths } = profile;
	const root = new Link();
	let parent = root;

	for ( let i = 0; i < axes.length; i ++ ) {
		const joint = new Joint();
		joint.setDoF( axisToDof( axes[ i ] ) );
		joint.setPosition( 0, 0, lengths[ i ] );

		const link = new Link();

		parent.addChild( joint );
		joint.addChild( link );
		parent = link;
	}

	const goal = new Goal();
	parent.getWorldPosition( goal.position );
	parent.getWorldQuaternion( goal.quaternion );
	goal.makeClosure( parent );
	root.addChild( goal );
	goal.setWorldPosition( ...target );

	const solver = new Solver( [ root, goal ] );
	solver.maxIterations = 60;
	solver.translationConvergeThreshold = 5e-5;
	solver.rotationConvergeThreshold = 1e-6;
	solver.solve();

	const pos = new Float32Array( 3 );
	parent.getWorldPosition( pos );
	return [ pos[ 0 ], pos[ 1 ], pos[ 2 ] ];
}

function solveWithWasm( profile, target, ccik ) {
	const { axes, lengths } = profile;

	const specs = new ccik.JointSpecList();
	for ( let i = 0; i < axes.length; i ++ ) {
		const axis = axes[ i ];
		specs.push_back( {
			axis: { x: axis[ 0 ], y: axis[ 1 ], z: axis[ 2 ] },
			length: lengths[ i ],
			mode: ccik.JointMode.Rotation,
			minLimit: -Math.PI,
			maxLimit: Math.PI,
			value: 0,
			name: `joint-${ i + 1 }`,
		} );
	}

	const chain = new ccik.Chain();
	chain.setBasePosition( { x: 0, y: 0, z: 0 } );
	chain.setJoints( specs );

	const solver = new ccik.IKSolver( chain );
	solver.setTolerance( ccik.CCIK_TOLERANCE );
	solver.setTarget( { x: target[ 0 ], y: target[ 1 ], z: target[ 2 ] } );
	solver.solve( 60 );

	const positions = solver.getPositions();
	const endIndex = positions.size() - 1;
	if ( endIndex < 0 ) {
		console.warn( 'ccik returned no positions for profile', profile.name );
		return ORIGIN;
	}

	const end = positions.get( endIndex );
	return [ end.x, end.y, end.z ];
}

async function main() {
	const ccik = await loadCCIK();
	let maxError = 0;
	let totalJs = 0;
	let totalWasm = 0;

	for ( const profile of PROFILES ) {
		for ( let i = 0; i < SAMPLE_COUNT; i ++ ) {
			const target = profile.target();

			const jsStart = performance.now();
			const jsResult = solveWithJS( profile, target );
			totalJs += performance.now() - jsStart;

			const wasmStart = performance.now();
			const wasmResult = solveWithWasm( profile, target, ccik );
			totalWasm += performance.now() - wasmStart;

			const err = distance( jsResult, wasmResult );
			maxError = Math.max( maxError, err );
		}
	}

	const totalSamples = SAMPLE_COUNT * PROFILES.length;

	console.log( `Compared ${ totalSamples } samples across ${ PROFILES.length } profiles` );
	console.log( `JS solve avg: ${( totalJs / totalSamples ).toFixed( 3 )} ms` );
	console.log( `WASM solve avg: ${( totalWasm / totalSamples ).toFixed( 3 )} ms` );
	console.log( `Max absolute error: ${ maxError } (threshold ${ ERROR_THRESHOLD })` );

	if ( maxError > ERROR_THRESHOLD ) {
		console.error( `WASM / JS parity failed: error ${ maxError } exceeds threshold ${ ERROR_THRESHOLD }` );
		process.exit( 1 );
	}
}

main().catch( err => {
	console.error( err );
	process.exit( 1 );
} );
