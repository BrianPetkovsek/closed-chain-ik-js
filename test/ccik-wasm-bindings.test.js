import { Solver, Joint, Link, Goal, DOF } from '../src/index.js';
import { loadCCIK } from '../lib/ccik-wasm.js';

jest.setTimeout( 30000 );

function buildJsSolution( target ) {
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

function buildWasmSolution( target, ccik ) {
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
	solver.setTarget( { x: target[ 0 ], y: target[ 1 ], z: target[ 2 ] } );
	solver.setTolerance( ccik.CCIK_TOLERANCE );
	solver.solve( 40 );

	const positions = solver.getPositions();
	const end = positions[ positions.length - 1 ];
	return [ end.x, end.y, end.z ];
}

function distance( a, b ) {
	const dx = a[ 0 ] - b[ 0 ];
	const dy = a[ 1 ] - b[ 1 ];
	const dz = a[ 2 ] - b[ 2 ];
	return Math.sqrt( dx * dx + dy * dy + dz * dz );
}

describe( 'ccik WASM bindings', () => {
	let ccik;
	beforeAll( async () => {
		ccik = await loadCCIK();
	} );

	it( 'matches JS solver output within tolerance', async () => {
		const target = [ 0.8, 0.6, 0 ];
		const jsPos = buildJsSolution( target );
		const wasmPos = buildWasmSolution( target, ccik );
		expect( distance( jsPos, wasmPos ) ).toBeLessThan( 1e-4 );
	} );
} );
