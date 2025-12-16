import { mat4, quat } from 'gl-matrix';
import { Solver } from '../../src/core/Solver.js';
import { Joint, DOF } from '../../src/core/Joint.js';
import { Link } from '../../src/core/Link.js';
import { Goal } from '../../src/core/Goal.js';
import { axisToDof } from '../../scripts/axis-to-dof.js';

const POSITION_TOLERANCE = 5e-4;
const VECTOR_TOLERANCE = 5e-7;

function expectVecClose( actual, expected, tol ) {

	expect( Math.abs( actual[ 0 ] - expected[ 0 ] ) ).toBeLessThan( tol );
	expect( Math.abs( actual[ 1 ] - expected[ 1 ] ) ).toBeLessThan( tol );
	expect( Math.abs( actual[ 2 ] - expected[ 2 ] ) ).toBeLessThan( tol );

}

function buildChain( axes, lengths, goalDoF = [ DOF.X, DOF.Y, DOF.Z ] ) {

	const root = new Link();
	let parent = root;

	for ( let i = 0; i < axes.length; i ++ ) {

		const joint = new Joint();
		const axis = axes[ i ];
		joint.setDoF( axisToDof( axis, DOF ) );
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
	goal.setGoalDoF( ...goalDoF );

	root.addChild( goal );

	return { roots: [ root, goal ], end: parent, goal };

}

function solveToTarget( solver, end, goal, target, settings ) {

	goal.setWorldPosition( ...target );

	Object.assign( solver, settings );
	solver.maxIterations = settings.maxIterations ?? 80;
	solver.solve();

	const pos = new Float32Array( 3 );
	end.getWorldPosition( pos );
	return [ pos[ 0 ], pos[ 1 ], pos[ 2 ] ];

}

describe( 'Solver integration', () => {

	it( 'maintains zero-error closure across diverse targets already satisfied', () => {

		const axes = [ [ 0, 0, 1 ], [ 0, 0, 1 ] ];
		const lengths = [ 0.9, 1.1 ];
		const { roots, end, goal } = buildChain( axes, lengths );
		const solver = new Solver( roots );

		const basePos = new Float32Array( 3 );
		end.updateMatrixWorld( true );
		end.getWorldPosition( basePos );

		const settings = {
			translationConvergeThreshold: 5e-6,
			rotationConvergeThreshold: 1e-7,
		};

		const offsets = [
			[ 0, 0, 0 ],
			[ 1e-4, - 2e-4, 0 ],
			[ - 3e-4, 1e-4, 0 ],
		];

		offsets.forEach( off => {

			const target = [ basePos[ 0 ] + off[ 0 ], basePos[ 1 ] + off[ 1 ], basePos[ 2 ] + off[ 2 ] ];
			const result = solveToTarget( solver, end, goal, target, settings );
			const dx = result[ 0 ] - target[ 0 ];
			const dy = result[ 1 ] - target[ 1 ];
			const dz = result[ 2 ] - target[ 2 ];

			expect( Math.hypot( dx, dy, dz ) ).toBeLessThan( POSITION_TOLERANCE );

		} );

	} );

	it( 'matches forward transforms for mixed-axis rotations with tight tolerance', () => {

		const axes = [ [ 0, 1, 0 ], [ 0, 0, 1 ], [ 1, 0, 0 ] ];
		const lengths = [ 0.6, 0.8, 0.5 ];
		const { roots, end } = buildChain( axes, lengths, [ DOF.X, DOF.Y, DOF.Z, DOF.EX, DOF.EY, DOF.EZ ] );

		const joints = [];
		roots[ 0 ].traverse( c => {

			if ( c.isJoint ) joints.push( c );

		} );

		const values = [ 0.35, - 0.4, 0.5 ];
		joints[ 0 ].setDoFValue( DOF.EY, values[ 0 ] );
		joints[ 1 ].setDoFValue( DOF.EZ, values[ 1 ] );
		joints[ 2 ].setDoFValue( DOF.EX, values[ 2 ] );

		roots[ 0 ].updateMatrixWorld( true );

		const worldPos = new Float32Array( 3 );
		end.getWorldPosition( worldPos );

		const m1 = new Float32Array( 16 );
		const m2 = new Float32Array( 16 );
		const m3 = new Float32Array( 16 );
		const rot1 = quat.create();
		const rot2 = quat.create();
		const rot3 = quat.create();

		quat.setAxisAngle( rot1, [ 0, 1, 0 ], values[ 0 ] );
		mat4.fromRotationTranslation( m1, rot1, [ 0, 0, lengths[ 0 ] ] );

		quat.setAxisAngle( rot2, [ 0, 0, 1 ], values[ 1 ] );
		mat4.fromRotationTranslation( m2, rot2, [ 0, 0, lengths[ 1 ] ] );
		mat4.multiply( m2, m1, m2 );

		quat.setAxisAngle( rot3, [ 1, 0, 0 ], values[ 2 ] );
		mat4.fromRotationTranslation( m3, rot3, [ 0, 0, lengths[ 2 ] ] );
		mat4.multiply( m3, m2, m3 );

		const expected = new Float32Array( 3 );
		mat4.getTranslation( expected, m3 );

		expectVecClose( worldPos, expected, VECTOR_TOLERANCE );

	} );

} );
