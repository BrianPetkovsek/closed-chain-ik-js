import { Joint, DOF } from '../../../src/core/Joint.js';
import { accumulateTargetError } from '../../../src/core/utils/solver.js';

function createSolverStub( overrides = {} ) {

	return {
		translationConvergeThreshold: 1e-6,
		rotationConvergeThreshold: 1e-6,
		lockedJointDoFCount: new Map(),
		translationErrorClamp: 1,
		rotationErrorClamp: 1,
		lockedJointDoF: new Map(),
		translationFactor: 1,
		rotationFactor: 1,
		...overrides,
	};

}

describe( 'accumulateTargetError', () => {

	it( 'uses quaternion distance when all three rotation axes are enabled.', () => {

		const joint = new Joint();
		joint.setDoF( DOF.EX, DOF.EY, DOF.EZ );
		joint.setDoFValues( 0, 0, 0 );
		joint.setTargetValues( Math.PI * 2, 0, 0 );

		const solver = createSolverStub( { rotationConvergeThreshold: 1e-5 } );
		const result = { isConverged: false, rowCount: 0, totalError: 0 };

		accumulateTargetError( solver, joint, 0, null, result );

		expect( result.isConverged ).toBeTruthy();
		expect( Math.abs( result.totalError ) ).toBeLessThan( 1e-6 );

	} );

	it( 'writes rotation rows with DOF-relative euler indices and solver scaling.', () => {

		const joint = new Joint();
		joint.setDoF( DOF.EY );
		joint.setDoFValues( 0 );
		joint.setTargetValues( 1 );

		const solver = createSolverStub( {
			rotationErrorClamp: 0.5,
			rotationFactor: 2,
		} );
		const errorVector = [[ 0 ]];
		const result = { isConverged: false, rowCount: 0, totalError: 0 };

		accumulateTargetError( solver, joint, 0, errorVector, result );

		expect( result.rowCount ).toEqual( 1 );
		expect( errorVector[ 0 ][ 0 ] ).toBeCloseTo( 1, 10 );

	} );

} );
