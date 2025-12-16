import { mat4, quat, vec3 } from 'gl-matrix';
import { Link } from '../../src/core/Link.js';
import { Joint, DOF } from '../../src/core/Joint.js';
import { findRoots } from '../../src/core/utils/findRoots.js';
import { quaternionDistance } from '../../src/core/utils/quaternion.js';
import { RAD2DEG } from '../../src/core/utils/constants.js';

describe( 'Joint', () => {

	describe( 'clearDoF', () => {

		it( 'should clear all DoF', () => {

			const joint = new Joint();
			joint.setDoF( DOF.X, DOF.Z, DOF.EX );

			expect( joint.dof ).toEqual( [ DOF.X, DOF.Z, DOF.EX ] );
			expect( joint.dofFlags ).toEqual( new Uint8Array( [ 1, 0, 1, 1, 0, 0 ] ) );
			expect( joint.translationDoFCount ).toEqual( 2 );
			expect( joint.rotationDoFCount ).toEqual( 1 );

			joint.clearDoF();

			expect( joint.dof ).toEqual( [] );
			expect( joint.dofFlags ).toEqual( new Uint8Array( [ 0, 0, 0, 0, 0, 0 ] ) );
			expect( joint.translationDoFCount ).toEqual( 0 );
			expect( joint.rotationDoFCount ).toEqual( 0 );

		} );

	} );

	describe( 'setDoF', () => {

		it( 'should set the degrees of freedom values.', () => {

			const joint = new Joint();
			joint.setDoF( DOF.X, DOF.Z, DOF.EX );

			expect( joint.dof ).toEqual( [ DOF.X, DOF.Z, DOF.EX ] );
			expect( joint.dofFlags ).toEqual( new Uint8Array( [ 1, 0, 1, 1, 0, 0 ] ) );
			expect( joint.translationDoFCount ).toEqual( 2 );
			expect( joint.rotationDoFCount ).toEqual( 1 );

		} );

		it( 'should reset DoF values.', () => {

			const joint = new Joint();
			joint.setDoF( DOF.X, DOF.Z, DOF.EX );

			joint.setMinLimits( 1, 2, 3 );
			joint.setMaxLimits( 2, 4, 6 );
			joint.setTargetValues( 2, 4, 6 );
			joint.setRestPoseValues( 2, 4, 6 );
			joint.setDoFValues( 1, 2, 3 );

			expect( joint.minDoFLimit ).toEqual( new Float32Array( [ 1, - Infinity, 2, 3, - Infinity, - Infinity ] ) );
			expect( joint.maxDoFLimit ).toEqual( new Float32Array( [ 2, Infinity, 4, 6, Infinity, Infinity ] ) );
			expect( joint.dofTarget ).toEqual( new Float32Array( [ 2, 0, 4, 6, 0, 0 ] ) );
			expect( joint.dofRestPose ).toEqual( new Float32Array( [ 2, 0, 4, 6, 0, 0 ] ) );
			expect( joint.dofValues ).toEqual( new Float32Array( [ 1, 0, 2, 3, 0, 0 ] ) );

			joint.setDoF( DOF.X, DOF.Z, DOF.EX );

			expect( joint.minDoFLimit ).toEqual( new Float32Array( 6 ).fill( - Infinity ) );
			expect( joint.maxDoFLimit ).toEqual( new Float32Array( 6 ).fill( Infinity ) );
			expect( joint.dofTarget ).toEqual( new Float32Array( [ 0, 0, 0, 0, 0, 0 ] ) );
			expect( joint.dofRestPose ).toEqual( new Float32Array( [ 0, 0, 0, 0, 0, 0 ] ) );
			expect( joint.dofValues ).toEqual( new Float32Array( [ 0, 0, 0, 0, 0, 0 ] ) );

		} );

	} );

	describe( 'setMatrixDoFNeedsUpdate', () => {

		it( 'should mark the joint as needing a dof matrix and world matrix update.', () => {

			const joint = new Joint();
			const child = new Link();
			joint.addChild( child );

			joint.updateMatrixWorld( true );

			expect( joint.matrixWorldNeedsUpdate ).toBeFalsy();
			expect( joint.matrixDoFNeedsUpdate ).toBeFalsy();
			expect( joint.matrixNeedsUpdate ).toBeFalsy();

			expect( child.matrixWorldNeedsUpdate ).toBeFalsy();

			joint.setMatrixDoFNeedsUpdate();

			expect( joint.matrixWorldNeedsUpdate ).toBeTruthy();
			expect( joint.matrixDoFNeedsUpdate ).toBeTruthy();
			expect( joint.matrixNeedsUpdate ).toBeFalsy();

			expect( child.matrixWorldNeedsUpdate ).toBeTruthy();

			joint.updateDoFMatrix();

			expect( joint.matrixWorldNeedsUpdate ).toBeTruthy();
			expect( joint.matrixDoFNeedsUpdate ).toBeFalsy();
			expect( joint.matrixNeedsUpdate ).toBeFalsy();

			expect( child.matrixWorldNeedsUpdate ).toBeTruthy();

		} );

		it( 'should get the dof matrix updated when calling updateMatrixWorld', () => {

			const joint = new Joint();

			joint.updateMatrixWorld();
			joint.setMatrixDoFNeedsUpdate();

			expect( joint.matrixDoFNeedsUpdate ).toBeTruthy();

			joint.updateMatrixWorld();

			expect( joint.matrixDoFNeedsUpdate ).toBeFalsy();

		} );

	} );

	describe( 'addChild', () => {

		it( 'should only allow adding links.', () => {

			const joint = new Joint();
			const joint2 = new Joint();

			let caught;
			caught = false;
			try {

				joint.addChild( joint2 );

			} catch ( e ) {

				caught = true;

			}

			expect( caught ).toBeTruthy();

		} );

		it( 'should only allow adding one child.', () => {

			const joint = new Joint();
			const child = new Link();
			const child2 = new Link();
			joint.addChild( child );

			let caught;
			caught = false;
			try {

				joint.addChild( child2 );

			} catch ( e ) {

				caught = true;

			}

			expect( caught ).toBeTruthy();

		} );

		it( 'should throw if the joint is already a closure.', () => {

			const joint = new Joint();
			const child = new Link();
			const child2 = new Link();
			joint.makeClosure( child );

			let caught;
			caught = false;
			try {

				joint.addChild( child2 );

			} catch ( e ) {

				caught = true;

			}

			expect( caught ).toBeTruthy();

		} );

		it( 'should set the child field.', () => {

			const joint = new Joint();
			const child = new Link();

			expect( joint.child ).toEqual( null );
			joint.addChild( child );
			expect( joint.child ).toEqual( child );

		} );

	} );

	describe( 'makeClosure', () => {

		it( 'should set child and isClosure.', () => {

			const joint = new Joint();
			const child = new Link();

			expect( joint.isClosure ).toEqual( false );
			expect( joint.child ).toEqual( null );

			joint.makeClosure( child );

			expect( joint.isClosure ).toEqual( true );
			expect( joint.child ).toEqual( child );
			expect( child.closureJoints ).toEqual( [ joint ] );

		} );

		it( 'should throw if called with a non link.', () => {

			const joint = new Joint();
			const joint2 = new Joint();

			let caught;
			caught = false;
			try {

				joint.makeClosure( joint2 );

			} catch ( e ) {

				caught = true;

			}

			expect( caught ).toBeTruthy();

		} );

		it( 'should throw if the joint already has a child.', () => {

			const joint = new Joint();
			const child = new Link();
			const child2 = new Link();

			joint.addChild( child );
			expect( joint.isClosure ).toEqual( false );

			let caught;
			caught = false;
			try {

				joint.makeClosure( child2 );

			} catch ( e ) {

				caught = true;

			}

			expect( child.closureJoints ).toEqual( [] );
			expect( caught ).toBeTruthy();

		} );

	} );

	describe( 'removeChild', () => {

		it( 'should remove the closure child.', () => {

			const joint = new Joint();
			const child = new Link();

			joint.makeClosure( child );
			expect( child.closureJoints ).toEqual( [ joint ] );

			joint.removeChild( child );

			expect( joint.isClosure ).toEqual( false );
			expect( joint.child ).toEqual( null );
			expect( child.closureJoints ).toEqual( [] );

		} );

		it( 'should throw if the child is not the closure child.', () => {

			const joint = new Joint();
			const child = new Link();
			const child2 = new Link();

			joint.makeClosure( child );

			let caught = false;
			try {

				joint.removeChild( child2 );

			} catch ( e ) {

				caught = true;

			}

			expect( child.closureJoints ).toEqual( [ joint ] );
			expect( caught ).toBeTruthy();

		} );

	} );

	describe( 'findRoots', () => {

		it( 'should find roots of connected closure joints.', () => {

			const joint = new Joint();
			const parent = new Link();
			const closureChild = new Link();

			const joint2 = new Joint();
			const closureChild2 = new Link();

			parent.addChild( joint );
			joint.makeClosure( closureChild );

			closureChild.addChild( joint2 );
			joint2.makeClosure( closureChild2 );

			const roots = findRoots( [ parent ] );
			expect( roots ).toEqual( [ parent, closureChild, closureChild2 ] );

			const roots2 = findRoots( [ closureChild ] );
			expect( roots2 ).toEqual( [ closureChild, parent, closureChild2 ] );

		} );

	} );

	describe( 'getDeltaWorldMatrix', () => {

		it( 'should describe an offset world matrix based on the give DoF.', () => {

			const joint = new Joint();
			joint.setDoF( DOF.X );
			joint.setMinLimit( DOF.X, 0 );
			joint.setMaxLimit( DOF.X, 0.5 );
			joint.setDoFValue( DOF.X, 0.4 );
			joint.updateMatrixWorld();

			const outMatrix = new Float32Array( 16 );
			const inverted = joint.getDeltaWorldMatrix( DOF.X, 0.2, outMatrix );
			const pos = new Float32Array( 3 );
			mat4.getTranslation( pos, outMatrix );

			expect( inverted ).toBeTruthy();
			expect( pos[ 0 ] ).toBeCloseTo( 0.2, 6 );
			expect( pos[ 1 ] ).toBeCloseTo( 0, 7 );
			expect( pos[ 2 ] ).toBeCloseTo( 0, 7 );

		} );

		it( 'should return true if the delta was inverted due to a joint limit.', () => {

			const joint = new Joint();
			joint.setPosition( 0.1, - 0.2, 0.3 );
			joint.setDoF( DOF.EY );
			joint.updateMatrixWorld();

			const outMatrix = new Float32Array( 16 );
			const inverted = joint.getDeltaWorldMatrix( DOF.EY, 0.25, outMatrix );
			const pos = new Float32Array( 3 );
			const outQuat = new Float32Array( 4 );

			mat4.getTranslation( pos, outMatrix );
			mat4.getRotation( outQuat, outMatrix );

			const expectedQuat = new Float32Array( 4 );
			quat.fromEuler( expectedQuat, 0, 0.25 * RAD2DEG, 0 );

			expect( inverted ).toBeFalsy();
			expect( pos[ 0 ] ).toBeCloseTo( 0.1, 7 );
			expect( pos[ 1 ] ).toBeCloseTo( - 0.2, 7 );
			expect( pos[ 2 ] ).toBeCloseTo( 0.3, 7 );
			expect( quaternionDistance( outQuat, expectedQuat ) ).toBeLessThan( 1e-7 );

		} );

	} );

	describe( 'getClosureError', () => {

		it( 'should throw if not a closure.', () => {

			const joint = new Joint();
			const pos = new Float32Array( 3 );
			const quatArray = new Float32Array( 4 );

			expect( () => joint.getClosureError( pos, quatArray ) ).toThrow();

		} );

		it( 'should return the delta pos and quat between the joint and closure link.', () => {

			const joint = new Joint();
			const link = new Link();

			joint.setPosition( 0.25, - 0.5, 0.75 );
			link.setPosition( 1.25, - 0.25, 0.5 );
			link.setQuaternion( 0, 0, 0, 1 );

			joint.makeClosure( link );

			joint.updateMatrixWorld( true );
			link.updateMatrixWorld( true );

			const pos = new Float32Array( 3 );
			const quatArray = new Float32Array( 4 );

			joint.getClosureError( pos, quatArray );

			expect( pos[ 0 ] ).toBeCloseTo( - 1, 6 );
			expect( pos[ 1 ] ).toBeCloseTo( - 0.25, 6 );
			expect( pos[ 2 ] ).toBeCloseTo( 0.25, 6 );
			expect( quatArray[ 0 ] ).toBeCloseTo( 0, 7 );
			expect( quatArray[ 1 ] ).toBeCloseTo( 0, 7 );
			expect( quatArray[ 2 ] ).toBeCloseTo( 0, 7 );
			expect( quatArray[ 3 ] ).toBeCloseTo( 0, 7 );

		} );

	} );

	describe( 'updateMatrixDoF', () => {

		it( 'should update matrixDoF using updateDoFMatrix based on the dofValues.', () => {

			const joint = new Joint();
			joint.setDoF( DOF.X, DOF.EZ );
			joint.setDoFValues( 0.3, Math.PI / 4 );

			const pos = new Float32Array( 3 );
			const outQuat = new Float32Array( 4 );

			joint.updateDoFMatrix();
			mat4.getTranslation( pos, joint.matrixDoF );
			mat4.getRotation( outQuat, joint.matrixDoF );

			const expectedQuat = new Float32Array( 4 );
			quat.fromEuler( expectedQuat, 0, 0, Math.PI / 4 * RAD2DEG );

			expect( pos[ 0 ] ).toBeCloseTo( 0.3, 7 );
			expect( pos[ 1 ] ).toBeCloseTo( 0, 7 );
			expect( pos[ 2 ] ).toBeCloseTo( 0, 7 );
			expect( quaternionDistance( outQuat, expectedQuat ) ).toBeLessThan( 1e-7 );
			expect( joint.matrixDoFNeedsUpdate ).toBeFalsy();

		} );

	} );

	describe( 'attachChild', () => {

		it( 'should account for the DoF matrix when updating the world transform.', () => {

			const joint = new Joint();
			const child = new Link();

			joint.setDoF( DOF.EZ );
			joint.setDoFValue( DOF.EZ, Math.PI / 2 );
			joint.setPosition( 0.25, 0.5, - 0.25 );
			joint.updateMatrixWorld( true );

			child.setPosition( 1, - 1, 0.5 );
			child.updateMatrixWorld();

			const startPos = new Float32Array( 3 );
			child.getWorldPosition( startPos );

			joint.attachChild( child );
			child.updateMatrixWorld();

			const endPos = new Float32Array( 3 );
			const endQuat = new Float32Array( 4 );
			child.getWorldPosition( endPos );
			child.getWorldQuaternion( endQuat );

			const inverseDoF = new Float32Array( 16 );
			const expectedPos = new Float32Array( 3 );
			const expectedQuat = new Float32Array( 4 );

			mat4.fromRotation( inverseDoF, - Math.PI / 2, [ 0, 0, 1 ] );
			vec3.sub( expectedPos, startPos, joint.position );
			vec3.transformMat4( expectedPos, expectedPos, inverseDoF );
			vec3.add( expectedPos, expectedPos, joint.position );
			quat.setAxisAngle( expectedQuat, [ 0, 0, 1 ], - Math.PI / 2 );

			expect( endPos[ 0 ] ).toBeCloseTo( expectedPos[ 0 ], 6 );
			expect( endPos[ 1 ] ).toBeCloseTo( expectedPos[ 1 ], 6 );
			expect( endPos[ 2 ] ).toBeCloseTo( expectedPos[ 2 ], 6 );
			expect( quaternionDistance( endQuat, expectedQuat ) ).toBeLessThan( 1e-7 );

		} );

	} );

	describe( 'detachChild', () => {

		it( 'should account for the DoF matrix when updating the world transform.', () => {

			const joint = new Joint();
			const child = new Link();

			joint.setDoF( DOF.EZ );
			joint.setDoFValue( DOF.EZ, - Math.PI / 4 );
			joint.setPosition( - 0.5, 0.25, 0.75 );
			joint.updateMatrixWorld( true );

			child.setPosition( - 1, 0.5, 0.25 );
			child.updateMatrixWorld();

			const startPos = new Float32Array( 3 );
			child.getWorldPosition( startPos );

			joint.attachChild( child );
			child.updateMatrixWorld();

			joint.detachChild( child );
			child.updateMatrixWorld();

			const endPos = new Float32Array( 3 );
			const endQuat = new Float32Array( 4 );
			child.getWorldPosition( endPos );
			child.getWorldQuaternion( endQuat );

			const inverseDoF = new Float32Array( 16 );
			const toJoint = new Float32Array( 16 );
			const fromJoint = new Float32Array( 16 );
			const combinedTransform = new Float32Array( 16 );
			const negativeJoint = new Float32Array( 3 );
			const expectedPos = new Float32Array( 3 );
			const expectedQuat = new Float32Array( 4 );

			mat4.fromRotation( inverseDoF, Math.PI / 4, [ 0, 0, 1 ] );
			mat4.fromTranslation( toJoint, joint.position );
			vec3.negate( negativeJoint, joint.position );
			mat4.fromTranslation( fromJoint, negativeJoint );

			// Apply inverseDoF about the joint (T(joint) * inverseDoF * T(-joint))
			// and then apply inverseDoF again to mirror detachChild updating the
			// world matrix after removal: inverseDoF * T(joint) * inverseDoF * T(-joint).
			mat4.multiply( combinedTransform, inverseDoF, fromJoint );
			mat4.multiply( combinedTransform, toJoint, combinedTransform );
			mat4.multiply( combinedTransform, inverseDoF, combinedTransform );
			vec3.transformMat4( expectedPos, startPos, combinedTransform );
			quat.setAxisAngle( expectedQuat, [ 0, 0, 1 ], Math.PI / 2 );

			expect( endPos[ 0 ] ).toBeCloseTo( expectedPos[ 0 ], 6 );
			expect( endPos[ 1 ] ).toBeCloseTo( expectedPos[ 1 ], 6 );
			expect( endPos[ 2 ] ).toBeCloseTo( expectedPos[ 2 ], 6 );
			expect( quaternionDistance( endQuat, expectedQuat ) ).toBeLessThan( 1e-7 );

		} );

	} );

} );
