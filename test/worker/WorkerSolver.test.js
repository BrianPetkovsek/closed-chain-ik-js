import { WorkerSolver } from '../../src/worker/WorkerSolver.js';
import { Joint, DOF } from '../../src/core/Joint.js';
import { Link } from '../../src/core/Link.js';
import { copyFrameToBuffer, JOINT_STRIDE } from '../../src/worker/utils.js';
import { SOLVE_STATUS } from '../../src/core/ChainSolver.js';

class MockWorker {

	constructor() {

		this.onmessage = null;

	}

	postMessage() {}
	terminate() {}

}

describe( 'WorkerSolver', () => {

	let originalWorker;

	beforeEach( () => {

		originalWorker = globalThis.Worker;
		globalThis.Worker = MockWorker;

	} );

	afterEach( () => {

		globalThis.Worker = originalWorker;

	} );

	it( 'ignores stale updateSolve events after structure updates.', async () => {

		const root = new Link();
		const joint = new Joint();
		const end = new Link();
		joint.setDoF( DOF.X );
		joint.setDoFValue( DOF.X, 0 );
		root.addChild( joint );
		joint.addChild( end );

		const workerSolver = new WorkerSolver( root );
		const jointIndex = workerSolver.jointsToIndexMap.get( joint );
		const offset = jointIndex * JOINT_STRIDE;

		const updatedJoint = new Joint();
		updatedJoint.setDoF( DOF.X );
		updatedJoint.setDoFValue( DOF.X, 2 );
		copyFrameToBuffer( updatedJoint, workerSolver.floatBuffer, workerSolver.byteBuffer, offset, true, false );

		workerSolver.worker.onmessage( {
			data: {
				type: 'updateSolve',
				data: {
					status: SOLVE_STATUS.CONVERGED,
					structureVersion: workerSolver.structureVersion - 1,
				},
			},
		} );
		await Promise.resolve();
		expect( joint.getDoFValue( DOF.X ) ).toBe( 0 );

		workerSolver.worker.onmessage( {
			data: {
				type: 'updateSolve',
				data: {
					status: SOLVE_STATUS.CONVERGED,
					structureVersion: workerSolver.structureVersion,
				},
			},
		} );
		await Promise.resolve();
		expect( joint.getDoFValue( DOF.X ) ).toBeCloseTo( 2, 8 );

		workerSolver.dispose();

	} );

} );
