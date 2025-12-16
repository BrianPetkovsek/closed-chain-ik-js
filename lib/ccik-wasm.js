import ccikFactory from '../dist/ccik.js';

let modulePromise = null;

export {
	Link,
	Joint,
	Goal,
	Solver,
	WorkerSolver,
	DOF,
	DOF_NAMES,
	SOLVE_STATUS,
	SOLVE_STATUS_NAMES,
	IKRootsHelper,
	setIKFromUrdf,
	setUrdfFromIK,
	urdfRobotToIKRoot,
	findRoots,
} from '../src/index.js';

export function loadCCIK( factory = ccikFactory, options = {} ) {
	if ( ! modulePromise ) {
		modulePromise = factory( options );
	}

	return modulePromise.then( ccik => ( {
		...ccik,
		Chain: ccik.Chain,
		IKSolver: ccik.IKSolver,
		tolerance: ccik.CCIK_TOLERANCE,
	} ) );
}

export default loadCCIK;
