import loadCCIK from '../lib/ccik-wasm.js';

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
solver.setTarget( { x: 0.5, y: 1.1, z: 0 } );
solver.solve( 32 );

const end = solver.getPositions().pop();
console.log( 'End effector:', end );
