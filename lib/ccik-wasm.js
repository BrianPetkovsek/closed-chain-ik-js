let modulePromise = null;

export async function loadCCIK( factory, options = {} ) {
	if ( ! modulePromise ) {
		const ccikFactory = factory || ( await import( '../dist/ccik.js' ) ).default;
		modulePromise = ccikFactory( options );
	}

	const ccik = await modulePromise;
	return {
		...ccik,
		Chain: ccik.Chain,
		IKSolver: ccik.IKSolver,
		tolerance: ccik.CCIK_TOLERANCE,
	};
}

export default loadCCIK;
