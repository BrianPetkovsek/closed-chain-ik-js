function normalizeFilePath( path, prefix = '' ) {

	if ( path.startsWith( 'file://' ) ) {

		return new URL( path ).pathname;

	}

	if ( path.startsWith( '/' ) || /^[A-Za-z]:[\\/]/.test( path ) || path.startsWith( 'http' ) ) {

		return path;

	}

	if ( prefix ) {

		const prefixed = prefix + path;
		return prefixed.startsWith( 'file://' ) ? new URL( prefixed ).pathname : prefixed;

	}

	return path;

}

let modulePromise = null;

export async function loadCCIK( factory, options = {} ) {
	if ( ! modulePromise ) {
		const ccikFactory = factory || ( await import( '../dist/ccik.js' ) ).default;
		const locateFile =
			options.locateFile || ( ( path, prefix ) => normalizeFilePath( path, prefix ) );
		modulePromise = ccikFactory( { ...options, locateFile } );
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
