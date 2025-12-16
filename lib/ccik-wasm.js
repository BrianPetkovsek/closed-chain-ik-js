/**
 * Normalize a wasm asset path by converting file:// URLs to filesystem paths and
 * preserving absolute POSIX, Windows, or HTTP(S) paths.
 * @param {string} path Path or URL provided by the Emscripten loader.
 * @param {string} prefix Optional script directory prefix.
 * @returns {string} Filesystem path or untouched URL string.
 */
function normalizeFilePath( path, prefix = '' ) {

	const toPathname = value => {

		if ( ! value.startsWith( 'file://' ) ) {

			return value;

		}

		try {

			return new URL( value ).pathname;

		} catch ( e ) {

			return value;

		}

	};

	const normalized = toPathname( path );
	if ( normalized !== path ) {

		return normalized;

	}

	// POSIX absolute (/root/...) or Windows drive paths (C:\...)
	if (
		normalized.startsWith( '/' ) ||
		/^[A-Za-z]:[/\\]/.test( normalized ) ||
		normalized.startsWith( 'http://' ) ||
		normalized.startsWith( 'https://' )
	) {

		return normalized;

	}

	if ( prefix ) {

		const normalizedPrefix = toPathname( prefix );
		const needsSeparator = !normalizedPrefix.endsWith( '/' ) && !normalized.startsWith( '/' );
		return `${normalizedPrefix}${needsSeparator ? '/' : ''}${normalized}`;

	}

	return normalized;

}

let modulePromise = null;

export async function loadCCIK( factory, options = {} ) {
	if ( ! modulePromise ) {
		const ccikFactory = factory || ( await import( '../dist/ccik.js' ) ).default;
		const locateFile = options.locateFile || ( ( path, prefix ) => normalizeFilePath( path, prefix ) );
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
