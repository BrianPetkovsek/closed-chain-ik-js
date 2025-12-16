export function axisToDof( axis, DOF ) {

	const [ x, y, z ] = axis;
	const abs = [ Math.abs( x ), Math.abs( y ), Math.abs( z ) ];
	const max = Math.max( abs[ 0 ], abs[ 1 ], abs[ 2 ] );

	if ( max === 0 ) {
		throw new Error( 'axisToDof: axis cannot be zero-length.' );
	}

	const maxCount = Number( max === abs[ 0 ] ) + Number( max === abs[ 1 ] ) + Number( max === abs[ 2 ] );
	if ( maxCount > 1 ) {
		throw new Error( 'axisToDof: axis is ambiguous across multiple components.' );
	}

	if ( max === abs[ 0 ] ) return DOF.EX;
	if ( max === abs[ 1 ] ) return DOF.EY;
	return DOF.EZ;

}
