import { DOF } from '../src/core/Joint.js';

export function axisToDof( axis ) {

	if ( axis[ 0 ] ) return DOF.EX;
	if ( axis[ 1 ] ) return DOF.EY;
	return DOF.EZ;

}
