import { normalizeFilePath } from '../lib/ccik-wasm.js';

describe( 'normalizeFilePath', () => {

	it( 'decodes file URLs and keeps drive letters on Windows-style paths', () => {

		expect( normalizeFilePath( 'file:///C:/Program%20Files/ccik/ccik.wasm' ) ).toBe( 'C:/Program Files/ccik/ccik.wasm' );

	} );

	it( 'joins relative paths with script prefix', () => {

		expect( normalizeFilePath( 'ccik.wasm', '/opt/app/dist' ) ).toBe( '/opt/app/dist/ccik.wasm' );

	} );

} );
