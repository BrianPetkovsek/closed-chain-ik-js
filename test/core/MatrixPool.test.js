import { MatrixPool } from '../../src/core/MatrixPool.js';

describe( 'MatrixPool', () => {

	it( 'should return a matrix with the requested rows and columns.', () => {

		const pool = new MatrixPool();
		let mat;

		mat = pool.get( 2, 3 );
		expect( mat ).toHaveLength( 2 );
		expect( mat[ 0 ] ).toHaveLength( 3 );

		mat = pool.get( 5, 4 );
		expect( mat ).toHaveLength( 5 );
		expect( mat[ 0 ] ).toHaveLength( 4 );

	} );

	it( 'should return different matrices after each get call.', () => {

		const pool = new MatrixPool();

		const mat1 = pool.get( 2, 3 );
		const mat2 = pool.get( 2, 3 );

		expect( mat1 ).not.toBe( mat2 );

	} );

	it( 'should reissue matrices after releaseAll is called.', () => {

		const pool = new MatrixPool();
		const mat1 = pool.get( 2, 3 );
		const mat2 = pool.get( 2, 3 );

		pool.releaseAll();

		const mat3 = pool.get( 2, 3 );
		const mat4 = pool.get( 2, 3 );

		expect( mat1 ).toBe( mat3 );
		expect( mat2 ).toBe( mat4 );

	} );

} );
