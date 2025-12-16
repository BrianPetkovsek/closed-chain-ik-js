#include "ccik/IKSolver.h"
#include <gtest/gtest.h>

using namespace ccik;

TEST( SolverTests, SolvesPlanarTwoLink ) {
	JointSpec j1;
	j1.axis = { 0, 0, 1 };
	j1.length = 1.0;
	j1.mode = JointMode::Rotation;

	JointSpec j2 = j1;

	Chain c( { j1, j2 } );
	IKSolver solver( c );
	solver.setTarget( { 0.0, 0.0, 2.0 } );

	const double error = solver.solve( 10 );
	EXPECT_LE( error, 2.5 );

	const auto pos = solver.getPositions().back();
	EXPECT_NEAR( pos.z, 2.0, 1e-2 );
}
