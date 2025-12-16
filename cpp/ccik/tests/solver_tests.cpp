#include "ccik/IKSolver.h"
#include <gtest/gtest.h>
#include <limits>
#include <vector>

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

TEST( SolverTests, SolvesSpatialThreeLinkWithTightTolerance ) {
	constexpr double halfPi = std::acos( -1.0 ) * 0.5;

	JointSpec j1;
	j1.axis = { 0, 0, 1 };
	j1.length = 0.8;
	j1.mode = JointMode::Rotation;
	j1.value = halfPi * 0.5;

	JointSpec j2;
	j2.axis = { 0, 1, 0 };
	j2.length = 0.7;
	j2.mode = JointMode::Rotation;
	j2.value = 0.0;

	JointSpec j3;
	j3.axis = { 1, 0, 0 };
	j3.length = 0.6;
	j3.mode = JointMode::Rotation;
	j3.value = 0.0;

	Chain c( { j1, j2, j3 } );
	IKSolver solver( c );
	solver.setTolerance( 1e-6 );
	const auto baseline = solver.getPositions().back();
	solver.setTarget( baseline );

	const double error = solver.solve( 60 );
	EXPECT_LT( error, 1e-4 );

	const auto pos = solver.getPositions().back();
	EXPECT_NEAR( pos.x, baseline.x, 1e-5 );
	EXPECT_NEAR( pos.y, baseline.y, 1e-5 );
	EXPECT_NEAR( pos.z, baseline.z, 1e-5 );
}

TEST( SolverTests, HonorsRotationLimitsAcrossTargets ) {
	JointSpec limited;
	limited.axis = { 0, 0, 1 };
	limited.length = 1.0;
	limited.mode = JointMode::Rotation;
	limited.minLimit = -0.5;
	limited.maxLimit = 0.5;

	JointSpec free = limited;
	free.minLimit = -std::numeric_limits<double>::infinity();
	free.maxLimit = std::numeric_limits<double>::infinity();

	Chain c( { limited, free } );
	IKSolver solver( c );
	solver.setTolerance( 1e-6 );

	const std::vector<Vec3> targets = {
		{ 1.0, 0.0, 2.0 },
		{ 0.5, 0.5, 2.0 },
		{ -0.5, 0.5, 2.0 },
	};

	for ( const auto &t : targets ) {
		solver.setTarget( t );
		solver.solve( 40 );

		const auto pos = solver.getPositions().back();
		EXPECT_NEAR( pos.z, t.z, 1e-2 );

		const auto &specs = c.getJoints();
		ASSERT_EQ( specs.size(), 2u );
		EXPECT_GE( specs[ 0 ].value, limited.minLimit - 1e-8 );
		EXPECT_LE( specs[ 0 ].value, limited.maxLimit + 1e-8 );
	}
}
