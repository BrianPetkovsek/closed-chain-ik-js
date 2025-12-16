#include "ccik/Chain.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace ccik;

TEST( ChainTests, ComputesForwardPositions ) {
	JointSpec j1;
	j1.axis = { 0, 0, 1 };
	j1.length = 1.0;
	j1.mode = JointMode::Rotation;
	j1.value = 0.0;

	JointSpec j2;
	j2.axis = { 0, 1, 0 };
	j2.length = 1.0;
	j2.mode = JointMode::Rotation;
	j2.value = 0.0;

	Chain c( { j1, j2 } );
	const auto positions = c.getPositions();
	ASSERT_EQ( positions.size(), 3u );
	EXPECT_NEAR( positions[ 0 ].x, 0.0, 1e-9 );
	EXPECT_NEAR( positions[ 1 ].x, 1.0, 1e-9 );
	EXPECT_NEAR( positions[ 2 ].x, 2.0, 1e-9 );
}

TEST( ChainTests, RespectsRotationOrientation ) {
	JointSpec base;
	base.axis = { 0, 0, 1 };
	base.length = 0.0;
	base.mode = JointMode::Rotation;
	const double halfPi = std::acos( -1.0 ) * 0.5;
	base.value = halfPi; // rotate 90 degrees

	JointSpec tip;
	tip.axis = { 1, 0, 0 };
	tip.length = 1.0;
	tip.mode = JointMode::Translation;
	tip.value = 0.0;

	Chain c( { base, tip } );
	const auto end = c.getEndEffector();
	// After rotating around Z, the translation axis points along +Y.
	EXPECT_NEAR( end.x, 0.0, 1e-9 );
	EXPECT_NEAR( end.y, 1.0, 1e-9 );
}
