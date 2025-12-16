#pragma once

#include "Types.h"
#include <vector>

namespace ccik {

struct JointState {
	Vec3 position;
	Vec3 axisWorld;
	JointMode mode;
};

class Chain {
public:
	explicit Chain( const std::vector<JointSpec> &specs = {}, const Vec3 &base = Vec3{} );

	void setBasePosition( const Vec3 &base );
	const Vec3 &getBasePosition() const;

	void setJoints( const std::vector<JointSpec> &specs );
	std::vector<JointSpec> &accessJoints();
	const std::vector<JointSpec> &getJoints() const;

	std::vector<Vec3> getPositions() const;
	Vec3 getEndEffector() const;

	std::vector<JointState> getJointStates() const;

private:
	Vec3 m_base;
	std::vector<JointSpec> m_specs;
};

} // namespace ccik
