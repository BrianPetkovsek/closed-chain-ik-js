#pragma once

#include "Chain.h"
#include <vector>

namespace ccik {

class IKSolver {
public:
	explicit IKSolver( const Chain &chain = Chain() );

	void setChain( const Chain &chain );
	const Chain &getChain() const;

	void setTarget( const Vec3 &target );
	const Vec3 &getTarget() const;

	void setDamping( double lambda );
	void setTolerance( double tol );
	void setStepScale( double step );

	// Returns the final absolute position error after solving.
	double solve( int iterations = 20 );

	std::vector<Vec3> getPositions() const;

private:
	Chain m_chain;
	Vec3 m_target;
	double m_damping { 1e-3 };
	double m_tolerance { CCIK_TOLERANCE };
	double m_stepScale { 1.0 };
};

} // namespace ccik
