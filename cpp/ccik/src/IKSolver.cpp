#include "ccik/IKSolver.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ccik {
namespace {

double length( const Vec3 &v ) {
	return std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
}

// Solves A * x = b for small dense matrices using Gaussian elimination.
std::vector<double> solveLinear( std::vector<std::vector<double>> A, const std::vector<double> &b ) {
	const std::size_t n = A.size();
	std::vector<double> x( n, 0.0 );
	if ( n == 0 ) return x;

	std::vector<double> rhs = b;

	for ( std::size_t i = 0; i < n; ++i ) {
		double pivot = A[ i ][ i ];
		if ( std::abs( pivot ) < std::numeric_limits<double>::epsilon() ) {
			// Avoid singular pivots by nudging the diagonal; this mirrors the
			// damping term used elsewhere and prevents division by zero when
			// the jacobian is rank deficient.
			pivot = std::copysign( std::numeric_limits<double>::epsilon(), pivot == 0.0 ? 1.0 : pivot );
			A[ i ][ i ] = pivot;
		}

		for ( std::size_t j = i; j < n; ++j ) {
			A[ i ][ j ] /= pivot;
		}
		rhs[ i ] /= pivot;

		for ( std::size_t r = i + 1; r < n; ++r ) {
			const double factor = A[ r ][ i ];
			for ( std::size_t c = i; c < n; ++c ) {
				A[ r ][ c ] -= factor * A[ i ][ c ];
			}
			rhs[ r ] -= factor * rhs[ i ];
		}
	}

	for ( std::size_t i = n; i-- > 0; ) {
		double accum = 0.0;
		for ( std::size_t c = i + 1; c < n; ++c ) {
			accum += A[ i ][ c ] * x[ c ];
		}
		x[ i ] = rhs[ i ] - accum;
	}

	return x;
}

} // namespace

IKSolver::IKSolver( const Chain &chain ) : m_chain( chain ), m_target( chain.getEndEffector() ) {}

void IKSolver::setChain( const Chain &chain ) {
	m_chain = chain;
	m_target = chain.getEndEffector();
}

const Chain &IKSolver::getChain() const {
	return m_chain;
}

void IKSolver::setTarget( const Vec3 &target ) {
	m_target = target;
}

const Vec3 &IKSolver::getTarget() const {
	return m_target;
}

void IKSolver::setDamping( double lambda ) {
	m_damping = std::max( 0.0, lambda );
}

void IKSolver::setTolerance( double tol ) {
	m_tolerance = std::max( tol, 0.0 );
}

void IKSolver::setStepScale( double step ) {
	m_stepScale = step;
}

double IKSolver::solve( int iterations ) {
	const int maxIterations = iterations < 0 ? 20 : iterations;

	for ( int iter = 0; iter < maxIterations; ++iter ) {
		const auto states = m_chain.getJointStates();
		const Vec3 end = m_chain.getEndEffector();
		const Vec3 diff = Vec3( m_target.x - end.x, m_target.y - end.y, m_target.z - end.z );
		const double err = length( diff );

		if ( err < m_tolerance ) {
			return err;
		}

		const std::size_t n = states.size();
		if ( n == 0 ) {
			return err;
		}

		// Build Jacobian J (3 x n)
		std::vector<std::array<double, 3>> J( n );
		for ( std::size_t i = 0; i < n; ++i ) {
			const auto &state = states[ i ];
			if ( state.mode == JointMode::Rotation ) {
				const Vec3 r = cross( state.axisWorld, Vec3( end.x - state.position.x, end.y - state.position.y, end.z - state.position.z ) );
				J[ i ] = { r.x, r.y, r.z };
			} else {
				J[ i ] = { state.axisWorld.x, state.axisWorld.y, state.axisWorld.z };
			}
		}

		// Compute JTJ and JTe
		std::vector<std::vector<double>> JTJ( n, std::vector<double>( n, 0.0 ) );
		std::vector<double> JTe( n, 0.0 );
		for ( std::size_t r = 0; r < n; ++r ) {
			for ( std::size_t c = 0; c < n; ++c ) {
				JTJ[ r ][ c ] = J[ r ][ 0 ] * J[ c ][ 0 ] + J[ r ][ 1 ] * J[ c ][ 1 ] + J[ r ][ 2 ] * J[ c ][ 2 ];
				if ( r == c ) {
					JTJ[ r ][ c ] += m_damping * m_damping;
				}
			}

			JTe[ r ] = J[ r ][ 0 ] * diff.x + J[ r ][ 1 ] * diff.y + J[ r ][ 2 ] * diff.z;
		}

		// Solve
		const std::vector<double> delta = solveLinear( JTJ, JTe );

		// Apply updates
		auto &specs = m_chain.accessJoints();
		double maxStep = 0.0;
		for ( std::size_t i = 0; i < specs.size(); ++i ) {
			auto &spec = specs[ i ];
			const double step = m_stepScale * delta[ i ];
			spec.value = std::clamp( spec.value + step, spec.minLimit, spec.maxLimit );
			maxStep = std::max( maxStep, std::abs( step ) );
		}

		if ( maxStep < m_tolerance * 0.1 ) {
			return err;
		}
	}

	const Vec3 end = m_chain.getEndEffector();
	const Vec3 diff = Vec3( m_target.x - end.x, m_target.y - end.y, m_target.z - end.z );
	return length( diff );
}

std::vector<Vec3> IKSolver::getPositions() const {
	return m_chain.getPositions();
}

} // namespace ccik
