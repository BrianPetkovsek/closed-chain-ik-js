#include "ccik/Chain.h"
#include <array>
#include <cmath>

namespace ccik {
namespace {

using Mat3 = std::array<double, 9>;

Mat3 identity() {
	return { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
}

Mat3 multiply( const Mat3 &a, const Mat3 &b ) {
	Mat3 r {};
	for ( int row = 0; row < 3; ++row ) {
		for ( int col = 0; col < 3; ++col ) {
			r[ row * 3 + col ] =
				a[ row * 3 + 0 ] * b[ 0 * 3 + col ] +
				a[ row * 3 + 1 ] * b[ 1 * 3 + col ] +
				a[ row * 3 + 2 ] * b[ 2 * 3 + col ];
		}
	}
	return r;
}

Vec3 applyMat( const Mat3 &m, const Vec3 &v ) {
	return Vec3(
		m[ 0 ] * v.x + m[ 1 ] * v.y + m[ 2 ] * v.z,
		m[ 3 ] * v.x + m[ 4 ] * v.y + m[ 5 ] * v.z,
		m[ 6 ] * v.x + m[ 7 ] * v.y + m[ 8 ] * v.z
	);
}

Mat3 rotationAroundAxis( const Vec3 &axis, double theta ) {
	const Vec3 n = axis.normalized();
	const double c = std::cos( theta );
	const double s = std::sin( theta );
	const double t = 1.0 - c;

	return Mat3{
		t * n.x * n.x + c,        t * n.x * n.y - s * n.z, t * n.x * n.z + s * n.y,
		t * n.x * n.y + s * n.z,  t * n.y * n.y + c,       t * n.y * n.z - s * n.x,
		t * n.x * n.z - s * n.y,  t * n.y * n.z + s * n.x, t * n.z * n.z + c
	};
}

} // namespace

Chain::Chain( const std::vector<JointSpec> &specs, const Vec3 &base ) : m_base( base ), m_specs( specs ) {}

void Chain::setBasePosition( const Vec3 &base ) {
	m_base = base;
}

const Vec3 &Chain::getBasePosition() const {
	return m_base;
}

void Chain::setJoints( const std::vector<JointSpec> &specs ) {
	m_specs = specs;
}

std::vector<JointSpec> &Chain::accessJoints() {
	return m_specs;
}

const std::vector<JointSpec> &Chain::getJoints() const {
	return m_specs;
}

std::vector<JointState> Chain::getJointStates() const {
	std::vector<JointState> out;
	out.reserve( m_specs.size() );

	Mat3 rot = identity();
	Vec3 pos = m_base;

	for ( const auto &spec : m_specs ) {
		Vec3 axisWorld = applyMat( rot, spec.axis ).normalized();

		if ( spec.mode == JointMode::Rotation ) {
			pos = pos + applyMat( rot, Vec3( 0.0, 0.0, spec.length ) );
			out.push_back( { pos, axisWorld, spec.mode } );
			rot = multiply( rot, rotationAroundAxis( axisWorld, spec.value ) );
		} else {
			out.push_back( { pos, axisWorld, spec.mode } );
			pos = pos + axisWorld * spec.value;
			pos = pos + axisWorld * spec.length;
		}
	}

	return out;
}

std::vector<Vec3> Chain::getPositions() const {
	std::vector<Vec3> positions;
	positions.reserve( m_specs.size() + 1 );

	Mat3 rot = identity();
	Vec3 pos = m_base;

	for ( const auto &spec : m_specs ) {
		Vec3 axisWorld = applyMat( rot, spec.axis ).normalized();

		if ( spec.mode == JointMode::Rotation ) {
			pos = pos + applyMat( rot, Vec3( 0.0, 0.0, spec.length ) );
			positions.push_back( pos );
			rot = multiply( rot, rotationAroundAxis( axisWorld, spec.value ) );
		} else {
			positions.push_back( pos );
			pos = pos + axisWorld * spec.value;
			pos = pos + axisWorld * spec.length;
		}
	}

	positions.push_back( pos );
	return positions;
}

Vec3 Chain::getEndEffector() const {
	const auto positions = getPositions();
	return positions.empty() ? m_base : positions.back();
}

} // namespace ccik
