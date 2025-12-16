#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace ccik {

// Shared numeric tolerance used across C++ and JS parity tests.
constexpr double CCIK_TOLERANCE = 1e-6;

struct Vec3 {
	double x { 0.0 };
	double y { 0.0 };
	double z { 0.0 };

	Vec3() = default;
	Vec3( double ix, double iy, double iz ) : x( ix ), y( iy ), z( iz ) {}

	double length() const {
		return std::sqrt( x * x + y * y + z * z );
	}

	Vec3 normalized() const {
		const double len = length();
		if ( len <= std::numeric_limits<double>::epsilon() ) {
			return { 0.0, 0.0, 0.0 };
		}
		return { x / len, y / len, z / len };
	}

	Vec3 operator+( const Vec3 &o ) const {
		return { x + o.x, y + o.y, z + o.z };
	}

	Vec3 operator-( const Vec3 &o ) const {
		return { x - o.x, y - o.y, z - o.z };
	}

	Vec3 operator*( double s ) const {
		return { x * s, y * s, z * s };
	}
};

inline Vec3 cross( const Vec3 &a, const Vec3 &b ) {
	return Vec3(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

inline double dot( const Vec3 &a, const Vec3 &b ) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

enum class JointMode {
	Rotation = 0,
	Translation = 1,
};

struct JointSpec {
	Vec3 axis { 0.0, 0.0, 1.0 };
	double length { 0.0 };
	JointMode mode { JointMode::Rotation };
	double minLimit { -std::numeric_limits<double>::infinity() };
	double maxLimit { std::numeric_limits<double>::infinity() };
	double value { 0.0 };
	std::string name {};
};

} // namespace ccik
