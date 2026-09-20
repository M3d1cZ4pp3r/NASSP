#pragma once

#include "Orbitersdk.h"

#include <algorithm>
#include <cmath>

namespace nasspmp_kinematics
{
struct Quaternion
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double w = 1.0;
};

struct State
{
	VECTOR3 position = {};
	VECTOR3 velocity = {};
	VECTOR3 acceleration = {};
	Quaternion orientation;
	VECTOR3 angularVelocity = {};
};

inline Quaternion Normalize(const Quaternion &value)
{
	const double length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
	if (length <= 0.0)
		return {};
	return { value.x / length, value.y / length, value.z / length, value.w / length };
}

inline Quaternion Inverse(const Quaternion &value)
{
	const Quaternion normalized = Normalize(value);
	return { -normalized.x, -normalized.y, -normalized.z, normalized.w };
}

// Converts Orbiter's rotation matrix into a representation suitable for interpolation.
inline Quaternion FromRotationMatrix(const MATRIX3 &rotation)
{
	Quaternion result;
	const double trace = 1.0 + rotation.m11 + rotation.m22 + rotation.m33;
	if (trace > 1e-12) {
		const double scale = 2.0 * std::sqrt(trace);
		result = { (rotation.m23 - rotation.m32) / scale, (rotation.m31 - rotation.m13) / scale,
			(rotation.m12 - rotation.m21) / scale, 0.25 * scale };
	} else if (rotation.m11 > rotation.m22 && rotation.m11 > rotation.m33) {
		const double scale = 2.0 * std::sqrt(1.0 + rotation.m11 - rotation.m22 - rotation.m33);
		result = { 0.25 * scale, (rotation.m12 + rotation.m21) / scale,
			(rotation.m31 + rotation.m13) / scale, (rotation.m23 - rotation.m32) / scale };
	} else if (rotation.m22 > rotation.m33) {
		const double scale = 2.0 * std::sqrt(1.0 + rotation.m22 - rotation.m11 - rotation.m33);
		result = { (rotation.m12 + rotation.m21) / scale, 0.25 * scale,
			(rotation.m23 + rotation.m32) / scale, (rotation.m31 - rotation.m13) / scale };
	} else {
		const double scale = 2.0 * std::sqrt(1.0 + rotation.m33 - rotation.m11 - rotation.m22);
		result = { (rotation.m31 + rotation.m13) / scale, (rotation.m23 + rotation.m32) / scale,
			0.25 * scale, (rotation.m12 - rotation.m21) / scale };
	}
	return Normalize(result);
}

// Produces Orbiter's left-handed local-to-global rotation convention.
inline MATRIX3 ToRotationMatrix(const Quaternion &value)
{
	const Quaternion rotation = Normalize(value);
	const double x2 = 2.0 * rotation.x;
	const double y2 = 2.0 * rotation.y;
	const double z2 = 2.0 * rotation.z;
	return _M(1.0 - y2 * rotation.y - z2 * rotation.z,
		x2 * rotation.y + z2 * rotation.w, x2 * rotation.z - y2 * rotation.w,
		x2 * rotation.y - z2 * rotation.w, 1.0 - x2 * rotation.x - z2 * rotation.z,
		y2 * rotation.z + x2 * rotation.w, x2 * rotation.z + y2 * rotation.w,
		y2 * rotation.z - x2 * rotation.w, 1.0 - x2 * rotation.x - y2 * rotation.y);
}

// Multiplies quaternions using Orbiter's left-handed rotation convention.
inline Quaternion Multiply(const Quaternion &left, const Quaternion &right)
{
	return {
		left.w * right.x + left.x * right.w - left.y * right.z + left.z * right.y,
		left.w * right.y + left.x * right.z + left.y * right.w - left.z * right.x,
		left.w * right.z - left.x * right.y + left.y * right.x + left.z * right.w,
		left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z
	};
}

inline Quaternion IntegrateOrientation(const Quaternion &orientation, const VECTOR3 &angularVelocity, double seconds)
{
	// Orbiter reports angular velocity in vessel-local axes, so post-multiply the
	// current global orientation by the local incremental rotation.
	const double angularSpeed = length(angularVelocity);
	if (angularSpeed <= 0.0 || seconds == 0.0)
		return Normalize(orientation);

	const double halfAngle = angularSpeed * seconds * 0.5;
	const double axisScale = std::sin(halfAngle) / angularSpeed;
	const Quaternion increment = {
		angularVelocity.x * axisScale,
		angularVelocity.y * axisScale,
		angularVelocity.z * axisScale,
		std::cos(halfAngle)
	};
	return Normalize(Multiply(orientation, increment));
}

inline Quaternion Slerp(Quaternion from, Quaternion to, double fraction)
{
	// q and -q describe the same orientation. Select the shorter interpolation arc.
	fraction = (std::max)(0.0, (std::min)(1.0, fraction));
	double dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
	if (dot < 0.0) {
		to = { -to.x, -to.y, -to.z, -to.w };
		dot = -dot;
	}

	if (dot > 0.9995) {
		return Normalize({
			from.x + (to.x - from.x) * fraction,
			from.y + (to.y - from.y) * fraction,
			from.z + (to.z - from.z) * fraction,
			from.w + (to.w - from.w) * fraction
		});
	}

	const double angle = std::acos((std::max)(-1.0, (std::min)(1.0, dot)));
	const double divisor = std::sin(angle);
	const double fromScale = std::sin((1.0 - fraction) * angle) / divisor;
	const double toScale = std::sin(fraction * angle) / divisor;
	return Normalize({
		from.x * fromScale + to.x * toScale,
		from.y * fromScale + to.y * toScale,
		from.z * fromScale + to.z * toScale,
		from.w * fromScale + to.w * toScale
	});
}

inline double AngularDistance(const Quaternion &left, const Quaternion &right)
{
	// The absolute dot product likewise treats both quaternion signs as identical.
	const Quaternion normalizedLeft = Normalize(left);
	const Quaternion normalizedRight = Normalize(right);
	const double dot = std::abs(normalizedLeft.x * normalizedRight.x + normalizedLeft.y * normalizedRight.y +
		normalizedLeft.z * normalizedRight.z + normalizedLeft.w * normalizedRight.w);
	return 2.0 * std::acos((std::max)(-1.0, (std::min)(1.0, dot)));
}

// Propagates one authority sample over a signed time interval.
inline State Extrapolate(const State &sample, double seconds)
{
	State predicted = sample;
	predicted.position = sample.position + sample.velocity * seconds + sample.acceleration * (0.5 * seconds * seconds);
	predicted.velocity = sample.velocity + sample.acceleration * seconds;
	predicted.orientation = IntegrateOrientation(sample.orientation, sample.angularVelocity, seconds);
	return predicted;
}

}
