/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_MATH_H
#define B2_MATH_H

#include <Box2D/Common/b2Settings.h>
#include <cmath>

namespace box2d
{
// forward declarations
struct Vec2;
struct Vec3;
constexpr inline float_t b2Dot(const Vec2& a, const Vec2& b) noexcept;
constexpr inline float_t b2Dot(const Vec3& a, const Vec3& b) noexcept;
constexpr inline float_t b2Cross(const Vec2& a, const Vec2& b) noexcept;
constexpr inline Vec2 b2Cross(const Vec2& a, float_t s) noexcept;
constexpr inline Vec3 b2Cross(const Vec3& a, const Vec3& b) noexcept;

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float_t x)
{
	return !std::isnan(x) && !std::isinf(x);
}

template<class T>
constexpr inline auto b2Square(T t) { return t * t; }

#define	b2Sqrt(x)	std::sqrt(x)
#define	b2Atan2(y, x)	std::atan2(y, x)

/// A 2D column vector.
struct Vec2
{
	/// Default constructor does nothing (for performance).
	Vec2() = default;
	
	Vec2(const Vec2& copy) = default;

	/// Construct using coordinates.
	constexpr Vec2(float_t x_, float_t y_) noexcept : x(x_), y(y_) {}

	/// Negate this vector.
	constexpr Vec2 operator -() const noexcept { return Vec2{-x, -y}; }

#if !defined(NO_B2VEC2_INDEXING)

	using index_t = unsigned;
	static constexpr auto NumElements = index_t(2);

	/// Read from and indexed element.
	float_t operator () (index_t i) const
	{
		assert((i >= 0) && (i < NumElements));
		switch (i)
		{
			case 0: return x;
			case 1: return y;
			default: break;
		}
		return x;
	}

	/// Write to an indexed element.
	float_t& operator () (index_t i)
	{
		assert((i >= 0) && (i < NumElements));
		switch (i)
		{
			case 0: return x;
			case 1: return y;
			default: break;
		}
		return x;
	}

#endif

	/// Add a vector to this vector.
	constexpr void operator += (const Vec2& v) noexcept
	{
		x += v.x; y += v.y;
	}
	
	/// Subtract a vector from this vector.
	constexpr void operator -= (const Vec2& v) noexcept
	{
		x -= v.x; y -= v.y;
	}

	/// Multiply this vector by a scalar.
	constexpr void operator *= (float_t a) noexcept
	{
		x *= a; y *= a;
	}

	/// Gets the length squared.
	/// For performance, use this instead of Vec2::Length (if possible).
	constexpr float_t LengthSquared() const noexcept
	{
		return b2Square(x) + b2Square(y);
	}

	/// Get the length of this vector (the norm).
	float_t Length() const
	{
		return b2Sqrt(LengthSquared());
	}
	
	/// Convert this vector into a unit vector. Returns the length.
	float_t Normalize()
	{
		const auto length = Length();
		if (length < Epsilon)
		{
			return float_t{0};
		}
		const auto invLength = float_t(1) / length;
		x *= invLength;
		y *= invLength;

		return length;
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b2IsValid(x) && b2IsValid(y);
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	constexpr Vec2 Skew() const noexcept
	{
		return Vec2(-y, x);
	}

	float_t x, y;
};

/// An all zero Vec2 value.
/// @see Vec2.
constexpr auto Vec2_zero = Vec2{0, 0};

/// A 2D column vector with 3 elements.
struct Vec3
{
	/// Default constructor does nothing (for performance).
	Vec3() = default;

	/// Construct using coordinates.
	constexpr explicit Vec3(float_t x_, float_t y_, float_t z_) noexcept : x(x_), y(y_), z(z_) {}

	/// Negate this vector.
	constexpr Vec3 operator -() const noexcept { return Vec3{-x, -y, -z}; }

	/// Add a vector to this vector.
	constexpr void operator += (const Vec3& v) noexcept
	{
		x += v.x; y += v.y; z += v.z;
	}

	/// Subtract a vector from this vector.
	constexpr void operator -= (const Vec3& v) noexcept
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	/// Multiply this vector by a scalar.
	constexpr void operator *= (float_t s) noexcept
	{
		x *= s; y *= s; z *= s;
	}

	float_t x, y, z;
};

/// An all zero Vec3 value.
/// @see Vec3.
constexpr auto Vec3_zero = Vec3{0, 0, 0};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat22(const Vec2& c1, const Vec2& c2) noexcept: ex(c1), ey(c2) {}

	/// Construct this matrix using scalars.
	constexpr b2Mat22(float_t a11, float_t a12, float_t a21, float_t a22) noexcept: ex(a11, a21), ey(a12, a22) {}

	constexpr b2Mat22 GetInverse() const noexcept
	{
		const auto a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		auto det = (a * d) - (b * c);
		if (det != float_t{0})
		{
			det = float_t(1) / det;
		}
		return b2Mat22(Vec2(det * d, -det * c), Vec2(-det * b, det * a));
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr Vec2 Solve(const Vec2& b) const noexcept
	{
		const auto a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		auto det = a11 * a22 - a12 * a21;
		if (det != float_t{0})
		{
			det = float_t(1) / det;
		}
		return Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	}

	Vec2 ex, ey;
};

/// An all zero b2Mat22 value.
/// @see b2Mat22.
constexpr auto b2Mat22_zero = b2Mat22(Vec2_zero, Vec2_zero);

/// Identity value for b2Mat22 objects.
/// @see b2Mat22.
constexpr auto b2Mat22_identity = b2Mat22(Vec2(1, 0), Vec2(0, 1));

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat33(const Vec3& c1, const Vec3& c2, const Vec3& c3):
		ex(c1), ey(c2), ez(c3) {}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr Vec3 Solve33(const Vec3& b) const
	{
		auto det = b2Dot(ex, b2Cross(ey, ez));
		if (det != float_t{0})
		{
			det = float_t(1) / det;
		}
		return Vec3(det * b2Dot(b, b2Cross(ey, ez)), det * b2Dot(ex, b2Cross(b, ez)), det * b2Dot(ex, b2Cross(ey, b)));
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	constexpr Vec2 Solve22(const Vec2& b) const
	{
		const auto a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		auto det = a11 * a22 - a12 * a21;
		if (det != float_t{0})
		{
			det = float_t(1) / det;
		}
		return Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	}

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;

	Vec3 ex, ey, ez;
};

constexpr auto b2Mat33_zero = b2Mat33(Vec3_zero, Vec3_zero, Vec3_zero);

/// Rotation
struct b2Rot
{
	b2Rot() = default;
	
	constexpr b2Rot(const b2Rot& copy) = default;

	/// Initialize from an angle.
	/// @param angle Angle in radians.
	explicit b2Rot(float_t angle): s(std::sin(angle)), c(std::cos(angle))
	{
		// TODO_ERIN optimize
	}
	
	/// Initialize from sine and cosine values.
	constexpr explicit b2Rot(float_t sine, float_t cosine) noexcept: s(sine), c(cosine) {}

	/// Get the angle in radians
	float_t GetAngle() const
	{
		return b2Atan2(s, c);
	}

	/// Get the x-axis
	constexpr Vec2 GetXAxis() const noexcept
	{
		return Vec2(c, s);
	}

	/// Get the u-axis
	constexpr Vec2 GetYAxis() const noexcept
	{
		return Vec2(-s, c);
	}

	/// Sine and cosine
	float_t s, c;
};

constexpr auto b2Rot_identity = b2Rot(0, 1);

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() = default;

	/// Initialize using a position vector and a rotation.
	constexpr b2Transform(const Vec2& position, const b2Rot& rotation) noexcept: p(position), q(rotation) {}

	constexpr b2Transform(const b2Transform& copy) = default;

	Vec2 p;
	b2Rot q;
};

constexpr auto b2Transform_identity = b2Transform{Vec2_zero, b2Rot_identity};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Advances the sweep forward to the given time factor.
	/// This updates c0 and a0 and sets alpha0 to the given time alpha.
	/// @param alpha New time factor in [0,1) to advance the sweep to.
	void Advance(float_t alpha);

	/// Normalize the angles.
	void Normalize();

	Vec2 localCenter;	///< local center of mass position
	Vec2 c0, c;		///< center world positions
	float_t a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float_t alpha0;
};

/// Performs the dot product on two vectors (A and B).
/// @param a Vector A.
/// @param b Vector B.
/// @return Dot product of the vectors.
/// @note If A and B are the same vectors, Vec2::LengthSquared() returns the same value
///   using effectively one less input parameter.
constexpr inline float_t b2Dot(const Vec2& a, const Vec2& b) noexcept
{
	return (a.x * b.x) + (a.y * b.y);
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
constexpr inline float_t b2Cross(const Vec2& a, const Vec2& b) noexcept
{
	return (a.x * b.y) - (a.y * b.x);
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
constexpr inline Vec2 b2Cross(const Vec2& a, float_t s) noexcept
{
	return Vec2{s * a.y, -s * a.x};
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
constexpr inline Vec2 b2Cross(float_t s, const Vec2& a) noexcept
{
	return Vec2{-s * a.y, s * a.x};
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
constexpr inline Vec2 b2Mul(const b2Mat22& A, const Vec2& v) noexcept
{
	return Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline Vec2 b2MulT(const b2Mat22& A, const Vec2& v) noexcept
{
	return Vec2{b2Dot(v, A.ex), b2Dot(v, A.ey)};
}

/// Add two vectors component-wise.
constexpr inline Vec2 operator + (const Vec2& a, const Vec2& b) noexcept
{
	return Vec2{a.x + b.x, a.y + b.y};
}

/// Subtract two vectors component-wise.
constexpr inline Vec2 operator - (const Vec2& a, const Vec2& b) noexcept
{
	return Vec2{a.x - b.x, a.y - b.y};
}

constexpr inline Vec2 operator * (float_t s, const Vec2& a) noexcept
{
	return Vec2{s * a.x, s * a.y};
}

constexpr inline Vec2 operator * (const Vec2& a, float_t s) noexcept
{
	return Vec2{a.x * s, a.y * s};
}

constexpr Vec2 operator/ (const Vec2& a, float_t s) noexcept
{
	return Vec2{a.x / s, a.y / s};
}

/// Normalizes the given value.
/// @param value Value to normalize.
/// @return value divided by its length if length not less than Epsilon otherwise value.
/// @sa Epsilon.
inline Vec2 b2Normalize(const Vec2& value)
{
	// implementation mirrors implementation of Vec2::Normalize()
	const auto length = value.Length();
	if (length < Epsilon)
	{
		return value;
	}
	const auto invLength = float_t(1) / length;
	return value * invLength;
}

constexpr inline bool operator == (const Vec2& a, const Vec2& b) noexcept
{
	return (a.x == b.x) && (a.y == b.y);
}

constexpr inline bool operator != (const Vec2& a, const Vec2& b) noexcept
{
	return (a.x != b.x) || (a.y != b.y);
}

constexpr inline float_t b2DistanceSquared(const Vec2& a, const Vec2& b) noexcept
{
	const auto c = a - b;
	return c.LengthSquared();
}

inline float_t b2Distance(const Vec2& a, const Vec2& b)
{
	return b2Sqrt(b2DistanceSquared(a, b));
}

constexpr inline Vec3 operator * (float_t s, const Vec3& a) noexcept
{
	return Vec3{s * a.x, s * a.y, s * a.z};
}

/// Add two vectors component-wise.
constexpr inline Vec3 operator + (const Vec3& a, const Vec3& b) noexcept
{
	return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

/// Subtract two vectors component-wise.
constexpr inline Vec3 operator - (const Vec3& a, const Vec3& b) noexcept
{
	return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

/// Perform the dot product on two vectors.
constexpr inline float_t b2Dot(const Vec3& a, const Vec3& b) noexcept
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/// Perform the cross product on two vectors.
constexpr inline Vec3 b2Cross(const Vec3& a, const Vec3& b) noexcept
{
	return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

constexpr inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B) noexcept
{
	return b2Mat22{A.ex + B.ex, A.ey + B.ey};
}

// A * B
constexpr inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B) noexcept
{
	return b2Mat22{b2Mul(A, B.ex), b2Mul(A, B.ey)};
}

// A^T * B
constexpr inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B) noexcept
{
	const auto c1 = Vec2(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
	const auto c2 = Vec2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
	return b2Mat22{c1, c2};
}

/// Multiply a matrix times a vector.
constexpr inline Vec3 b2Mul(const b2Mat33& A, const Vec3& v) noexcept
{
	return (v.x * A.ex) + (v.y * A.ey) + (v.z * A.ez);
}

/// Multiply a matrix times a vector.
constexpr inline Vec2 b2Mul22(const b2Mat33& A, const Vec2& v) noexcept
{
	return Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
}

/// Multiply two rotations: q * r
constexpr inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r) noexcept
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	return b2Rot(q.s * r.c + q.c * r.s, q.c * r.c - q.s * r.s);
}

/// Transpose multiply two rotations: qT * r
constexpr inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r) noexcept
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	return b2Rot{q.c * r.s - q.s * r.c, q.c * r.c + q.s * r.s};
}

/// Rotate a vector
constexpr inline Vec2 b2Mul(const b2Rot& q, const Vec2& v) noexcept
{
	return Vec2{q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

/// Inverse rotate a vector
constexpr inline Vec2 b2MulT(const b2Rot& q, const Vec2& v) noexcept
{
	return Vec2{q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

constexpr inline Vec2 b2Mul(const b2Transform& T, const Vec2& v) noexcept
{
	const auto x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	const auto y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
	return Vec2{x, y};
}

constexpr inline Vec2 b2MulT(const b2Transform& T, const Vec2& v) noexcept
{
	const auto px = v.x - T.p.x;
	const auto py = v.y - T.p.y;
	const auto x = (T.q.c * px + T.q.s * py);
	const auto y = (-T.q.s * px + T.q.c * py);
	return Vec2{x, y};
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
constexpr inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B) noexcept
{
	return b2Transform{b2Mul(A.q, B.p) + A.p, b2Mul(A.q, B.q)};
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
constexpr inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B) noexcept
{
	return b2Transform{b2MulT(A.q, B.p - A.p), b2MulT(A.q, B.q)};
}

template <typename T>
constexpr inline T b2Abs(T a)
{
	return (a >= T(0)) ? a : -a;
}

inline Vec2 b2Abs(const Vec2& a)
{
	return Vec2{b2Abs(a.x), b2Abs(a.y)};
}

inline b2Mat22 b2Abs(const b2Mat22& A)
{
	return b2Mat22{b2Abs(A.ex), b2Abs(A.ey)};
}

template <typename T>
constexpr inline T b2Min(T a, T b)
{
	return (a < b) ? a : b;
}

constexpr inline Vec2 b2Min(const Vec2& a, const Vec2& b)
{
	return Vec2{b2Min(a.x, b.x), b2Min(a.y, b.y)};
}

template <typename T>
constexpr inline T b2Max(T a, T b)
{
	return (a > b) ? a : b;
}

constexpr inline Vec2 b2Max(const Vec2& a, const Vec2& b)
{
	return Vec2{b2Max(a.x, b.x), b2Max(a.y, b.y)};
}

template <typename T>
constexpr inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

constexpr inline Vec2 b2Clamp(const Vec2& a, const Vec2& low, const Vec2& high)
{
	return b2Max(low, b2Min(a, high));
}

template<typename T>
constexpr inline void b2Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
constexpr inline uint32 b2NextPowerOfTwo(uint32 x) noexcept
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

constexpr inline bool b2IsPowerOfTwo(uint32 x) noexcept
{
	return (x > 0) && ((x & (x - 1)) == 0);
}

constexpr inline b2Transform b2Displace(const Vec2& ctr, const b2Rot& rot, const Vec2& local_ctr) noexcept
{
	return b2Transform{ctr - b2Mul(rot, local_ctr), rot};
}

/// Gets the interpolated transform at a specific time.
/// @param sweep Sweep data to get the transform from.
/// @param beta Time factor in [0,1], where 0 indicates alpha0.
/// @return Transform of the given sweep at the specified time.
inline b2Transform b2GetTransform(const b2Sweep& sweep, float_t beta)
{
	assert(beta >= 0);
	assert(beta <= 1);
	const auto one_minus_beta = float_t(1) - beta;
	return b2Displace(one_minus_beta * sweep.c0 + beta * sweep.c, b2Rot(one_minus_beta * sweep.a0 + beta * sweep.a), sweep.localCenter);
}

/// Gets the transform at "time" zero.
/// @note This is like calling b2GetTransform(sweep, 0.0), except more efficiently.
/// @sa b2GetTransform(const b2Sweep& sweep, float_t beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transform of the given sweep at time zero.
inline b2Transform b2GetTransformZero(const b2Sweep& sweep)
{
	return b2Displace(sweep.c0, b2Rot(sweep.a0), sweep.localCenter);
}

/// Gets the transform at "time" one.
/// @note This is like calling b2GetTransform(sweep, 1.0), except more efficiently.
/// @sa b2GetTransform(const b2Sweep& sweep, float_t beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transform of the given sweep at time one.
inline b2Transform b2GetTransformOne(const b2Sweep& sweep)
{
	return b2Displace(sweep.c, b2Rot(sweep.a), sweep.localCenter);
}

inline void b2Sweep::Advance(float_t alpha)
{
	assert(alpha < float_t(1));
	assert(alpha0 < float_t(1));
	const auto beta = (alpha - alpha0) / (float_t(1) - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	constexpr auto twoPi = float_t{2} * Pi;
	const auto d =  twoPi * std::floor(a0 / twoPi);
	a0 -= d;
	a -= d;
}

}
#endif
