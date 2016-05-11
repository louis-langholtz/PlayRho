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

// forward declarations
struct b2Vec2;
struct b2Vec3;
constexpr inline b2Float b2Dot(const b2Vec2& a, const b2Vec2& b) noexcept;
constexpr inline b2Float b2Dot(const b2Vec3& a, const b2Vec3& b) noexcept;
constexpr inline b2Float b2Cross(const b2Vec2& a, const b2Vec2& b) noexcept;
constexpr inline b2Vec2 b2Cross(const b2Vec2& a, b2Float s) noexcept;
constexpr inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b) noexcept;

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(b2Float x)
{
	return !std::isnan(x) && !std::isinf(x);
}

template<class T>
constexpr inline auto b2Square(T t) { return t * t; }

#define	b2Sqrt(x)	std::sqrt(x)
#define	b2Atan2(y, x)	std::atan2(y, x)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() = default;
	
	b2Vec2(const b2Vec2& copy) = default;

	/// Construct using coordinates.
	constexpr b2Vec2(b2Float x_, b2Float y_) noexcept : x(x_), y(y_) {}

	/// Negate this vector.
	constexpr b2Vec2 operator -() const noexcept { return b2Vec2{-x, -y}; }

#if !defined(NO_B2VEC2_INDEXING)

	using index_t = unsigned;
	static constexpr auto NumElements = index_t(2);

	/// Read from and indexed element.
	b2Float operator () (index_t i) const
	{
		b2Assert((i >= 0) && (i < NumElements));
		switch (i)
		{
			case 0: return x;
			case 1: return y;
			default: break;
		}
		return x;
	}

	/// Write to an indexed element.
	b2Float& operator () (index_t i)
	{
		b2Assert((i >= 0) && (i < NumElements));
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
	constexpr void operator += (const b2Vec2& v) noexcept
	{
		x += v.x; y += v.y;
	}
	
	/// Subtract a vector from this vector.
	constexpr void operator -= (const b2Vec2& v) noexcept
	{
		x -= v.x; y -= v.y;
	}

	/// Multiply this vector by a scalar.
	constexpr void operator *= (b2Float a) noexcept
	{
		x *= a; y *= a;
	}

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	constexpr b2Float LengthSquared() const noexcept
	{
		return b2Square(x) + b2Square(y);
	}

	/// Get the length of this vector (the norm).
	b2Float Length() const
	{
		return b2Sqrt(LengthSquared());
	}
	
	/// Convert this vector into a unit vector. Returns the length.
	b2Float Normalize()
	{
		const auto length = Length();
		if (length < b2_epsilon)
		{
			return b2Float{0};
		}
		const auto invLength = b2Float(1) / length;
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
	constexpr b2Vec2 Skew() const noexcept
	{
		return b2Vec2(-y, x);
	}

	b2Float x, y;
};

/// An all zero b2Vec2 value.
/// @see b2Vec2.
constexpr auto b2Vec2_zero = b2Vec2{0, 0};

/// A 2D column vector with 3 elements.
struct b2Vec3
{
	/// Default constructor does nothing (for performance).
	b2Vec3() = default;

	/// Construct using coordinates.
	constexpr explicit b2Vec3(b2Float x_, b2Float y_, b2Float z_) noexcept : x(x_), y(y_), z(z_) {}

	/// Negate this vector.
	constexpr b2Vec3 operator -() const noexcept { return b2Vec3{-x, -y, -z}; }

	/// Add a vector to this vector.
	constexpr void operator += (const b2Vec3& v) noexcept
	{
		x += v.x; y += v.y; z += v.z;
	}

	/// Subtract a vector from this vector.
	constexpr void operator -= (const b2Vec3& v) noexcept
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	/// Multiply this vector by a scalar.
	constexpr void operator *= (b2Float s) noexcept
	{
		x *= s; y *= s; z *= s;
	}

	b2Float x, y, z;
};

/// An all zero b2Vec3 value.
/// @see b2Vec3.
constexpr auto b2Vec3_zero = b2Vec3{0, 0, 0};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat22(const b2Vec2& c1, const b2Vec2& c2) noexcept: ex(c1), ey(c2) {}

	/// Construct this matrix using scalars.
	constexpr b2Mat22(b2Float a11, b2Float a12, b2Float a21, b2Float a22) noexcept: ex(a11, a21), ey(a12, a22) {}

	constexpr b2Mat22 GetInverse() const noexcept
	{
		const auto a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		auto det = (a * d) - (b * c);
		if (det != b2Float{0})
		{
			det = b2Float(1) / det;
		}
		return b2Mat22(b2Vec2(det * d, -det * c), b2Vec2(-det * b, det * a));
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr b2Vec2 Solve(const b2Vec2& b) const noexcept
	{
		const auto a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		auto det = a11 * a22 - a12 * a21;
		if (det != b2Float{0})
		{
			det = b2Float(1) / det;
		}
		return b2Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	}

	b2Vec2 ex, ey;
};

/// An all zero b2Mat22 value.
/// @see b2Mat22.
constexpr auto b2Mat22_zero = b2Mat22(b2Vec2_zero, b2Vec2_zero);

/// Identity value for b2Mat22 objects.
/// @see b2Mat22.
constexpr auto b2Mat22_identity = b2Mat22(b2Vec2(1, 0), b2Vec2(0, 1));

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3):
		ex(c1), ey(c2), ez(c3) {}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr b2Vec3 Solve33(const b2Vec3& b) const
	{
		auto det = b2Dot(ex, b2Cross(ey, ez));
		if (det != b2Float{0})
		{
			det = b2Float(1) / det;
		}
		return b2Vec3(det * b2Dot(b, b2Cross(ey, ez)), det * b2Dot(ex, b2Cross(b, ez)), det * b2Dot(ex, b2Cross(ey, b)));
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	constexpr b2Vec2 Solve22(const b2Vec2& b) const
	{
		const auto a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		auto det = a11 * a22 - a12 * a21;
		if (det != b2Float{0})
		{
			det = b2Float(1) / det;
		}
		return b2Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	}

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;

	b2Vec3 ex, ey, ez;
};

constexpr auto b2Mat33_zero = b2Mat33(b2Vec3_zero, b2Vec3_zero, b2Vec3_zero);

/// Rotation
struct b2Rot
{
	b2Rot() = default;
	b2Rot(const b2Rot& copy) = default;

	/// Initialize from an angle.
	/// @param angle Angle in radians.
	explicit b2Rot(b2Float angle): s(std::sin(angle)), c(std::cos(angle))
	{
		// TODO_ERIN optimize
	}
	
	/// Initialize from sine and cosine values.
	constexpr explicit b2Rot(b2Float sine, b2Float cosine) noexcept: s(sine), c(cosine) {}

	/// Get the angle in radians
	b2Float GetAngle() const
	{
		return b2Atan2(s, c);
	}

	/// Get the x-axis
	constexpr b2Vec2 GetXAxis() const noexcept
	{
		return b2Vec2(c, s);
	}

	/// Get the u-axis
	constexpr b2Vec2 GetYAxis() const noexcept
	{
		return b2Vec2(-s, c);
	}

	/// Sine and cosine
	b2Float s, c;
};

constexpr auto b2Rot_identity = b2Rot(0, 1);

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() = default;

	/// Initialize using a position vector and a rotation.
	constexpr b2Transform(const b2Vec2& position, const b2Rot& rotation) noexcept: p(position), q(rotation) {}

	b2Vec2 p;
	b2Rot q;
};

constexpr auto b2Transform_identity = b2Transform{b2Vec2_zero, b2Rot_identity};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta Time factor in [0,1], where 0 indicates alpha0.
	b2Transform GetTransform(b2Float beta) const;

	/// Advances the sweep forward to the given time factor.
	/// This updates c0 and a0 and sets alpha0 to the given time alpha.
	/// @param alpha New time factor in [0,1) to advance the sweep to.
	void Advance(b2Float alpha);

	/// Normalize the angles.
	void Normalize();

	b2Vec2 localCenter;	///< local center of mass position
	b2Vec2 c0, c;		///< center world positions
	b2Float a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	b2Float alpha0;
};

/// Perform the dot product on two vectors.
constexpr inline b2Float b2Dot(const b2Vec2& a, const b2Vec2& b) noexcept
{
	return (a.x * b.x) + (a.y * b.y);
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
constexpr inline b2Float b2Cross(const b2Vec2& a, const b2Vec2& b) noexcept
{
	return (a.x * b.y) - (a.y * b.x);
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
constexpr inline b2Vec2 b2Cross(const b2Vec2& a, b2Float s) noexcept
{
	return b2Vec2{s * a.y, -s * a.x};
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
constexpr inline b2Vec2 b2Cross(b2Float s, const b2Vec2& a) noexcept
{
	return b2Vec2{-s * a.y, s * a.x};
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
constexpr inline b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v) noexcept
{
	return b2Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v) noexcept
{
	return b2Vec2{b2Dot(v, A.ex), b2Dot(v, A.ey)};
}

/// Add two vectors component-wise.
constexpr inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return b2Vec2{a.x + b.x, a.y + b.y};
}

/// Subtract two vectors component-wise.
constexpr inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return b2Vec2{a.x - b.x, a.y - b.y};
}

constexpr inline b2Vec2 operator * (b2Float s, const b2Vec2& a) noexcept
{
	return b2Vec2{s * a.x, s * a.y};
}

constexpr inline b2Vec2 operator * (const b2Vec2& a, b2Float s) noexcept
{
	return b2Vec2{a.x * s, a.y * s};
}

constexpr b2Vec2 operator/ (const b2Vec2& a, b2Float s) noexcept
{
	return b2Vec2{a.x / s, a.y / s};
}

inline b2Vec2 b2Normalize(const b2Vec2& value)
{
	// implementation mirrors implementation of b2Vec2::Normalize()
	const auto length = value.Length();
	if (length < b2_epsilon)
	{
		return value;
	}
	const auto invLength = b2Float(1) / length;
	return value * invLength;
}

constexpr inline bool operator == (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return (a.x == b.x) && (a.y == b.y);
}

constexpr inline bool operator != (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return (a.x != b.x) || (a.y != b.y);
}

inline b2Float b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	const auto c = a - b;
	return c.Length();
}

constexpr inline b2Float b2DistanceSquared(const b2Vec2& a, const b2Vec2& b) noexcept
{
	const auto c = a - b;
	return b2Dot(c, c);
}

constexpr inline b2Vec3 operator * (b2Float s, const b2Vec3& a) noexcept
{
	return b2Vec3{s * a.x, s * a.y, s * a.z};
}

/// Add two vectors component-wise.
constexpr inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

/// Subtract two vectors component-wise.
constexpr inline b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

/// Perform the dot product on two vectors.
constexpr inline b2Float b2Dot(const b2Vec3& a, const b2Vec3& b) noexcept
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/// Perform the cross product on two vectors.
constexpr inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
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
	const auto c1 = b2Vec2(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
	const auto c2 = b2Vec2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
	return b2Mat22{c1, c2};
}

/// Multiply a matrix times a vector.
constexpr inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v) noexcept
{
	return (v.x * A.ex) + (v.y * A.ey) + (v.z * A.ez);
}

/// Multiply a matrix times a vector.
constexpr inline b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v) noexcept
{
	return b2Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
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
constexpr inline b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v) noexcept
{
	return b2Vec2{q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

/// Inverse rotate a vector
constexpr inline b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v) noexcept
{
	return b2Vec2{q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

constexpr inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v) noexcept
{
	const auto x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	const auto y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
	return b2Vec2{x, y};
}

constexpr inline b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v) noexcept
{
	const auto px = v.x - T.p.x;
	const auto py = v.y - T.p.y;
	const auto x = (T.q.c * px + T.q.s * py);
	const auto y = (-T.q.s * px + T.q.c * py);
	return b2Vec2{x, y};
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

inline b2Vec2 b2Abs(const b2Vec2& a)
{
	return b2Vec2{b2Abs(a.x), b2Abs(a.y)};
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

constexpr inline b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2{b2Min(a.x, b.x), b2Min(a.y, b.y)};
}

template <typename T>
constexpr inline T b2Max(T a, T b)
{
	return (a > b) ? a : b;
}

constexpr inline b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2{b2Max(a.x, b.x), b2Max(a.y, b.y)};
}

template <typename T>
constexpr inline T b2Clamp(T a, T low, T high)
{
	return b2Max(low, b2Min(a, high));
}

constexpr inline b2Vec2 b2Clamp(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
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

constexpr inline b2Transform b2Displace(const b2Vec2& ctr, const b2Vec2& local_ctr, const b2Rot& rot) noexcept
{
	return b2Transform{ctr - b2Mul(rot, local_ctr), rot};
}

inline b2Transform b2ComputeTransform(const b2Sweep& sweep)
{
	return b2Displace(sweep.c, sweep.localCenter, b2Rot(sweep.a));
}

inline b2Transform b2Sweep::GetTransform(b2Float beta) const
{
	b2Assert(beta >= 0);
	b2Assert(beta <= 1);
	const auto one_minus_beta = b2Float(1) - beta;
	const auto rot = b2Rot{one_minus_beta * a0 + beta * a};
	const auto pos = (one_minus_beta * c0 + beta * c) - b2Mul(rot, localCenter);
	return b2Transform{pos, rot};
}

inline void b2Sweep::Advance(b2Float alpha)
{
	b2Assert(alpha < b2Float(1));
	b2Assert(alpha0 < b2Float(1));
	const auto beta = (alpha - alpha0) / (b2Float(1) - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	constexpr auto twoPi = b2Float{2} * b2_pi;
	const auto d =  twoPi * std::floor(a0 / twoPi);
	a0 -= d;
	a -= d;
}

#endif
