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
#include <math.h>

// forward declarations
struct b2Vec2;
struct b2Vec3;
constexpr inline float32 b2Dot(const b2Vec2& a, const b2Vec2& b) noexcept;
constexpr inline float32 b2Dot(const b2Vec3& a, const b2Vec3& b) noexcept;
constexpr inline float32 b2Cross(const b2Vec2& a, const b2Vec2& b) noexcept;
constexpr inline b2Vec2 b2Cross(const b2Vec2& a, float32 s) noexcept;
constexpr inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b) noexcept;

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float32 x)
{
	int32 ix = *reinterpret_cast<int32*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

/// This is a approximate yet fast inverse square-root.
inline float32 b2InvSqrt(float32 x)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	const auto xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}

#define	b2Sqrt(x)	sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)

/// A 2D column vector.
struct b2Vec2
{
	/// Default constructor does nothing (for performance).
	b2Vec2() = default;

	/// Construct using coordinates.
	constexpr b2Vec2(float32 x_, float32 y_) noexcept : x(x_), y(y_) {}

	/// Set this vector to all zeros.
	constexpr void SetZero() noexcept { x = 0.0f; y = 0.0f; };

	/// Set this vector to some specified coordinates.
	constexpr void Set(float32 x_, float32 y_) noexcept { x = x_; y = y_; }

	/// Negate this vector.
	constexpr b2Vec2 operator -() const noexcept { return {-x, -y}; }
	
	/// Read from and indexed element.
	float32 operator () (int32 i) const
	{
		return (&x)[i];
	}

	/// Write to an indexed element.
	float32& operator () (int32 i)
	{
		return (&x)[i];
	}

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
	constexpr void operator *= (float32 a) noexcept
	{
		x *= a; y *= a;
	}

	/// Get the length of this vector (the norm).
	float32 Length() const
	{
		return b2Sqrt(x * x + y * y);
	}

	/// Get the length squared. For performance, use this instead of
	/// b2Vec2::Length (if possible).
	constexpr float32 LengthSquared() const noexcept
	{
		return x * x + y * y;
	}

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		const auto length = Length();
		if (length < b2_epsilon)
		{
			return 0.0f;
		}
		const auto invLength = 1.0f / length;
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

	float32 x, y;
};

/// A 2D column vector with 3 elements.
struct b2Vec3
{
	/// Default constructor does nothing (for performance).
	b2Vec3() = default;

	/// Construct using coordinates.
	constexpr b2Vec3(float32 x_, float32 y_, float32 z_) noexcept : x(x_), y(y_), z(z_) {}

	/// Set this vector to all zeros.
	constexpr void SetZero() noexcept { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	constexpr void Set(float32 x_, float32 y_, float32 z_) noexcept { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	constexpr b2Vec3 operator -() const noexcept { return {-x, -y, -z}; }

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
	constexpr void operator *= (float32 s) noexcept
	{
		x *= s; y *= s; z *= s;
	}

	float32 x, y, z;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat22(const b2Vec2& c1, const b2Vec2& c2) noexcept: ex(c1), ey(c2) {}

	/// Construct this matrix using scalars.
	constexpr b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22) noexcept: ex(a11, a21), ey(a12, a22) {}

	/// Initialize this matrix using columns.
	constexpr void Set(const b2Vec2& c1, const b2Vec2& c2) noexcept
	{
		ex = c1;
		ey = c2;
	}

	/// Set this to the identity matrix.
	constexpr void SetIdentity() noexcept
	{
		ex.x = 1.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 1.0f;
	}

	/// Set this matrix to all zeros.
	constexpr void SetZero() noexcept
	{
		ex.x = 0.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 0.0f;
	}

	constexpr b2Mat22 GetInverse() const noexcept
	{
		const auto a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		auto det = a * d - b * c;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		return b2Mat22(b2Vec2(det * d, -det * c), b2Vec2(-det * b, det * a));
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr b2Vec2 Solve(const b2Vec2& b) const noexcept
	{
		const auto a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		auto det = a11 * a22 - a12 * a21;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		return b2Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
	}

	b2Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() = default;

	/// Construct this matrix using columns.
	constexpr b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3):
		ex(c1), ey(c2), ez(c3) {}

	/// Set this matrix to all zeros.
	constexpr void SetZero() noexcept
	{
		ex.SetZero();
		ey.SetZero();
		ez.SetZero();
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	constexpr b2Vec3 Solve33(const b2Vec3& b) const
	{
		auto det = b2Dot(ex, b2Cross(ey, ez));
		if (det != 0.0f)
		{
			det = 1.0f / det;
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
		if (det != 0.0f)
		{
			det = 1.0f / det;
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

/// Rotation
struct b2Rot
{
	b2Rot() = default;

	/// Initialize from an angle in radians
	explicit b2Rot(float32 angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}
	
	constexpr explicit b2Rot(float32 sine, float32 cosine) noexcept: s(sine), c(cosine) {}

	/// Set using an angle in radians.
	void Set(float32 angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}

	/// Set to the identity rotation
	constexpr void SetIdentity() noexcept
	{
		s = 0.0f;
		c = 1.0f;
	}

	/// Get the angle in radians
	float32 GetAngle() const
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
	float32 s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() = default;

	/// Initialize using a position vector and a rotation.
	constexpr b2Transform(const b2Vec2& position, const b2Rot& rotation) noexcept: p(position), q(rotation) {}

	/// Set this to the identity transform.
	constexpr void SetIdentity()
	{
		p.SetZero();
		q.SetIdentity();
	}

	/// Set this based on the position and angle.
	void Set(const b2Vec2& position, float32 angle)
	{
		p = position;
		q.Set(angle);
	}

	b2Vec2 p;
	b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform* xfb, float32 beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);

	/// Normalize the angles.
	void Normalize();

	b2Vec2 localCenter;	///< local center of mass position
	b2Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float32 alpha0;
};

/// Useful constant
constexpr b2Vec2 b2Vec2_zero{0.0f, 0.0f};

/// Perform the dot product on two vectors.
constexpr inline float32 b2Dot(const b2Vec2& a, const b2Vec2& b) noexcept
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
constexpr inline float32 b2Cross(const b2Vec2& a, const b2Vec2& b) noexcept
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
constexpr inline b2Vec2 b2Cross(const b2Vec2& a, float32 s) noexcept
{
	return b2Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
constexpr inline b2Vec2 b2Cross(float32 s, const b2Vec2& a) noexcept
{
	return b2Vec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
constexpr inline b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v) noexcept
{
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v) noexcept
{
	return b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
}

/// Add two vectors component-wise.
constexpr inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return b2Vec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
constexpr inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return b2Vec2(a.x - b.x, a.y - b.y);
}

constexpr inline b2Vec2 operator * (float32 s, const b2Vec2& a) noexcept
{
	return b2Vec2(s * a.x, s * a.y);
}

constexpr inline bool operator == (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return a.x == b.x && a.y == b.y;
}

constexpr inline bool operator != (const b2Vec2& a, const b2Vec2& b) noexcept
{
	return a.x != b.x || a.y != b.y;
}

inline float32 b2Distance(const b2Vec2& a, const b2Vec2& b)
{
	const auto c = a - b;
	return c.Length();
}

constexpr inline float32 b2DistanceSquared(const b2Vec2& a, const b2Vec2& b) noexcept
{
	const auto c = a - b;
	return b2Dot(c, c);
}

constexpr inline b2Vec3 operator * (float32 s, const b2Vec3& a) noexcept
{
	return b2Vec3(s * a.x, s * a.y, s * a.z);
}

/// Add two vectors component-wise.
constexpr inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/// Subtract two vectors component-wise.
constexpr inline b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Perform the dot product on two vectors.
constexpr inline float32 b2Dot(const b2Vec3& a, const b2Vec3& b) noexcept
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
constexpr inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b) noexcept
{
	return b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

constexpr inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B) noexcept
{
	return b2Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
constexpr inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B) noexcept
{
	return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
}

// A^T * B
constexpr inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B) noexcept
{
	const b2Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
	const b2Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
	return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
constexpr inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v) noexcept
{
	return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

/// Multiply a matrix times a vector.
constexpr inline b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v) noexcept
{
	return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
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
	return b2Rot(q.c * r.s - q.s * r.c, q.c * r.c + q.s * r.s);
}

/// Rotate a vector
constexpr inline b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v) noexcept
{
	return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
constexpr inline b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v) noexcept
{
	return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

constexpr inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v) noexcept
{
	const auto x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	const auto y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
	
	return b2Vec2(x, y);
}

constexpr inline b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v) noexcept
{
	const auto px = v.x - T.p.x;
	const auto py = v.y - T.p.y;
	const auto x = (T.q.c * px + T.q.s * py);
	const auto y = (-T.q.s * px + T.q.c * py);

	return b2Vec2(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
constexpr inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B) noexcept
{
	return b2Transform(b2Mul(A.q, B.p) + A.p, b2Mul(A.q, B.q));
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
constexpr inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B) noexcept
{
	return b2Transform(b2MulT(A.q, B.p - A.p), b2MulT(A.q, B.q));
}

template <typename T>
inline T b2Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline b2Vec2 b2Abs(const b2Vec2& a)
{
	return b2Vec2(b2Abs(a.x), b2Abs(a.y));
}

inline b2Mat22 b2Abs(const b2Mat22& A)
{
	return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
}

template <typename T>
constexpr inline T b2Min(T a, T b)
{
	return a < b ? a : b;
}

constexpr inline b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

template <typename T>
constexpr inline T b2Max(T a, T b)
{
	return a > b ? a : b;
}

constexpr inline b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
{
	return b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
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

template<typename T> inline void b2Swap(T& a, T& b)
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

inline void b2Sweep::GetTransform(b2Transform* xf, float32 beta) const
{
	xf->p = (1.0f - beta) * c0 + beta * c;
	const auto angle = (1.0f - beta) * a0 + beta * a;
	xf->q.Set(angle);

	// Shift to origin
	xf->p -= b2Mul(xf->q, localCenter);
}

inline void b2Sweep::Advance(float32 alpha)
{
	b2Assert(alpha0 < 1.0f);
	const auto beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	const auto twoPi = 2.0f * b2_pi;
	const auto d =  twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}

#endif
