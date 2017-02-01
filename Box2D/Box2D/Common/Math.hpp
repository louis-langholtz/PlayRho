/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef BOX2D_MATH_H
#define BOX2D_MATH_H

#include <Box2D/Common/Settings.hpp>
#include <Box2D/Common/Span.hpp>
#include <Box2D/Common/Angle.hpp>
#include <Box2D/Common/UnitVec2.hpp>
#include <Box2D/Common/Vec2.hpp>
#include <cmath>
#include <iostream>

namespace box2d
{
// forward declarations
struct Vec3;
constexpr inline RealNum Dot(const Vec2 a, const Vec2 b) noexcept;
constexpr inline RealNum Dot(const Vec3 a, const Vec3 b) noexcept;
constexpr inline RealNum Cross(const Vec2 a, const Vec2 b) noexcept;
constexpr inline Vec3 Cross(const Vec3 a, const Vec3 b) noexcept;
constexpr bool operator == (const Vec2 a, const Vec2 b) noexcept;
constexpr bool operator != (const Vec2 a, const Vec2 b) noexcept;

// Addition GetInvalid and IsValid template specializations.

template <>
constexpr inline Angle GetInvalid() noexcept
{
	return Angle::GetFromRadians(GetInvalid<RealNum>());
}

template <>
inline bool IsValid(const Angle& a) noexcept
{
	return IsValid(a.ToRadians());
}

// Other templates.

template<class T>
constexpr inline auto Square(T t) noexcept { return t * t; }

template<typename T>
inline auto Sqrt(T t) noexcept(noexcept(std::sqrt(t))) { return std::sqrt(t); }

template<typename T>
inline auto Atan2(T y, T x) { return std::atan2(y, x); }

template <typename T>
constexpr inline T Abs(T a)
{
	return (a >= T(0)) ? a : -a;
}

template <>
constexpr inline Angle Abs(Angle a)
{
	return (a >= 0_deg) ? a : -a;
}

inline RealNum Cos(Angle value)
{
	return RealNum{std::cos(value / 1_rad)};
}

inline RealNum Sin(Angle value)
{
	return RealNum{std::sin(value / 1_rad)};
}

template <typename T>
inline T round(T value, unsigned precision = 100000);

template <>
inline float round(float value, uint32_t precision)
{
	const auto factor = float(static_cast<int64_t>(precision));
	return std::round(value * factor) / factor;
}

template <>
inline double round(double value, uint32_t precision)
{
	const auto factor = double(static_cast<int64_t>(precision));
	return std::round(value * factor) / factor;
}

template <>
inline long double round(long double value, uint32_t precision)
{
	using ldouble = long double;
	const auto factor = ldouble(static_cast<int64_t>(precision));
	return std::round(value * factor) / factor;
}

template <>
inline Fixed32 round(Fixed32 value, uint32_t precision)
{
	const auto factor = Fixed32(static_cast<int64_t>(precision));
	return std::round(value * factor) / factor;
}

template <>
inline Fixed64 round(Fixed64 value, uint32_t precision)
{
	const auto factor = Fixed64(static_cast<int64_t>(precision));
	return std::round(value * factor) / factor;
}

/// Gets whether a given value is almost zero.
/// @detail An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(RealNum value)
{
	return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

constexpr inline bool almost_equal(float x, float y, int ulp = 2)
{
	// From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
	//   "the machine epsilon has to be scaled to the magnitude of the values used
	//    and multiplied by the desired precision in ULPs (units in the last place)
	//    unless the result is subnormal".
	// Where "subnormal" means almost zero.
	//
	return (Abs(x - y) < (std::numeric_limits<float>::epsilon() * Abs(x + y) * ulp)) || almost_zero(x - y);
}

constexpr inline bool almost_equal(double x, double y, int ulp = 2)
{
	// From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
	//   "the machine epsilon has to be scaled to the magnitude of the values used
	//    and multiplied by the desired precision in ULPs (units in the last place)
	//    unless the result is subnormal".
	// Where "subnormal" means almost zero.
	//
	return (Abs(x - y) < (std::numeric_limits<double>::epsilon() * Abs(x + y) * ulp)) || almost_zero(x - y);
}

constexpr inline bool almost_equal(long double x, long double y, int ulp = 2)
{
	// From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
	//   "the machine epsilon has to be scaled to the magnitude of the values used
	//    and multiplied by the desired precision in ULPs (units in the last place)
	//    unless the result is subnormal".
	// Where "subnormal" means almost zero.
	//
	return (Abs(x - y) < (std::numeric_limits<long double>::epsilon() * Abs(x + y) * ulp)) || almost_zero(x - y);
}

constexpr inline bool almost_equal(Fixed32 x, Fixed32 y, int ulp = 2)
{
	return Abs(x - y) <= Fixed32{0, static_cast<uint32_t>(ulp)};
}

constexpr inline bool almost_equal(Fixed64 x, Fixed64 y, int ulp = 2)
{
	return Abs(x - y) <= Fixed64{0, static_cast<uint32_t>(ulp)};
}

template <typename T>
inline T Average(Span<const T> span)
{
	assert(span.size() < std::numeric_limits<T>::max());

	auto sum = T{0};
	for (auto&& element: span)
	{
		sum += element;
	}
	return (span.size() > decltype(span.size()){0})? sum / static_cast<T>(span.size()): sum;
}

/// An all zero Vec2 value.
/// @see Vec2.
constexpr auto Vec2_zero = Vec2{0, 0};

template <>
constexpr inline Vec2 GetInvalid() noexcept
{
	return Vec2{GetInvalid<RealNum>(), GetInvalid<RealNum>()};
}

template <>
inline Vec2 round(Vec2 value, uint32 precision)
{
	return Vec2{round(value.x, precision), round(value.y, precision)};
}

/// Gets the angle.
/// @return Anglular value in the range of -Pi to +Pi radians.
inline Angle GetAngle(Vec2 value)
{
	return 1_rad * Atan2(GetY(value), GetX(value));
}

inline Angle GetAngle(UnitVec2 value)
{
	return 1_rad * Atan2(GetY(value), GetX(value));
}

/// A 2D column vector with 3 elements.
/// @note This data structure is 3 times the size of <code>RealNum</code> -
///   i.e. 12-bytes (with 4-byte RealNum).
struct Vec3
{
	/// Default constructor does nothing (for performance).
	Vec3() noexcept = default;

	/// Construct using coordinates.
	constexpr Vec3(RealNum x_, RealNum y_, RealNum z_) noexcept : x(x_), y(y_), z(z_) {}

	/// Negate this vector.
	constexpr auto operator- () const noexcept { return Vec3{-x, -y, -z}; }

	RealNum x, y, z;
};

/// An all zero Vec3 value.
/// @see Vec3.
constexpr auto Vec3_zero = Vec3{0, 0, 0};

template <>
constexpr inline Vec3 GetInvalid() noexcept
{
	return Vec3{GetInvalid<RealNum>(), GetInvalid<RealNum>(), GetInvalid<RealNum>()};
}

/// Gets the square of the length/magnitude of the given value.
/// For performance, use this instead of GetLength(T value) (if possible).
/// @return Non-negative value.
template <typename T>
constexpr inline RealNum GetLengthSquared(T value) noexcept { return RealNum{0}; }

template <>
constexpr inline RealNum GetLengthSquared(Vec2 value) noexcept
{
	return Square(value.x) + Square(value.y);		
}

template <>
constexpr inline RealNum GetLengthSquared(Vec3 value) noexcept
{
	return Square(value.x) + Square(value.y) + Square(value.z);		
}

template <typename T>
inline RealNum GetLength(T value)
{
	return Sqrt(GetLengthSquared(value));
}

/// Does this vector contain finite coordinates?
template <>
inline bool IsValid(const Vec2& value) noexcept
{
	return IsValid(value.x) && IsValid(value.y);
}

/// Does this vector contain finite coordinates?
template <>
inline bool IsValid(const Vec3& value) noexcept
{
	return IsValid(value.x) && IsValid(value.y) && IsValid(value.z);
}

/// A 2-by-2 matrix.
/// @detail Stored in column-major order.
/// @note This structure is likely about 16-bytes large.
struct Mat22
{
	/// The default constructor does nothing (for performance).
	Mat22() noexcept = default;

	/// Construct this matrix using columns.
	constexpr Mat22(const Vec2 c1, const Vec2 c2) noexcept: ex{c1}, ey{c2} {}

	/// Construct this matrix using scalars.
	constexpr Mat22(RealNum a11, RealNum a12, RealNum a21, RealNum a22) noexcept: ex{a11, a21}, ey{a12, a22} {}

	Vec2 ex, ey;
};

template <>
inline bool IsValid(const Mat22& value) noexcept
{
	return IsValid(value.ex) && IsValid(value.ey);
}

/// An all zero Mat22 value.
/// @see Mat22.
constexpr auto Mat22_zero = Mat22(Vec2_zero, Vec2_zero);

template <>
constexpr inline Mat22 GetInvalid() noexcept
{
	return Mat22{GetInvalid<Vec2>(), GetInvalid<Vec2>()};
}

/// Identity value for Mat22 objects.
/// @see Mat22.
constexpr auto Mat22_identity = Mat22(Vec2{1, 0}, Vec2{0, 1});

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
constexpr Vec2 Solve(const Mat22 mat, const Vec2 b) noexcept
{
	const auto cp = Cross(mat.ex, mat.ey);
	return (cp != 0)?
		Vec2{(mat.ey.y * b.x - mat.ey.x * b.y) / cp, (mat.ex.x * b.y - mat.ex.y * b.x) / cp}:
		Vec2{0, 0};
}

constexpr Mat22 Invert(const Mat22 value) noexcept
{
	const auto cp = Cross(value.ex, value.ey);
	return (cp != 0)?
		Mat22{Vec2{value.ey.y / cp, -value.ex.y / cp}, Vec2{-value.ey.x / cp, value.ex.x / cp}}:
		Mat22{Vec2{0, 0}, Vec2{0, 0}};
}

/// A 3-by-3 matrix. Stored in column-major order.
/// @note This data structure is 36-bytes large (on at least one 64-bit platform with 4-byte RealNum).
struct Mat33
{
	/// The default constructor does nothing (for performance).
	Mat33() noexcept = default;

	/// Construct this matrix using columns.
	constexpr Mat33(const Vec3 c1, const Vec3 c2, const Vec3 c3) noexcept:
		ex{c1}, ey{c2}, ez{c3} {}

	Vec3 ex, ey, ez;
};

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
constexpr Vec3 Solve33(const Mat33& mat, const Vec3 b) noexcept
{
	const auto dp = Dot(mat.ex, Cross(mat.ey, mat.ez));
	const auto det = (dp != 0)? 1 / dp: dp;
	const auto x = det * Dot(b, Cross(mat.ey, mat.ez));
	const auto y = det * Dot(mat.ex, Cross(b, mat.ez));
	const auto z = det * Dot(mat.ex, Cross(mat.ey, b));
	return Vec3{x, y, z};
}
	
/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases. Solve only the upper
/// 2-by-2 matrix equation.
constexpr Vec2 Solve22(const Mat33& mat, const Vec2 b) noexcept
{
	const auto cp = mat.ex.x * mat.ey.y - mat.ey.x * mat.ex.y;
	const auto det = (cp != 0)? 1 / cp: cp;
	const auto x = det * (mat.ey.y * b.x - mat.ey.x * b.y);
	const auto y = det * (mat.ex.x * b.y - mat.ex.y * b.x);
	return Vec2{x, y};
}

constexpr auto Mat33_zero = Mat33(Vec3_zero, Vec3_zero, Vec3_zero);

/// Get the inverse of this matrix as a 2-by-2.
/// Returns the zero matrix if singular.
constexpr inline Mat33 GetInverse22(const Mat33& value) noexcept
{
	const auto a = value.ex.x, b = value.ey.x, c = value.ex.y, d = value.ey.y;
	auto det = (a * d) - (b * c);
	if (det != RealNum{0})
	{
		det = RealNum{1} / det;
	}
	return Mat33{Vec3{det * d, -det * c, RealNum{0}}, Vec3{-det * b, det * a, 0}, Vec3{0, 0, 0}};
}
	
/// Get the symmetric inverse of this matrix as a 3-by-3.
/// Returns the zero matrix if singular.
constexpr inline Mat33 GetSymInverse33(const Mat33& value) noexcept
{
	auto det = Dot(value.ex, Cross(value.ey, value.ez));
	if (det != RealNum{0})
	{
		det = RealNum{1} / det;
	}
	
	const auto a11 = value.ex.x, a12 = value.ey.x, a13 = value.ez.x;
	const auto a22 = value.ey.y, a23 = value.ez.y;
	const auto a33 = value.ez.z;
	
	const auto ex_y = det * (a13 * a23 - a12 * a33);
	const auto ey_z = det * (a13 * a12 - a11 * a23);
	const auto ex_z = det * (a12 * a23 - a13 * a22);
	
	return Mat33{
		Vec3{det * (a22 * a33 - a23 * a23), ex_y, ex_z},
		Vec3{ex_y, det * (a11 * a33 - a13 * a13), ey_z},
		Vec3{ex_z, ey_z, det * (a11 * a22 - a12 * a12)}
	};
}

template <>
constexpr UnitVec2 GetInvalid() noexcept
{
	return UnitVec2{};
}

template <>
inline bool IsValid(const UnitVec2& value) noexcept
{
	return IsValid(value.GetX()) && IsValid(value.GetY()) && (value != UnitVec2::GetZero());
}

/// Transformation.
/// @detail
/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
/// @note This data structure is 16-bytes large (on at least one 64-bit platform).
struct Transformation
{
	/// The default constructor does nothing.
	Transformation() noexcept = default;

	/// Initialize using a translation and a rotation.
	constexpr Transformation(Vec2 translation, UnitVec2 rotation) noexcept: p{translation}, q{rotation} {}

	constexpr Transformation(const Transformation& copy) noexcept = default;

	Vec2 p; ///< Translational portion of the transformation. 8-bytes.
	UnitVec2 q; ///< Rotational portion of the transformation. 8-bytes.
};

constexpr auto Transform_identity = Transformation{Vec2_zero, UnitVec2::GetRight()};

template <>
inline bool IsValid(const Transformation& value) noexcept
{
	return IsValid(value.p) && IsValid(value.q);
}

/// Positional data structure.
/// @note This structure is likely to be 12-bytes large (at least on 64-bit platforms).
struct Position
{
	Position() noexcept = default;
	
	constexpr Position(const Position& copy) noexcept = default;
	
	/// Initializing constructor.
	/// @param c_ Linear position.
	/// @param a_ Angular position.
	constexpr Position(Vec2 c_, Angle a_) noexcept: linear{c_}, angular{a_} {}
	
	Vec2 linear; ///< Linear position (in meters).
	Angle angular; ///< Angular position (in radians).
};

template <>
inline bool IsValid(const Position& value) noexcept
{
	return IsValid(value.linear) && IsValid(value.angular);
}

/// Velocity related data structure.
struct Velocity
{
	Velocity() = default;
	
	constexpr Velocity(const Velocity& copy) = default;

	constexpr Velocity(Vec2 v_, Angle w_) noexcept: linear{v_}, angular{w_} {}
	
	Velocity& operator= (const Velocity& rhs) noexcept
	{
		linear = rhs.linear;
		angular = rhs.angular;
		return *this;
	}

	Vec2 linear; ///< Linear velocity (in meters/second).
	Angle angular; ///< Angular velocity (in radians/second).
};

template <>
inline bool IsValid(const Velocity& value) noexcept
{
	return IsValid(value.linear) && IsValid(value.angular);
}

/// Sweep.
/// @detail
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
/// @note This data structure is likely 36-bytes (at least on 64-bit platforms).
class Sweep
{
public:
	/// Default constructor.
	Sweep() = default;

	/// Copy constructor.
	constexpr Sweep(const Sweep& copy) noexcept = default;

	/// Initializing constructor.
	constexpr Sweep(const Position& p0, const Position& p1, const Vec2 lc = Vec2_zero, RealNum a0 = 0) noexcept:
		pos0{p0}, pos1{p1}, localCenter{lc}, alpha0{a0}
	{
		assert(a0 >= 0);
		assert(a0 < 1);
	}
	
	/// Initializing constructor.
	constexpr explicit Sweep(const Position& p, const Vec2 lc = Vec2_zero) noexcept: Sweep{p, p, lc, 0} {}

	/// Gets the local center of mass position.
 	/// @note This value can only be set via a sweep constructed using an initializing constructor.
	Vec2 GetLocalCenter() const noexcept { return localCenter; }

	/// Gets the alpha0 for this sweep.
	/// @return Value between 0 and less than 1.
	RealNum GetAlpha0() const noexcept { return alpha0; }

	/// Advances the sweep by a factor of the difference between the given time alpha and the sweep's alpha0.
	/// @detail
	/// This advances position 0 (<code>pos0</code>) of the sweep towards position 1 (<code>pos1</code>)
	/// by a factor of the difference between the given alpha and the alpha0.
	/// @param alpha Valid new time factor in [0,1) to update the sweep to. Behavior is undefined if value is invalid.
	void Advance0(RealNum alpha);

	void ResetAlpha0() noexcept;

	Position pos0; ///< Center world position and world angle at time "0". 12-bytes.
	Position pos1; ///< Center world position and world angle at time "1". 12-bytes.

private:
	Vec2 localCenter; ///< Local center of mass position. 8-bytes.

	/// Fraction of the current time step in the range [0,1]
	/// pos0.linear and pos0.angular are the positions at alpha0.
	/// @note 4-bytes.
	RealNum alpha0;
};

/// Gets a vector counter-clockwise (reverse-clockwise) perpendicular to the given vector.
/// @detail This takes a vector of form (x, y) and returns the vector (-y, x).
/// @param vector Vector to return a counter-clockwise perpendicular equivalent for.
/// @return A counter-clockwise 90-degree rotation of the given vector.
/// @sa GetFwdPerpendicular.
constexpr inline Vec2 GetRevPerpendicular(const Vec2 vector) noexcept
{
	// See http://mathworld.wolfram.com/PerpendicularVector.html
	return Vec2{-vector.y, vector.x};
}
	
/// Gets a vector clockwise (forward-clockwise) perpendicular to the given vector.
/// @detail This takes a vector of form (x, y) and returns the vector (y, -x).
/// @param vector Vector to return a clockwise perpendicular equivalent for.
/// @return A clockwise 90-degree rotation of the given vector.
/// @sa GetRevPerpendicular.
constexpr inline Vec2 GetFwdPerpendicular(const Vec2 vector) noexcept
{
	// See http://mathworld.wolfram.com/PerpendicularVector.html
	return Vec2{vector.y, -vector.x};
}

/// Performs the dot product on two vectors (A and B).
///
/// @detail The dot product of two vectors is defined as:
///   the magnitude of vector A, mulitiplied by, the magnitude of vector B,
///   multiplied by, the cosine of the angle between the two vectors (A and B).
///   Thus the dot product of two vectors is a value ranging between plus and minus the
///   magnitudes of each vector times each other.
///   The middle value of 0 indicates that two vectors are perpendicular to each other
///   (at an angle of +/- 90 degrees from each other).
///   
///
/// @note This operation is commutative. I.e. Dot(a, b) == Dot(b, a).
/// @note If A and B are the same vectors, GetLengthSquared(Vec2) returns the same value
///   using effectively one less input parameter.
///
/// @sa https://en.wikipedia.org/wiki/Dot_product
///
/// @param a Vector A.
/// @param b Vector B.
///
/// @return Dot product of the vectors (0 means the two vectors are perpendicular).
///
constexpr inline RealNum Dot(const Vec2 a, const Vec2 b) noexcept
{
	return (GetX(a) * GetX(b)) + (GetY(a) * GetY(b));
}

/// Performs the 2D analog of the cross product of two vectors.
///
/// @detail
/// This is defined as the result of: <code>(a.x * b.y) - (a.y * b.x)</code>.
///
/// @note This operation is anti-commutative. I.e. Cross(a, b) == -Cross(b, a).
/// @note The result will be 0 if any of the following are true:
///   vector A or vector B has a length of zero;
///   vectors A and B point in the same direction; or
///   vectors A and B point in exactly opposite direction of each other.
/// @note The result will be positive if:
///   neither vector A nor B has a length of zero; and
///   vector B is at an angle from vector A of greater than 0 and less than 180 degrees
///   (counter-clockwise from A being a positive angle).
/// @note Result will be negative if:
///   neither vector A nor B has a length of zero; and
///   vector B is at an angle from vector A of less than 0 and greater than -180 degrees
///   (clockwise from A being a negative angle).
/// @note The absolute value of the result is the area of the parallelogram formed by
///   the vectors A and B.
///
/// @sa https://en.wikipedia.org/wiki/Cross_product
///
/// @param a Vector A.
/// @param b Vector B.
///
/// @return Cross product of the two vectors.
///
constexpr inline RealNum Cross(const Vec2 a, const Vec2 b) noexcept
{
	// Both vectors of same direction...
	// If a = Vec2{1, 2} and b = Vec2{1, 2} then: a x b = 1 * 2 - 2 * 1 = 0.
	// If a = Vec2{1, 2} and b = Vec2{2, 4} then: a x b = 1 * 4 - 2 * 2 = 0.
	//
	// Vectors at +/- 90 degrees of each other...
	// If a = Vec2{1, 2} and b = Vec2{-2, 1} then: a x b = 1 * 1 - 2 * (-2) = 1 + 4 = 5.
	// If a = Vec2{1, 2} and b = Vec2{2, -1} then: a x b = 1 * (-1) - 2 * 2 = -1 - 4 = -5.
	//
	// Vectors between 0 and 180 degrees of each other excluding 90 degrees...
	// If a = Vec2{1, 2} and b = Vec2{-1, 2} then: a x b = 1 * 2 - 2 * (-1) = 2 + 2 = 4.
	return (GetX(a) * GetY(b)) - (GetY(a) * GetX(b));
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
constexpr inline Vec2 Transform(const Vec2 v, const Mat22& A) noexcept
{
	return Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline Vec2 InverseTransform(const Vec2 v, const Mat22& A) noexcept
{
	return Vec2{Dot(v, A.ex), Dot(v, A.ey)};
}

/// Increment the left hand side value by the right hand side value.
constexpr Vec2& operator += (Vec2& lhs, Vec2 rhs) noexcept
{
	lhs.x += rhs.x;
	lhs.y += rhs.y;
	return lhs;
}

/// Decrement the left hand side value by the right hand side value.
constexpr Vec2& operator -= (Vec2& lhs, Vec2 rhs) noexcept
{
	lhs.x -= rhs.x;
	lhs.y -= rhs.y;
	return lhs;
}

constexpr Vec2& operator *= (Vec2& lhs, Vec2::data_type rhs) noexcept
{
	lhs.x *= rhs;
	lhs.y *= rhs;
	return lhs;
}

/// Add two vectors component-wise.
constexpr inline Vec2 operator + (const Vec2 a, const Vec2 b) noexcept
{
	return Vec2{a.x + b.x, a.y + b.y};
}

/// Subtract two vectors component-wise.
constexpr inline Vec2 operator - (const Vec2 a, const Vec2 b) noexcept
{
	return Vec2{a.x - b.x, a.y - b.y};
}

constexpr inline Vec2 operator * (Vec2::data_type s, const Vec2 a) noexcept
{
	return Vec2{s * a.x, s * a.y};
}

constexpr inline Vec2 operator * (const Vec2 a, const Vec2::data_type s) noexcept
{
	return Vec2{a.x * s, a.y * s};
}

constexpr Vec2 operator/ (const Vec2 a, const Vec2::data_type s) noexcept
{
	return Vec2{a.x / s, a.y / s};
}

constexpr inline bool operator == (const Vec2 a, const Vec2 b) noexcept
{
	return (a.x == b.x) && (a.y == b.y);
}

constexpr inline bool operator != (const Vec2 a, const Vec2 b) noexcept
{
	return (a.x != b.x) || (a.y != b.y);
}

constexpr inline RealNum Dot(const UnitVec2 a, const UnitVec2 b) noexcept
{
	return (GetX(a) * GetX(b)) + (GetY(a) * GetY(b));
}

constexpr inline RealNum Dot(const Vec2 a, const UnitVec2 b) noexcept
{
	return (GetX(a) * GetX(b)) + (GetY(a) * GetY(b));
}

constexpr inline RealNum Dot(const UnitVec2 a, const Vec2 b) noexcept
{
	return (GetX(a) * GetX(b)) + (GetY(a) * GetY(b));
}

constexpr inline RealNum Cross(const UnitVec2 a, const UnitVec2 b) noexcept
{
	return (GetX(a) * GetY(b)) - (GetY(a) * GetX(b));
}

constexpr inline RealNum Cross(const UnitVec2 a, const Vec2 b) noexcept
{
	return (GetX(a) * GetY(b)) - (GetY(a) * GetX(b));
}

constexpr inline RealNum Cross(const Vec2 a, const UnitVec2 b) noexcept
{
	return (GetX(a) * GetY(b)) - (GetY(a) * GetX(b));
}

constexpr inline Vec2 operator+ (const UnitVec2 lhs, const UnitVec2 rhs) noexcept
{
	return Vec2{lhs.GetX() + rhs.GetX(), lhs.GetY() + rhs.GetY()};
}

constexpr inline Vec2 operator- (const UnitVec2 lhs, const UnitVec2 rhs) noexcept
{
	return Vec2{lhs.GetX() - rhs.GetX(), lhs.GetY() - rhs.GetY()};
}

constexpr inline Vec2 operator* (const UnitVec2::data_type s, const UnitVec2 u) noexcept
{
	return Vec2{u.GetX() * s, u.GetY() * s};
}

constexpr inline Vec2 operator* (const UnitVec2 u, const UnitVec2::data_type s) noexcept
{
	return Vec2{u.GetX() * s, u.GetY() * s};
}

constexpr inline Vec2 operator/ (const UnitVec2 u, const UnitVec2::data_type s) noexcept
{
	return Vec2{u.GetX() / s, u.GetY() / s};
}

constexpr inline bool operator == (const Vec3 lhs, const Vec3 rhs) noexcept
{
	return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

constexpr inline bool operator != (const Vec3 lhs, const Vec3 rhs) noexcept
{
	return (lhs.x != rhs.x) || (lhs.y != rhs.y) || (lhs.z != rhs.z);
}

constexpr inline bool operator == (Transformation lhs, Transformation rhs) noexcept
{
	return (lhs.p == rhs.p) && (lhs.q == rhs.q);
}

constexpr inline bool operator != (Transformation lhs, Transformation rhs) noexcept
{
	return (lhs.p != rhs.p) || (lhs.q != rhs.q);
}

constexpr Vec3& operator += (Vec3& lhs, const Vec3& rhs) noexcept
{
	lhs.x += rhs.x;
	lhs.y += rhs.y;
	lhs.z += rhs.z;
	return lhs;
}

constexpr Vec3& operator -= (Vec3& lhs, const Vec3& rhs) noexcept
{
	lhs.x -= rhs.x;
	lhs.y -= rhs.y;
	lhs.z -= rhs.z;
	return lhs;
}

constexpr Vec3& operator *= (Vec3& lhs, const RealNum rhs) noexcept
{
	lhs.x *= rhs;
	lhs.y *= rhs;
	lhs.z *= rhs;
	return lhs;
}

constexpr inline Vec3 operator * (const RealNum s, const Vec3 a) noexcept
{
	return Vec3{s * a.x, s * a.y, s * a.z};
}

/// Add two vectors component-wise.
constexpr inline Vec3 operator + (const Vec3 a, const Vec3 b) noexcept
{
	return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

/// Subtract two vectors component-wise.
constexpr inline Vec3 operator - (const Vec3 a, const Vec3 b) noexcept
{
	return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

/// Perform the dot product on two vectors.
constexpr inline RealNum Dot(const Vec3 a, const Vec3 b) noexcept
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/// Perform the cross product on two vectors.
constexpr inline Vec3 Cross(const Vec3 a, const Vec3 b) noexcept
{
	return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

constexpr inline Mat22 operator + (const Mat22 A, const Mat22 B) noexcept
{
	return Mat22{A.ex + B.ex, A.ey + B.ey};
}

// A * B
constexpr inline Mat22 Mul(const Mat22& A, const Mat22& B) noexcept
{
	return Mat22{Transform(B.ex, A), Transform(B.ey, A)};
}

// A^T * B
constexpr inline Mat22 MulT(const Mat22& A, const Mat22& B) noexcept
{
	const auto c1 = Vec2{Dot(A.ex, B.ex), Dot(A.ey, B.ex)};
	const auto c2 = Vec2{Dot(A.ex, B.ey), Dot(A.ey, B.ey)};
	return Mat22{c1, c2};
}

/// Multiply a matrix times a vector.
constexpr inline Vec3 Transform(const Vec3& v, const Mat33& A) noexcept
{
	return (v.x * A.ex) + (v.y * A.ey) + (v.z * A.ez);
}

/// Multiply a matrix times a vector.
constexpr inline Vec2 Transform(const Vec2 v, const Mat33& A) noexcept
{
	return Vec2{A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y};
}

/// Rotates a vector by a given angle.
constexpr inline Vec2 Rotate(const Vec2 vector, const UnitVec2& angle) noexcept
{
	return Vec2{(angle.cos() * vector.x) - (angle.sin() * vector.y), (angle.sin() * vector.x) + (angle.cos() * vector.y)};
}

/// Inverse rotate a vector
constexpr inline Vec2 InverseRotate(const Vec2 vector, const UnitVec2& angle) noexcept
{
	return Vec2{(angle.cos() * vector.x) + (angle.sin() * vector.y), (angle.cos() * vector.y) - (angle.sin() * vector.x)};
}

/// Transforms the given 2-D vector with the given transformation.
/// @detail
/// Rotate and translate the given 2-D linear position according to the rotation and translation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>InverseTransform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @note For a 2-D linear position of the origin (0, 0), the result is simply the translation.
/// @sa <code>InverseTransform</code>.
/// @param v 2-D position to transform (to rotate and then translate).
/// @param T Transformation (a translation and rotation) to apply to the given vector.
/// @return Rotated and translated vector.
constexpr inline Vec2 Transform(const Vec2 v, const Transformation T) noexcept
{
	return Rotate(v, T.q) + T.p;
}

/// Inverse transforms the given 2-D vector with the given transformation.
/// @detail
/// Inverse translate and rotate the given 2-D vector according to the translation and rotation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>Transform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @sa <code>Transform</code>.
/// @param v 2-D vector to inverse transform (inverse translate and inverse rotate).
/// @param T Transformation (a translation and rotation) to invertedly apply to the given vector.
/// @return Inverse transformed vector.
constexpr inline Vec2 InverseTransform(const Vec2 v, const Transformation T) noexcept
{
	return InverseRotate(v - T.p, T.q);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
constexpr inline Transformation Mul(const Transformation& A, const Transformation& B) noexcept
{
	return Transformation{A.p + Rotate(B.p, A.q), A.q.Rotate(B.q)};
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
constexpr inline Transformation MulT(const Transformation& A, const Transformation& B) noexcept
{
	return Transformation{InverseRotate(B.p - A.p, A.q), B.q.Rotate(A.q.FlipY())};
}

template <>
inline Vec2 Abs(Vec2 a)
{
	return Vec2{Abs(a.x), Abs(a.y)};
}

template <>
inline UnitVec2 Abs(UnitVec2 a)
{
	return a.Absolute();
}

inline Mat22 Abs(const Mat22& A)
{
	return Mat22{Abs(A.ex), Abs(A.ey)};
}

template <typename T>
constexpr inline T Min(T a, T b)
{
	return (a < b) ? a : b;
}

template <>
constexpr inline Vec2 Min(Vec2 a, Vec2 b)
{
	return Vec2{Min(a.x, b.x), Min(a.y, b.y)};
}

template <typename T>
constexpr inline T Max(T a, T b)
{
	return (a > b) ? a : b;
}

template <>
constexpr inline Vec2 Max(Vec2 a, Vec2 b)
{
	return Vec2{Max(a.x, b.x), Max(a.y, b.y)};
}

/// Clamps the given value within the given range (inclusive).
/// @param a Value to clamp.
/// @param low Lowest value to return.
/// @param high Highest value to return.
template <typename T>
constexpr inline T Clamp(T a, T low, T high)
{
	return Max(low, Min(a, high));
}

template<typename T>
constexpr inline void Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 64-bit value:"
inline uint64 NextPowerOfTwo(uint64 x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	x |= (x >> 32);
	return x + 1;
}

constexpr inline bool operator==(const Position& lhs, const Position& rhs)
{
	return (lhs.linear == rhs.linear) && (lhs.angular == rhs.angular);
}

constexpr inline bool operator!=(const Position& lhs, const Position& rhs)
{
	return (lhs.linear != rhs.linear) || (lhs.angular != rhs.angular);
}

constexpr inline Position operator- (const Position& value)
{
	return Position{-value.linear, -value.angular};
}

constexpr inline Position operator+ (const Position& value)
{
	return value;
}

constexpr inline Position& operator+= (Position& lhs, const Position& rhs)
{
	lhs.linear += rhs.linear;
	lhs.angular += rhs.angular;
	return lhs;
}

constexpr inline Position operator+ (const Position& lhs, const Position& rhs)
{
	return Position{lhs.linear + rhs.linear, lhs.angular + rhs.angular};
}
	
constexpr inline Position& operator-= (Position& lhs, const Position& rhs)
{
	lhs.linear -= rhs.linear;
	lhs.angular -= rhs.angular;
	return lhs;
}

constexpr inline Position operator- (const Position& lhs, const Position& rhs)
{
	return Position{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
}

constexpr inline Position operator* (const Position& pos, const RealNum scalar)
{
	return Position{pos.linear * scalar, pos.angular * scalar};
}

constexpr inline Position operator* (const RealNum scalar, const Position& pos)
{
	return Position{pos.linear * scalar, pos.angular * scalar};
}
	
constexpr inline bool operator==(const Velocity& lhs, const Velocity& rhs)
{
	return (lhs.linear == rhs.linear) && (lhs.angular == rhs.angular);
}

constexpr inline bool operator!=(const Velocity& lhs, const Velocity& rhs)
{
	return (lhs.linear != rhs.linear) || (lhs.angular != rhs.angular);
}

constexpr inline Velocity& operator+= (Velocity& lhs, const Velocity& rhs)
{
	lhs.linear += rhs.linear;
	lhs.angular += rhs.angular;
	return lhs;
}

constexpr inline Velocity operator+ (const Velocity& lhs, const Velocity& rhs)
{
	return Velocity{lhs.linear + rhs.linear, lhs.angular + rhs.angular};
}

constexpr inline Velocity& operator-= (Velocity& lhs, const Velocity& rhs)
{
	lhs.linear -= rhs.linear;
	lhs.angular -= rhs.angular;
	return lhs;
}

constexpr inline Velocity operator- (const Velocity& lhs, const Velocity& rhs)
{
	return Velocity{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
}

constexpr inline Velocity operator- (const Velocity& value)
{
	return Velocity{-value.linear, -value.angular};
}

constexpr inline Velocity operator+ (const Velocity& value)
{
	return value;
}

constexpr inline Velocity operator* (const Velocity& lhs, const RealNum rhs)
{
	return Velocity{lhs.linear * rhs, lhs.angular * rhs};
}

constexpr inline Velocity operator* (const RealNum lhs, const Velocity& rhs)
{
	return Velocity{rhs.linear * lhs, rhs.angular * lhs};
}

constexpr inline Transformation GetTransformation(const Vec2 ctr, const UnitVec2 rot, const Vec2 local_ctr) noexcept
{
	return Transformation{ctr - Rotate(local_ctr, rot), rot};
}

inline Transformation GetTransformation(const Position pos, const Vec2 local_ctr) noexcept
{
	assert(IsValid(pos));
	assert(IsValid(local_ctr));
	return GetTransformation(pos.linear, UnitVec2{pos.angular}, local_ctr);
}

inline Position GetPosition(const Position pos0, const Position pos1, const RealNum beta)
{
	return pos0 * (RealNum{1} - beta) + pos1 * beta;
}

/// Gets the interpolated transform at a specific time.
/// @param sweep Sweep data to get the transform from.
/// @param beta Time factor in [0,1], where 0 indicates alpha0.
/// @return Transformation of the given sweep at the specified time.
inline Transformation GetTransformation(const Sweep& sweep, const RealNum beta)
{
	assert(beta >= 0);
	assert(beta <= 1);
	return GetTransformation(GetPosition(sweep.pos0, sweep.pos1, beta), sweep.GetLocalCenter());
}

/// Gets the transform at "time" zero.
/// @note This is like calling GetTransformation(sweep, 0.0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, RealNum beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time zero.
inline Transformation GetTransform0(const Sweep& sweep)
{
	return GetTransformation(sweep.pos0, sweep.GetLocalCenter());
}

/// Gets the transform at "time" one.
/// @note This is like calling GetTransformation(sweep, 1.0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, RealNum beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time one.
inline Transformation GetTransform1(const Sweep& sweep)
{
	return GetTransformation(sweep.pos1, sweep.GetLocalCenter());
}

inline void Sweep::Advance0(const RealNum alpha)
{
	assert(IsValid(alpha));
	assert(alpha >= 0);
	assert(alpha < 1);
	assert(alpha0 < 1);
	
	const auto beta = (alpha - alpha0) / (RealNum{1} - alpha0);
	pos0 = GetPosition(pos0, pos1, beta);
	alpha0 = alpha;
}

inline void Sweep::ResetAlpha0() noexcept
{
	alpha0 = RealNum{0};
}

/// Gets a sweep with the given sweep's angles normalized.
/// @param sweep Sweep to return with its angles normalized.
/// @return Sweep with its pos0 angle in radians to be between -2 pi and 2 pi
///    and its pos1 angle reduced by the amount pos0's angle was reduced by.
inline Sweep GetAnglesNormalized(Sweep sweep)
{
	const auto pos0a = GetNormalized(sweep.pos0.angular);
	const auto d = sweep.pos0.angular - pos0a;
	sweep.pos0.angular = pos0a;
	sweep.pos1.angular -= d;
	return sweep;
}

/// Converts the given vector into a unit vector and returns its original length.
inline RealNum Normalize(Vec2& vector)
{
	const auto length = GetLength(vector);
	if (almost_zero(length))
	{
		return RealNum{0};
	}
	const auto invLength = RealNum{1} / length;
	vector.x *= invLength;
	vector.y *= invLength;
	
	return length;
}

inline bool IsSleepable(Velocity velocity)
{
	return (Square(velocity.angular.ToRadians()) <= Square(AngularSleepTolerance))
	    && (GetLengthSquared(velocity.linear) <= Square(LinearSleepTolerance));
}

/// Gets the contact relative velocity.
/// @note If vcp_rA and vcp_rB are the zero vectors the resulting value is simply velB.linear - velA.linear.
constexpr inline Vec2 GetContactRelVelocity(const Velocity velA, const Vec2 vcp_rA,
											const Velocity velB, const Vec2 vcp_rB) noexcept
{
	return (velB.linear + (GetRevPerpendicular(vcp_rB) * velB.angular.ToRadians()))
		 - (velA.linear + (GetRevPerpendicular(vcp_rA) * velA.angular.ToRadians()));
}

template <>
inline Vec2 Average(Span<const Vec2> span)
{
	auto sum = Vec2(0, 0);
	for (auto&& element: span)
	{
		sum += element;
	}
	return (span.size() > decltype(span.size()){0})? sum / static_cast<Vec2::data_type>(span.size()): sum;
}

/// Computes the centroid of a counter-clockwise array of 3 or more vertices.
/// @note Behavior is undefined if there are less than 3 vertices or the vertices don't
///   go counter-clockwise.
Vec2 ComputeCentroid(const Span<const Vec2>& vertices);

template <typename T>
constexpr inline T GetModuloNext(T value, T count) noexcept
{
	assert(value < count);
	return (value + 1) % count;
}

template <typename T>
constexpr inline T GetModuloPrev(T value, T count) noexcept
{
	assert(value < count);
	return (value? value: count) - 1;
}

::std::ostream& operator<<(::std::ostream& os, const Vec2& value);

::std::ostream& operator<<(::std::ostream& os, const UnitVec2& value);

::std::ostream& operator<<(::std::ostream& os, const Angle& value);

::std::ostream& operator<<(::std::ostream& os, const Fixed32& value);

::std::ostream& operator<<(::std::ostream& os, const Fixed64& value);

}
#endif
