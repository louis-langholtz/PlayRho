/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Common/UnitVec2.hpp>
#include <Box2D/Common/Vector2D.hpp>
#include <Box2D/Common/Vector3D.hpp>
#include <Box2D/Common/Position.hpp>
#include <Box2D/Common/Velocity.hpp>
#include <Box2D/Common/Transformation.hpp>
#include <Box2D/Common/Sweep.hpp>
#include <Box2D/Common/Mat22.hpp>
#include <Box2D/Common/Mat33.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <numeric>

namespace box2d
{
// Other templates.

template<class TYPE>
constexpr inline auto Square(TYPE t) noexcept { return t * t; }

template<typename T>
inline auto Sqrt(T t)
{
    return std::sqrt(StripUnit(t));
}

#ifdef USE_BOOST_UNITS
template<>
inline auto Sqrt(Area t)
{
    return std::sqrt(StripUnit(t)) * Meter;
}
#endif

template<typename T>
inline auto Atan2(T y, T x)
{
    return Angle{static_cast<RealNum>(std::atan2(StripUnit(y), StripUnit(x))) * Radian};
}

template<>
inline auto Atan2(double y, double x)
{
    return Angle{static_cast<RealNum>(std::atan2(y, x)) * Radian};
}

template <typename T>
constexpr inline T Abs(T a)
{
    return (a >= T{0}) ? a : -a;
}

template <typename T>
inline auto Average(Span<const T> span)
{
    // For C++17, switch from using std::accumulate to using std::reduce.
    const auto sum = std::accumulate(std::cbegin(span), std::cend(span), static_cast<T>(0));
    const auto count = static_cast<RealNum>(span.size());
    return (count > decltype(count){0})? sum / count: sum / RealNum(1);
}

template <typename T>
inline T round(T value, unsigned precision = 100000);

template <>
inline float round(float value, std::uint32_t precision)
{
    const auto factor = float(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

template <>
inline double round(double value, std::uint32_t precision)
{
    const auto factor = double(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

template <>
inline long double round(long double value, std::uint32_t precision)
{
    using ldouble = long double;
    const auto factor = ldouble(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

template <>
inline Fixed32 round(Fixed32 value, std::uint32_t precision)
{
    const auto factor = Fixed32(precision);
    return std::round(value * factor) / factor;
}

#ifndef _WIN32
template <>
inline Fixed64 round(Fixed64 value, std::uint32_t precision)
{
    const auto factor = Fixed64(precision);
    return std::round(value * factor) / factor;
}
#endif

template <>
inline Vec2 round(Vec2 value, std::uint32_t precision)
{
    return Vec2{round(value.x, precision), round(value.y, precision)};
}

/// Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(float value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(double value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(long double value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(Fixed32 value)
{
    return value == 0;
}

#ifndef _WIN32
/// Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occuring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool almost_zero(Fixed64 value)
{
    return value == 0;
}
#endif

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
    return Abs(x - y) <= Fixed32{0, static_cast<std::uint32_t>(ulp)};
}

#ifndef _WIN32
constexpr inline bool almost_equal(Fixed64 x, Fixed64 y, int ulp = 2)
{
    return Abs(x - y) <= Fixed64{0, static_cast<std::uint32_t>(ulp)};
}
#endif

/// Gets the angle.
/// @return Anglular value in the range of -Pi to +Pi radians.
template <class T>
inline Angle GetAngle(T value)
{
    return Atan2(GetY(value), GetX(value));
}

/// Gets the square of the length/magnitude of the given value.
/// For performance, use this instead of GetLength(T value) (if possible).
/// @return Non-negative value.
template <typename T>
constexpr inline auto GetLengthSquared(T value) noexcept
{
    return Square(GetX(value)) + Square(GetY(value));
}

template <>
constexpr inline auto GetLengthSquared(Vec3 value) noexcept
{
    return Square(value.x) + Square(value.y) + Square(value.z);        
}

template <typename T>
inline auto GetLength(T value)
{
    return Sqrt(GetLengthSquared(value));
}

/// Performs the dot product on two vectors (A and B).
///
/// @details The dot product of two vectors is defined as:
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
template <typename T1, typename T2>
constexpr inline auto Dot(const T1 a, const T2 b) noexcept
{
    return (GetX(a) * GetX(b)) + (GetY(a) * GetY(b));
}

/// Perform the dot product on two vectors.
template <>
constexpr inline auto Dot(const Vec3 a, const Vec3 b) noexcept
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

/// Performs the 2D analog of the cross product of two vectors.
///
/// @details
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
template <class T1, class T2>
constexpr inline auto Cross(const T1 a, const T2 b) noexcept
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

template <>
constexpr inline auto Cross(const Vec3 a, const Vec3 b) noexcept
{
    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

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

struct ContactImpulses
{
    Momentum m_normal; ///< Normal impulse. This is the non-penetration impulse (4-bytes).
    Momentum m_tangent; ///< Tangent impulse. This is the friction impulse (4-bytes).
};

/// Gets a vector counter-clockwise (reverse-clockwise) perpendicular to the given vector.
/// @details This takes a vector of form (x, y) and returns the vector (-y, x).
/// @param vector Vector to return a counter-clockwise perpendicular equivalent for.
/// @return A counter-clockwise 90-degree rotation of the given vector.
/// @sa GetFwdPerpendicular.
template <class T>
constexpr inline auto GetRevPerpendicular(const T vector) noexcept
{
    // See http://mathworld.wolfram.com/PerpendicularVector.html
    return T{-GetY(vector), GetX(vector)};
}
    
/// Gets a vector clockwise (forward-clockwise) perpendicular to the given vector.
/// @details This takes a vector of form (x, y) and returns the vector (y, -x).
/// @param vector Vector to return a clockwise perpendicular equivalent for.
/// @return A clockwise 90-degree rotation of the given vector.
/// @sa GetRevPerpendicular.
template <class T>
constexpr inline auto GetFwdPerpendicular(const T vector) noexcept
{
    // See http://mathworld.wolfram.com/PerpendicularVector.html
    return T{GetY(vector), -GetX(vector)};
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

constexpr inline Vec2 operator+ (const UnitVec2 lhs, const UnitVec2 rhs) noexcept
{
    return Vec2{lhs.GetX() + rhs.GetX(), lhs.GetY() + rhs.GetY()};
}

constexpr inline Vec2 operator- (const UnitVec2 lhs, const UnitVec2 rhs) noexcept
{
    return Vec2{lhs.GetX() - rhs.GetX(), lhs.GetY() - rhs.GetY()};
}

template <class T>
constexpr inline Vector2D<T> operator* (const T s, const UnitVec2 u) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * s};
}

template <class T>
constexpr inline Vector2D<T> operator* (const UnitVec2 u, const T s) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * s};
}

constexpr inline Vec2 operator/ (const UnitVec2 u, const UnitVec2::data_type s) noexcept
{
    return Vec2{u.GetX() / s, u.GetY() / s};
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
/// @sa InverseRotate.
template <class T>
constexpr inline auto Rotate(const Vector2D<T> vector, const UnitVec2& angle) noexcept
{
    const auto newX = (angle.cos() * GetX(vector)) - (angle.sin() * GetY(vector));
    const auto newY = (angle.sin() * GetX(vector)) + (angle.cos() * GetY(vector));
    return Vector2D<T>{newX, newY};
}

/// Inverse rotate a vector.
/// @details This is the inverse of rotating a vector - it undoes what rotate does.
/// @sa Rotate.
template <class T>
constexpr inline auto InverseRotate(const Vector2D<T> vector, const UnitVec2& angle) noexcept
{
    const auto newX = (angle.cos() * GetX(vector)) + (angle.sin() * GetY(vector));
    const auto newY = (angle.cos() * GetY(vector)) - (angle.sin() * GetX(vector));
    return Vector2D<T>{newX, newY};
}

/// Transforms the given 2-D vector with the given transformation.
/// @details
/// Rotate and translate the given 2-D linear position according to the rotation and translation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>InverseTransform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @note For a 2-D linear position of the origin (0, 0), the result is simply the translation.
/// @sa <code>InverseTransform</code>.
/// @param v 2-D position to transform (to rotate and then translate).
/// @param T Transformation (a translation and rotation) to apply to the given vector.
/// @return Rotated and translated vector.
constexpr inline Length2D Transform(const Length2D v, const Transformation T) noexcept
{
    return Rotate(v, T.q) + T.p;
}

/// Inverse transforms the given 2-D vector with the given transformation.
/// @details
/// Inverse translate and rotate the given 2-D vector according to the translation and rotation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>Transform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @sa <code>Transform</code>.
/// @param v 2-D vector to inverse transform (inverse translate and inverse rotate).
/// @param T Transformation (a translation and rotation) to invertedly apply to the given vector.
/// @return Inverse transformed vector.
constexpr inline Length2D InverseTransform(const Length2D v, const Transformation T) noexcept
{
    const auto v2 = v - T.p;
    return InverseRotate(v2, T.q);
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
    const auto dp = B.p - A.p;
    return Transformation{InverseRotate(dp, A.q), B.q.Rotate(A.q.FlipY())};
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
constexpr inline T Min(T a, T b) noexcept
{
    return (a < b) ? a : b;
}

template <typename T>
constexpr inline T Max(T a, T b) noexcept
{
    return (a > b) ? a : b;
}

/// Clamps the given value within the given range (inclusive).
/// @param value Value to clamp.
/// @param low Lowest value to return or NaN to keep the low-end unbounded.
/// @param high Highest value to return or NaN to keep the high-end unbounded.
template <typename T>
constexpr inline T Clamp(T value, T low, T high) noexcept
{
    const auto tmp = (value > high)? high: value; // std::isnan(high)? a: Min(a, high);
    return (tmp < low)? low: tmp; // std::isnan(low)? b: Max(b, low);
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
inline std::uint64_t NextPowerOfTwo(std::uint64_t x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    x |= (x >> 32);
    return x + 1;
}

constexpr inline Transformation GetTransformation(const Length2D ctr, const UnitVec2 rot, const Length2D localCtr) noexcept
{
    return Transformation{ctr - (Rotate(localCtr, rot)), rot};
}

inline Transformation GetTransformation(const Position pos, const Length2D local_ctr) noexcept
{
    assert(IsValid(pos));
    assert(IsValid(local_ctr));
    return GetTransformation(pos.linear, UnitVec2{pos.angular}, local_ctr);
}

/// Gets the interpolated transform at a specific time.
/// @param sweep Sweep data to get the transform from.
/// @param beta Time factor in [0,1], where 0 indicates alpha0.
/// @return Transformation of the given sweep at the specified time.
inline Transformation GetTransformation(const Sweep& sweep, const RealNum beta) noexcept
{
    assert(beta >= 0);
    assert(beta <= 1);
    return GetTransformation(GetPosition(sweep.pos0, sweep.pos1, beta), sweep.GetLocalCenter());
}

/// Gets the transform at "time" zero.
/// @note This is like calling GetTransformation(sweep, 0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, RealNum beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time zero.
inline Transformation GetTransform0(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos0, sweep.GetLocalCenter());
}

/// Gets the transform at "time" one.
/// @note This is like calling GetTransformation(sweep, 1.0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, RealNum beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time one.
inline Transformation GetTransform1(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos1, sweep.GetLocalCenter());
}

inline Angle GetNormalized(Angle value)
{
#if 1
    constexpr auto radsPerCircle = RealNum{2 * Pi} * Radian;
    const auto laps = static_cast<int>(RealNum{value / radsPerCircle});
    return value - Angle(RealNum(laps) * radsPerCircle);
#else
    const auto TwoPi = Pi * 2;
    const auto res = RealNum{value / Radian} / TwoPi;
    const auto wholePart = static_cast<int>(res);
    const auto fractionPart = res - wholePart;
    return RealNum{fractionPart * TwoPi} * Radian;
#endif
}

/// Gets a sweep with the given sweep's angles normalized.
/// @param sweep Sweep to return with its angles normalized.
/// @return Sweep with its pos0 angle to be between -2 pi and 2 pi
///    and its pos1 angle reduced by the amount pos0's angle was reduced by.
inline Sweep GetAnglesNormalized(Sweep sweep) noexcept
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
    if (!almost_zero(length))
    {
        const auto invLength = 1 / length;
        vector.x *= invLength;
        vector.y *= invLength;
        return length;
    }
    return 0;
}

inline bool IsUnderActive(Velocity velocity,
                          LinearVelocity linSleepTol, AngularVelocity angSleepTol) noexcept
{
    const auto linVelSquared = GetLengthSquared(velocity.linear);
    const auto angVelSquared = Square(velocity.angular);
    return (angVelSquared <= Square(angSleepTol)) && (linVelSquared <= Square(linSleepTol));
}

/// @brief Gets the contact relative velocity.
/// @note If relA and relB are the zero vectors, the resulting value is simply velB.linear - velA.linear.
inline LinearVelocity2D
GetContactRelVelocity(const Velocity velA, const Length2D relA,
                      const Velocity velB, const Length2D relB) noexcept
{
#if 0 // Using std::fma appears to be slower!
    const auto revPerpRelB = GetRevPerpendicular(relB);
    const auto xRevPerpRelB = StripUnit(revPerpRelB.x);
    const auto yRevPerpRelB = StripUnit(revPerpRelB.y);
    const auto angVelB = StripUnit(velB.angular);
    const auto xLinVelB = StripUnit(velB.linear.x);
    const auto yLinVelB = StripUnit(velB.linear.y);
    const auto xFmaB = std::fma(xRevPerpRelB, angVelB, xLinVelB);
    const auto yFmaB = std::fma(yRevPerpRelB, angVelB, yLinVelB);
    
    const auto revPerpRelA = GetRevPerpendicular(relA);
    const auto xRevPerpRelA = StripUnit(revPerpRelA.x);
    const auto yRevPerpRelA = StripUnit(revPerpRelA.y);
    const auto angVelA = StripUnit(velA.angular);
    const auto xLinVelA = StripUnit(velA.linear.x);
    const auto yLinVelA = StripUnit(velA.linear.y);
    const auto xFmaA = std::fma(xRevPerpRelA, angVelA, xLinVelA);
    const auto yFmaA = std::fma(yRevPerpRelA, angVelA, yLinVelA);
    
    const auto deltaFmaX = xFmaB - xFmaA;
    const auto deltaFmaY = yFmaB - yFmaA;
    
    return Vec2{deltaFmaX, deltaFmaY} * MeterPerSecond;
#else
    const auto velBrot = GetRevPerpendicular(relB) * velB.angular / Radian;
    const auto velArot = GetRevPerpendicular(relA) * velA.angular / Radian;
    return (velB.linear + velBrot) - (velA.linear + velArot);
#endif
}

/// Computes the centroid of a counter-clockwise array of 3 or more vertices.
/// @note Behavior is undefined if there are less than 3 vertices or the vertices don't
///   go counter-clockwise.
Length2D ComputeCentroid(const Span<const Length2D>& vertices);

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

/// Gets the reverse (counter) clockwise rotational angle to go from angle 1 to angle 2.
/// @note The given angles must be normalized between -Pi to Pi radians.
/// @return Angular rotation in the counter clockwise direction to go from angle 1 to angle 2.
constexpr inline Angle GetRevRotationalAngle(Angle a1, Angle a2) noexcept
{
    // If a1=90 * Degree and a2=45 * Degree then, 360 * Degree - (90 * Degree - 45) = 315 * Degree
    // If a1=90 * Degree and a2=-90 * Degree then, 360 * Degree - (90 * Degree - -90 * Degree) = 180 * Degree
    // If a1=45 * Degree and a2=90 * Degree then, 90 * Degree - 45 * Degree = 45 * Degree
    // If a1=90 * Degree and a2=45 * Degree then, 360 * Degree - 45 * Degree - 90 * Degree = 235 * Degree
    // If a1=-45 * Degree and a2=0 * Degree then, 45 * Degree
    // If a1=-90 * Degree and a2=-100 * Degree then, 360 * Degree - (-90 * Degree - -100 * Degree) = 350 * Degree
    // If a1=-100 * Degree and a2=-90 * Degree then, -90 * Degree - -100 * Degree = 10 * Degree
    return (a1 > a2)? RealNum{360} * Degree - (a1 - a2): a2 - a1;
}

constexpr inline Vec2 GetVec2(const UnitVec2 value)
{
    return Vec2{GetX(value), GetY(value)};
}

/// Gets the unit vector for the given value.
/// @param value Value to get the unit vector for.
/// @param fallback Fallback unit vector value to use in case a unit vector can't effectively be
///   calculated from the given value.
/// @return value divided by its length if length not almost zero otherwise invalid value.
/// @sa almost_equal.
template <class T>
inline UnitVec2 GetUnitVector(const Vector2D<T> value, const UnitVec2 fallback = UnitVec2::GetDefaultFallback())
{
    RealNum magnitude = 1;
    return UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), magnitude, fallback);
}

/// Gets the unit vector for the given value.
/// @param value Value to get the unit vector for.
/// @param magnitude Returns the calculated magnitude of the given vector.
/// @param fallback Fallback unit vector value to use in case a unit vector can't effectively be
///   calculated from the given value.
/// @return value divided by its length if length not almost zero otherwise invalid value.
/// @sa almost_equal.
template <class T>
inline UnitVec2 GetUnitVector(const Vector2D<T> value, T& magnitude,
                              const UnitVec2 fallback = UnitVec2::GetDefaultFallback());

template <>
inline UnitVec2 GetUnitVector(const Vector2D<RealNum> value, RealNum& magnitude, const UnitVec2 fallback)
{
    return UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), magnitude, fallback);
}

#ifdef USE_BOOST_UNITS

template <>
inline UnitVec2 GetUnitVector(const Vector2D<Length> value, Length& magnitude, const UnitVec2 fallback)
{
    auto tmp = RealNum{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * Meter;
    return uv;
}

template <>
inline UnitVec2 GetUnitVector(const Vector2D<LinearVelocity> value, LinearVelocity& magnitude,
                              const UnitVec2 fallback)
{
    auto tmp = RealNum{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * MeterPerSecond;
    return uv;
}

#endif

std::vector<Length2D> GetCircleVertices(const Length radius, unsigned slices,
                                        Angle start = Angle{0}, RealNum turns = RealNum{1});

::std::ostream& operator<<(::std::ostream& os, const Vec2& value);

::std::ostream& operator<<(::std::ostream& os, const UnitVec2& value);

::std::ostream& operator<<(::std::ostream& os, const Fixed32& value);

#ifndef _WIN32
::std::ostream& operator<<(::std::ostream& os, const Fixed64& value);
#endif

}
#endif
