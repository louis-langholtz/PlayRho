/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_MATH_H
#define PLAYRHO_MATH_H

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Span.hpp>
#include <PlayRho/Common/UnitVec2.hpp>
#include <PlayRho/Common/Vector2D.hpp>
#include <PlayRho/Common/Vector3D.hpp>
#include <PlayRho/Common/Position.hpp>
#include <PlayRho/Common/Velocity.hpp>
#include <PlayRho/Common/Transformation.hpp>
#include <PlayRho/Common/Sweep.hpp>
#include <PlayRho/Common/Matrix.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <numeric>

namespace playrho
{
// Other templates.

template <typename T>
constexpr auto& GetX(T& value)
{
    return std::get<0>(value);
}

template <typename T>
constexpr auto& GetY(T& value)
{
    return std::get<1>(value);
}

template <typename T>
constexpr auto& GetZ(T& value)
{
    return std::get<2>(value);
}

template <typename T>
constexpr inline auto GetX(const T& value)
{
    return std::get<0>(value);
}

template <typename T>
constexpr inline auto GetY(const T& value)
{
    return std::get<1>(value);
}

template <typename T>
constexpr inline auto GetZ(const T& value)
{
    return std::get<2>(value);
}

template <typename T, LoValueCheck lo, HiValueCheck hi>
constexpr inline auto StripUnit(const BoundedValue<T, lo, hi>& v)
{
    return StripUnit(v.get());
}

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
    return Angle{static_cast<Real>(std::atan2(StripUnit(y), StripUnit(x))) * Radian};
}

template<>
inline auto Atan2(double y, double x)
{
    return Angle{static_cast<Real>(std::atan2(y, x)) * Radian};
}

template <typename T>
constexpr inline T Abs(T a)
{
    return (a >= T{0}) ? a : -a;
}

template <typename T>
inline auto Average(Span<const T> span)
{
    using value_type = typename std::remove_cv<T>::type;

    // Relies on C++11 zero initialization to zero initialize value_type.
    // See: http://en.cppreference.com/w/cpp/language/zero_initialization
    constexpr auto zero = value_type{};
    assert(zero * Real{2} == zero);
    
    // For C++17, switch from using std::accumulate to using std::reduce.
    const auto sum = std::accumulate(std::cbegin(span), std::cend(span), zero);
    const auto count = std::max(span.size(), std::size_t{1});
    return sum / static_cast<Real>(count);
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
    return Vec2{round(value[0], precision), round(value[1], precision)};
}

constexpr inline Vec2 GetVec2(const UnitVec2 value)
{
    return Vec2{std::get<0>(value), std::get<1>(value)};
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
inline Angle GetAngle(const T value)
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
    return Square(GetX(value)) + Square(GetY(value)) + Square(GetZ(value));
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
    return (std::get<0>(a) * std::get<0>(b)) + (std::get<1>(a) * std::get<1>(b));
}

/// Perform the dot product on two vectors.
template <>
constexpr inline auto Dot(const Vec3 a, const Vec3 b) noexcept
{
    return (std::get<0>(a) * std::get<0>(b)) + (std::get<1>(a) * std::get<1>(b))
        + (std::get<2>(a) * std::get<2>(b));
}

/// @brief Performs the 2D analog of the cross product of two vectors.
///
/// @details Defined as the result of: <code>(a.x * b.y) - (a.y * b.x)</code>.
///
/// @note This operation is dimension squashing. I.e. A cross of a 2-D length by a 2-D unit
///   vector results in a 1-D length value.
/// @note The unit of the result is the 1-D product of the inputs.
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
    return Vec3{
        GetY(a) * GetZ(b) - GetZ(a) * GetY(b),
        GetZ(a) * GetX(b) - GetX(a) * GetZ(b),
        GetX(a) * GetY(b) - GetY(a) * GetX(b)
    };
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
template <typename T>
constexpr T Solve(const Mat22 mat, const T b) noexcept
{
    const auto cp = Cross(std::get<0>(mat), std::get<1>(mat));
    return (cp != 0)?
        T{
            (std::get<1>(mat)[1] * b[0] - std::get<1>(mat)[0] * b[1]) / cp,
            (std::get<0>(mat)[0] * b[1] - std::get<0>(mat)[1] * b[0]) / cp
        }: T{};
}

template <class IN_TYPE>
constexpr auto Invert(const Matrix22<IN_TYPE> value) noexcept
{
    const auto cp = Cross(std::get<0>(value), std::get<1>(value));
    using OutType = decltype(std::get<0>(value)[0] / cp);
    return (!almost_zero(StripUnit(cp)))?
        Matrix22<OutType>{
            Vector2D<OutType>{ std::get<1>(std::get<1>(value)) / cp, -std::get<1>(std::get<0>(value)) / cp},
            Vector2D<OutType>{-std::get<0>(std::get<1>(value)) / cp,  std::get<0>(std::get<0>(value)) / cp}
        }:
        Matrix22<OutType>{};
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
constexpr Vec3 Solve33(const Mat33& mat, const Vec3 b) noexcept
{
    const auto dp = Dot(GetX(mat), Cross(GetY(mat), GetZ(mat)));
    const auto det = (dp != 0)? 1 / dp: dp;
    const auto x = det * Dot(b, Cross(GetY(mat), GetZ(mat)));
    const auto y = det * Dot(GetX(mat), Cross(b, GetZ(mat)));
    const auto z = det * Dot(GetX(mat), Cross(GetY(mat), b));
    return Vec3{x, y, z};
}
    
/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases. Solve only the upper
/// 2-by-2 matrix equation.
    template <typename T>
constexpr T Solve22(const Mat33& mat, const T b) noexcept
{
    const auto cp = GetX(GetX(mat)) * GetY(GetY(mat)) - GetX(GetY(mat)) * GetY(GetX(mat));
    const auto det = (cp != 0)? 1 / cp: cp;
    const auto x = det * (GetY(GetY(mat)) * GetX(b) - GetX(GetY(mat)) * GetY(b));
    const auto y = det * (GetX(GetX(mat)) * GetY(b) - GetY(GetX(mat)) * GetX(b));
    return T{x, y};
}

/// Get the inverse of this matrix as a 2-by-2.
/// Returns the zero matrix if singular.
constexpr inline Mat33 GetInverse22(const Mat33& value) noexcept
{
    const auto a = GetX(GetX(value)), b = GetX(GetY(value)), c = GetY(GetX(value)), d = GetY(GetY(value));
    auto det = (a * d) - (b * c);
    if (det != Real{0})
    {
        det = Real{1} / det;
    }
    return Mat33{Vec3{det * d, -det * c, Real{0}}, Vec3{-det * b, det * a, 0}, Vec3{0, 0, 0}};
}
    
/// Get the symmetric inverse of this matrix as a 3-by-3.
/// Returns the zero matrix if singular.
constexpr inline Mat33 GetSymInverse33(const Mat33& value) noexcept
{
    auto det = Dot(GetX(value), Cross(GetY(value), GetZ(value)));
    if (det != Real{0})
    {
        det = Real{1} / det;
    }
    
    const auto a11 = GetX(GetX(value)), a12 = GetX(GetY(value)), a13 = GetX(GetZ(value));
    const auto a22 = GetY(GetY(value)), a23 = GetY(GetZ(value));
    const auto a33 = GetZ(GetZ(value));
    
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
    return Vec2{
        GetX(GetX(A)) * GetX(v) + GetX(GetY(A)) * GetY(v),
        GetY(GetX(A)) * GetX(v) + GetY(GetY(A)) * GetY(v)
    };
}

#ifdef USE_BOOST_UNITS
constexpr inline auto Transform(const LinearVelocity2D v, const Mass22& A) noexcept
{
    return Momentum2D{
        GetX(GetX(A)) * GetX(v) + GetX(GetY(A)) * GetY(v),
        GetY(GetX(A)) * GetX(v) + GetY(GetY(A)) * GetY(v)
    };
}

constexpr inline auto Transform(const Momentum2D v, const InvMass22 A) noexcept
{
    return LinearVelocity2D{
        GetX(GetX(A)) * GetX(v) + GetX(GetY(A)) * GetY(v),
        GetY(GetX(A)) * GetX(v) + GetY(GetY(A)) * GetY(v)
    };
}
#endif

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline Vec2 InverseTransform(const Vec2 v, const Mat22& A) noexcept
{
    return Vec2{Dot(v, GetX(A)), Dot(v, GetY(A))};
}

template <class T, LoValueCheck lo, HiValueCheck hi>
constexpr inline Vector2D<T> operator* (BoundedValue<T, lo, hi> s, UnitVec2 u) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * T{s}};
}

template <class T>
constexpr inline Vector2D<T> operator* (const T s, const UnitVec2 u) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * s};
}

template <class T, LoValueCheck lo, HiValueCheck hi>
constexpr inline Vector2D<T> operator* (UnitVec2 u, BoundedValue<T, lo, hi> s) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * T{s}};
}

template <class T>
constexpr inline Vector2D<T> operator* (const UnitVec2 u, const T s) noexcept
{
    return Vector2D<T>{u.GetX() * s, u.GetY() * s};
}

constexpr inline Vec2 operator/ (const UnitVec2 u, const UnitVec2::value_type s) noexcept
{
    return Vec2{GetX(u) / s, GetY(u) / s};
}

// A * B
constexpr inline Mat22 Mul(const Mat22& A, const Mat22& B) noexcept
{
    return Mat22{Transform(GetX(B), A), Transform(GetY(B), A)};
}

// A^T * B
constexpr inline Mat22 MulT(const Mat22& A, const Mat22& B) noexcept
{
    const auto c1 = Vec2{Dot(GetX(A), GetX(B)), Dot(GetY(A), GetX(B))};
    const auto c2 = Vec2{Dot(GetX(A), GetY(B)), Dot(GetY(A), GetY(B))};
    return Mat22{c1, c2};
}

/// Multiply a matrix times a vector.
constexpr inline Vec3 Transform(const Vec3& v, const Mat33& A) noexcept
{
    return (GetX(v) * GetX(A)) + (GetY(v) * GetY(A)) + (GetZ(v) * GetZ(A));
}

/// Multiply a matrix times a vector.
constexpr inline Vec2 Transform(const Vec2 v, const Mat33& A) noexcept
{
    return Vec2{
        GetX(GetX(A)) * v[0] + GetX(GetY(A)) * v[1],
        GetY(GetX(A)) * v[0] + GetY(GetY(A)) * v[1]
    };
}

/// @brief Rotates a vector by a given angle.
/// @details This rotates a vector by the angle expressed by the angle parameter.
/// @param vector Vector to forward rotate.
/// @param angle Expresses the angle to forward rotate the given vector by.
/// @sa InverseRotate.
template <class T>
constexpr inline auto Rotate(const Vector2D<T> vector, const UnitVec2& angle) noexcept
{
    const auto newX = (angle.cos() * GetX(vector)) - (angle.sin() * GetY(vector));
    const auto newY = (angle.sin() * GetX(vector)) + (angle.cos() * GetY(vector));
    return Vector2D<T>{newX, newY};
}

/// @brief Inverse rotates a vector.
/// @details This is the inverse of rotating a vector - it undoes what rotate does. I.e.
///   this effectively subtracts from the angle of the given vector the angle that's
///   expressed by the angle parameter.
/// @param vector Vector to reverse rotate.
/// @param angle Expresses the angle to reverse rotate the given vector by.
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
    return Vec2{Abs(a[0]), Abs(a[1])};
}

template <>
inline UnitVec2 Abs(UnitVec2 a)
{
    return a.Absolute();
}

inline Mat22 Abs(const Mat22& A)
{
    return Mat22{Abs(GetX(A)), Abs(GetY(A))};
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
    return GetTransformation(pos.linear, UnitVec2::Get(pos.angular), local_ctr);
}

/// Gets the interpolated transform at a specific time.
/// @param sweep Sweep data to get the transform from.
/// @param beta Time factor in [0,1], where 0 indicates alpha0.
/// @return Transformation of the given sweep at the specified time.
inline Transformation GetTransformation(const Sweep& sweep, const Real beta) noexcept
{
    assert(beta >= 0);
    assert(beta <= 1);
    return GetTransformation(GetPosition(sweep.pos0, sweep.pos1, beta), sweep.GetLocalCenter());
}

/// Gets the transform at "time" zero.
/// @note This is like calling GetTransformation(sweep, 0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, Real beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time zero.
inline Transformation GetTransform0(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos0, sweep.GetLocalCenter());
}

/// Gets the transform at "time" one.
/// @note This is like calling GetTransformation(sweep, 1.0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, Real beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time one.
inline Transformation GetTransform1(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos1, sweep.GetLocalCenter());
}

inline Angle GetNormalized(Angle value)
{
    const auto angleInRadians = Real(value / Radian);
    return Angle{std::fmod(angleInRadians, Real(2 * Pi)) * Radian};
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
inline Real Normalize(Vec2& vector)
{
    const auto length = GetLength(vector);
    if (!almost_zero(length))
    {
        const auto invLength = 1 / length;
        vector[0] *= invLength;
        vector[1] *= invLength;
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
    const auto velBrot = GetRevPerpendicular(relB) * (velB.angular / Radian);
    const auto velArot = GetRevPerpendicular(relA) * (velA.angular / Radian);
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
    return (a1 > a2)? Angle(Real{360} * Degree) - (a1 - a2): a2 - a1;
}

/// Gets the unit vector for the given value.
/// @param value Value to get the unit vector for.
/// @param fallback Fallback unit vector value to use in case a unit vector can't effectively be
///   calculated from the given value.
/// @return value divided by its length if length not almost zero otherwise invalid value.
/// @sa almost_equal.
template <class T>
inline UnitVec2 GetUnitVector(const Vector2D<T> value,
                              const UnitVec2 fallback = UnitVec2::GetDefaultFallback())
{
    auto magnitude = Real(1);
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
inline UnitVec2 GetUnitVector(const Vector2D<Real> value, Real& magnitude, const UnitVec2 fallback)
{
    return UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), magnitude, fallback);
}

#ifdef USE_BOOST_UNITS

template <>
inline UnitVec2 GetUnitVector(const Vector2D<Length> value, Length& magnitude, const UnitVec2 fallback)
{
    auto tmp = Real{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * Meter;
    return uv;
}

template <>
inline UnitVec2 GetUnitVector(const Vector2D<LinearVelocity> value, LinearVelocity& magnitude,
                              const UnitVec2 fallback)
{
    auto tmp = Real{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * MeterPerSecond;
    return uv;
}

#endif

std::vector<Length2D> GetCircleVertices(const Length radius, unsigned slices,
                                        Angle start = Angle{0}, Real turns = Real{1});

::std::ostream& operator<<(::std::ostream& os, const Vec2& value);

::std::ostream& operator<<(::std::ostream& os, const UnitVec2& value);

::std::ostream& operator<<(::std::ostream& os, const Fixed32& value);

#ifndef _WIN32
::std::ostream& operator<<(::std::ostream& os, const Fixed64& value);
#endif

}
#endif
