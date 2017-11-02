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

#ifndef PLAYRHO_COMMON_MATH_HPP
#define PLAYRHO_COMMON_MATH_HPP

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Span.hpp>
#include <PlayRho/Common/UnitVec2.hpp>
#include <PlayRho/Common/Vector2.hpp>
#include <PlayRho/Common/Vector3.hpp>
#include <PlayRho/Common/Position.hpp>
#include <PlayRho/Common/Velocity.hpp>
#include <PlayRho/Common/Acceleration.hpp>
#include <PlayRho/Common/Transformation.hpp>
#include <PlayRho/Common/Sweep.hpp>
#include <PlayRho/Common/Matrix.hpp>

#include <cmath>
#include <vector>
#include <numeric>

namespace playrho
{

// Other templates.

/// @brief Gets the "X" element of the given value - i.e. the first element.
template <typename T>
constexpr auto& GetX(T& value)
{
    return Get<0>(value);
}

/// @brief Gets the "Y" element of the given value - i.e. the second element.
template <typename T>
constexpr auto& GetY(T& value)
{
    return Get<1>(value);
}

/// @brief Gets the "Z" element of the given value - i.e. the third element.
template <typename T>
constexpr auto& GetZ(T& value)
{
    return Get<2>(value);
}

/// @brief Gets the "X" element of the given value - i.e. the first element.
template <typename T>
constexpr inline auto GetX(const T& value)
{
    return Get<0>(value);
}

/// @brief Gets the "Y" element of the given value - i.e. the second element.
template <typename T>
constexpr inline auto GetY(const T& value)
{
    return Get<1>(value);
}

/// @brief Gets the "Z" element of the given value - i.e. the third element.
template <typename T>
constexpr inline auto GetZ(const T& value)
{
    return Get<2>(value);
}

/// @brief Strips the unit from the given value.
template <typename T, LoValueCheck lo, HiValueCheck hi>
constexpr inline auto StripUnit(const BoundedValue<T, lo, hi>& v)
{
    return StripUnit(v.get());
}

/// @defgroup Math Mathematical functions.
/// @details These are non-member non-friend functions for mathematical operations
///   especially those with mixed input and output types.
/// @{

/// @brief Squares the given value.
template<class TYPE>
constexpr inline auto Square(TYPE t) noexcept { return t * t; }

/// @brief Square root's the given value.
template<typename T>
inline auto Sqrt(T t)
{
    // Note that std::sqrt is not declared noexcept at cppreference.com.
    // See: http://en.cppreference.com/w/cpp/numeric/math/sqrt
    return std::sqrt(StripUnit(t));
}

#ifdef USE_BOOST_UNITS
template<>
inline auto Sqrt(Area t)
{
    return std::sqrt(StripUnit(t)) * Meter;
}
#endif

/// @brief Computes the arc-tangent of the given y and x values.
template<typename T>
inline auto Atan2(T y, T x)
{
    return Angle{static_cast<Real>(std::atan2(StripUnit(y), StripUnit(x))) * Radian};
}

/// @brief Computes the arc-tangent of the given y and x values.
template<>
inline auto Atan2(double y, double x)
{
    return Angle{static_cast<Real>(std::atan2(y, x)) * Radian};
}

/// @brief Computes the absolute value of the given value.
template <typename T>
constexpr inline T Abs(T a)
{
    return (a >= T{0}) ? a : -a;
}

/// @brief Computes the average of the given values.
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

/// @brief Computes the rounded value of the given value.
template <typename T>
inline T Round(T value, unsigned precision = 100000);

/// @brief Computes the rounded value of the given value.
template <>
inline float Round(float value, std::uint32_t precision)
{
    const auto factor = float(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

/// @brief Computes the rounded value of the given value.
template <>
inline double Round(double value, std::uint32_t precision)
{
    const auto factor = double(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

/// @brief Computes the rounded value of the given value.
template <>
inline long double Round(long double value, std::uint32_t precision)
{
    using ldouble = long double;
    const auto factor = ldouble(static_cast<std::int64_t>(precision));
    return std::round(value * factor) / factor;
}

/// @brief Computes the rounded value of the given value.
template <>
inline Fixed32 Round(Fixed32 value, std::uint32_t precision)
{
    const auto factor = Fixed32(precision);
    return std::round(value * factor) / factor;
}

#ifndef _WIN32
/// @brief Computes the rounded value of the given value.
template <>
inline Fixed64 Round(Fixed64 value, std::uint32_t precision)
{
    const auto factor = Fixed64(precision);
    return std::round(value * factor) / factor;
}
#endif

/// @brief Computes the rounded value of the given value.
template <>
inline Vec2 Round(Vec2 value, std::uint32_t precision)
{
    return Vec2{Round(value[0], precision), Round(value[1], precision)};
}

/// @brief Gets a Vec2 representation of the given value.
constexpr inline Vec2 GetVec2(const UnitVec2 value)
{
    return Vec2{Get<0>(value), Get<1>(value)};
}

/// @brief Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occurring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool AlmostZero(float value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// @brief Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occurring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool AlmostZero(double value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// @brief Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occurring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool AlmostZero(long double value)
{
    return Abs(value) < std::numeric_limits<decltype(value)>::min();
}

/// @brief Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occurring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool AlmostZero(Fixed32 value)
{
    return value == 0;
}

#ifndef _WIN32
/// @brief Gets whether a given value is almost zero.
/// @details An almost zero value is "subnormal". Dividing by these values can lead to
/// odd results like a divide by zero trap occurring.
/// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
constexpr inline bool AlmostZero(Fixed64 value)
{
    return value == 0;
}
#endif

/// @brief Determines whether the given two values are "almost equal".
constexpr inline bool AlmostEqual(float x, float y, int ulp = 2)
{
    // From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
    //   "the machine epsilon has to be scaled to the magnitude of the values used
    //    and multiplied by the desired precision in ULPs (units in the last place)
    //    unless the result is subnormal".
    // Where "subnormal" means almost zero.
    //
    return (Abs(x - y) < (std::numeric_limits<float>::epsilon() * Abs(x + y) * ulp)) || AlmostZero(x - y);
}

/// @brief Determines whether the given two values are "almost equal".
constexpr inline bool AlmostEqual(double x, double y, int ulp = 2)
{
    // From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
    //   "the machine epsilon has to be scaled to the magnitude of the values used
    //    and multiplied by the desired precision in ULPs (units in the last place)
    //    unless the result is subnormal".
    // Where "subnormal" means almost zero.
    //
    return (Abs(x - y) < (std::numeric_limits<double>::epsilon() * Abs(x + y) * ulp)) || AlmostZero(x - y);
}

/// @brief Determines whether the given two values are "almost equal".
constexpr inline bool AlmostEqual(long double x, long double y, int ulp = 2)
{
    // From http://en.cppreference.com/w/cpp/types/numeric_limits/epsilon :
    //   "the machine epsilon has to be scaled to the magnitude of the values used
    //    and multiplied by the desired precision in ULPs (units in the last place)
    //    unless the result is subnormal".
    // Where "subnormal" means almost zero.
    //
    return (Abs(x - y) < (std::numeric_limits<long double>::epsilon() * Abs(x + y) * ulp)) || AlmostZero(x - y);
}

/// @brief Determines whether the given two values are "almost equal".
constexpr inline bool AlmostEqual(Fixed32 x, Fixed32 y, int ulp = 2)
{
    return Abs(x - y) <= Fixed32{0, static_cast<std::uint32_t>(ulp)};
}

#ifndef _WIN32
/// @brief Determines whether the given two values are "almost equal".
constexpr inline bool AlmostEqual(Fixed64 x, Fixed64 y, int ulp = 2)
{
    return Abs(x - y) <= Fixed64{0, static_cast<std::uint32_t>(ulp)};
}
#endif

/// @brief Gets the angle of the given unit vector.
inline Angle GetAngle(const UnitVec2 value)
{
    return Atan2(GetY(value), GetX(value));
}

/// @brief Gets the angle.
/// @return Angular value in the range of -Pi to +Pi radians.
template <class T>
inline Angle GetAngle(const Vector2<T> value)
{
    return Atan2(GetY(value), GetX(value));
}

/// @brief Gets the square of the length/magnitude of the given value.
/// @note For performance, use this instead of GetLength(T value) (if possible).
/// @return Non-negative value.
template <typename T>
constexpr inline auto GetLengthSquared(T value) noexcept
{
    return Square(GetX(value)) + Square(GetY(value));
}

/// @brief Gets the square of the length/magnitude of the given value.
template <>
constexpr inline auto GetLengthSquared(Vec3 value) noexcept
{
    return Square(GetX(value)) + Square(GetY(value)) + Square(GetZ(value));
}

/// @brief Gets the length/magnitude of the given value.
template <typename T>
inline auto GetLength(T value)
{
    return Sqrt(GetLengthSquared(value));
}

/// @brief Performs the dot product on two vectors (A and B).
///
/// @details The dot product of two vectors is defined as:
///   the magnitude of vector A, multiplied by, the magnitude of vector B,
///   multiplied by, the cosine of the angle between the two vectors (A and B).
///   Thus the dot product of two vectors is a value ranging between plus and minus the
///   magnitudes of each vector times each other.
///   The middle value of 0 indicates that two vectors are perpendicular to each other
///   (at an angle of +/- 90 degrees from each other).
///
/// @note This operation is commutative. I.e. Dot(a, b) == Dot(b, a).
/// @note If A and B are the same vectors, GetLengthSquared(Vec2) returns the same value
///   using effectively one less input parameter.
/// @note This is similar to the <code>std::inner_product</code> standard library algorithm
///   except benchmark tests suggest this implementation is faster at least for
///   <code>Vec2</code> like instances.
///
/// @sa https://en.wikipedia.org/wiki/Dot_product
///
/// @param a Vector A.
/// @param b Vector B.
///
/// @return Dot product of the vectors (0 means the two vectors are perpendicular).
///
template <typename T1, typename T2>
constexpr auto Dot(const T1 a, const T2 b) noexcept
{
    static_assert(a.size() == b.size(), "Dot only for same sized values");
    using VT1 = typename T1::value_type;
    using VT2 = typename T2::value_type;
    using OT = decltype(VT1{} * VT2{});
    auto result = OT{};
    const auto size = a.size();
    for (auto i = decltype(size){0}; i < size; ++i)
    {
        result += a[i] * b[i];
    }
    return result;
}

/// @brief Performs the 2-element analog of the cross product of two vectors.
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
/// @param a Value A of a 2-element type.
/// @param b Value B of a 2-element type.
///
/// @return Cross product of the two values.
///
template <class T1, class T2, std::enable_if_t<
    std::tuple_size<T1>::value == 2 && std::tuple_size<T2>::value == 2, int> = 0>
constexpr auto Cross(T1 a, T2 b) noexcept
{
    assert(std::isfinite(Get<0>(a)));
    assert(std::isfinite(Get<1>(a)));
    assert(std::isfinite(Get<0>(b)));
    assert(std::isfinite(Get<1>(b)));

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
    const auto minuend = Get<0>(a) * Get<1>(b);
    const auto subtrahend = Get<1>(a) * Get<0>(b);
    assert(std::isfinite(minuend));
    assert(std::isfinite(subtrahend));
    return minuend - subtrahend;
}

/// @brief Cross-products the given two values.
/// @note This operation is anti-commutative. I.e. Cross(a, b) == -Cross(b, a).
/// @sa https://en.wikipedia.org/wiki/Cross_product
/// @param a Value A of a 3-element type.
/// @param b Value B of a 3-element type.
/// @return Cross product of the two values.
template <class T1, class T2, std::enable_if_t<
    std::tuple_size<T1>::value == 3 && std::tuple_size<T2>::value == 3, int> = 0>
constexpr auto Cross(T1 a, T2 b) noexcept
{
    assert(std::isfinite(Get<0>(a)));
    assert(std::isfinite(Get<1>(a)));
    assert(std::isfinite(Get<2>(a)));
    assert(std::isfinite(Get<0>(b)));
    assert(std::isfinite(Get<1>(b)));
    assert(std::isfinite(Get<2>(b)));

    using OT = decltype(Get<0>(a) * Get<0>(b));
    return Vector<OT, 3>{
        GetY(a) * GetZ(b) - GetZ(a) * GetY(b),
        GetZ(a) * GetX(b) - GetX(a) * GetZ(b),
        GetX(a) * GetY(b) - GetY(a) * GetX(b)
    };
}

/// @brief Solves A * x = b, where b is a column vector.
/// @note This is more efficient than computing the inverse in one-shot cases.
template <typename T, typename U>
constexpr auto Solve(const Matrix22<U> mat, const Vector2<T> b) noexcept
{
    const auto cp = Cross(Get<0>(mat), Get<1>(mat));
    using OutType = decltype((U{} * T{}) / cp);
    return (!AlmostZero(StripUnit(cp)))?
        Vector2<OutType>{
            (Get<1>(mat)[1] * b[0] - Get<1>(mat)[0] * b[1]) / cp,
            (Get<0>(mat)[0] * b[1] - Get<0>(mat)[1] * b[0]) / cp
        }: Vector2<OutType>{};
}

/// @brief Inverts the given value.
template <class IN_TYPE>
constexpr auto Invert(const Matrix22<IN_TYPE> value) noexcept
{
    const auto cp = Cross(Get<0>(value), Get<1>(value));
    using OutType = decltype(Get<0>(value)[0] / cp);
    return (!AlmostZero(StripUnit(cp)))?
        Matrix22<OutType>{
            Vector2<OutType>{ Get<1>(Get<1>(value)) / cp, -Get<1>(Get<0>(value)) / cp},
            Vector2<OutType>{-Get<0>(Get<1>(value)) / cp,  Get<0>(Get<0>(value)) / cp}
        }:
        Matrix22<OutType>{};
}

/// @brief Solves A * x = b, where b is a column vector.
/// @note This is more efficient than computing the inverse in one-shot cases.
constexpr Vec3 Solve33(const Mat33& mat, const Vec3 b) noexcept
{
    const auto dp = Dot(GetX(mat), Cross(GetY(mat), GetZ(mat)));
    const auto det = (dp != 0)? 1 / dp: dp;
    const auto x = det * Dot(b, Cross(GetY(mat), GetZ(mat)));
    const auto y = det * Dot(GetX(mat), Cross(b, GetZ(mat)));
    const auto z = det * Dot(GetX(mat), Cross(GetY(mat), b));
    return Vec3{x, y, z};
}
    
/// @brief Solves A * x = b, where b is a column vector.
/// @note This is more efficient than computing the inverse in one-shot cases.
/// @note Solves only the upper 2-by-2 matrix equation.
template <typename T>
constexpr T Solve22(const Mat33& mat, const T b) noexcept
{
    const auto cp = GetX(GetX(mat)) * GetY(GetY(mat)) - GetX(GetY(mat)) * GetY(GetX(mat));
    const auto det = (cp != 0)? 1 / cp: cp;
    const auto x = det * (GetY(GetY(mat)) * GetX(b) - GetX(GetY(mat)) * GetY(b));
    const auto y = det * (GetX(GetX(mat)) * GetY(b) - GetY(GetX(mat)) * GetX(b));
    return T{x, y};
}

/// @brief Gets the inverse of the given matrix as a 2-by-2.
/// @return Zero matrix if singular.
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
    
/// @brief Gets the symmetric inverse of this matrix as a 3-by-3.
/// @return Zero matrix if singular.
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

/// @brief Gets a vector counter-clockwise (reverse-clockwise) perpendicular to the given vector.
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
    
/// @brief Gets a vector clockwise (forward-clockwise) perpendicular to the given vector.
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
        Get<0>(Get<0>(A)) * Get<0>(v) + Get<0>(Get<1>(A)) * Get<1>(v),
        Get<1>(Get<0>(A)) * Get<0>(v) + Get<1>(Get<1>(A)) * Get<1>(v)
    };
}

#ifdef USE_BOOST_UNITS
constexpr inline auto Transform(const LinearVelocity2 v, const Mass22& A) noexcept
{
    return Momentum2{
        Get<0>(Get<0>(A)) * Get<0>(v) + Get<0>(Get<1>(A)) * Get<1>(v),
        Get<1>(Get<0>(A)) * Get<0>(v) + Get<1>(Get<1>(A)) * Get<1>(v)
    };
}

constexpr inline auto Transform(const Momentum2 v, const InvMass22 A) noexcept
{
    return LinearVelocity2{
        Get<0>(Get<0>(A)) * Get<0>(v) + Get<0>(Get<1>(A)) * Get<1>(v),
        Get<1>(Get<0>(A)) * Get<0>(v) + Get<1>(Get<1>(A)) * Get<1>(v)
    };
}
#endif

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
constexpr inline Vec2 InverseTransform(const Vec2 v, const Mat22& A) noexcept
{
    return Vec2{Dot(v, GetX(A)), Dot(v, GetY(A))};
}

/// @brief Multiplication operator.
template <class T, LoValueCheck lo, HiValueCheck hi>
constexpr inline Vector2<T> operator* (BoundedValue<T, lo, hi> s, UnitVec2 u) noexcept
{
    return Vector2<T>{u.GetX() * s, u.GetY() * T{s}};
}

/// @brief Multiplication operator.
template <class T>
constexpr inline Vector2<T> operator* (const T s, const UnitVec2 u) noexcept
{
    return Vector2<T>{u.GetX() * s, u.GetY() * s};
}

/// @brief Multiplication operator.
template <class T, LoValueCheck lo, HiValueCheck hi>
constexpr inline Vector2<T> operator* (UnitVec2 u, BoundedValue<T, lo, hi> s) noexcept
{
    return Vector2<T>{u.GetX() * s, u.GetY() * T{s}};
}

/// @brief Multiplication operator.
template <class T>
constexpr inline Vector2<T> operator* (const UnitVec2 u, const T s) noexcept
{
    return Vector2<T>{u.GetX() * s, u.GetY() * s};
}

/// @brief Division operator.
constexpr inline Vec2 operator/ (const UnitVec2 u, const UnitVec2::value_type s) noexcept
{
    return Vec2{GetX(u) / s, GetY(u) / s};
}

/// @brief Computes A * B.
constexpr inline Mat22 Mul(const Mat22& A, const Mat22& B) noexcept
{
    return Mat22{Transform(GetX(B), A), Transform(GetY(B), A)};
}

/// @brief Computes A^T * B.
constexpr inline Mat22 MulT(const Mat22& A, const Mat22& B) noexcept
{
    const auto c1 = Vec2{Dot(GetX(A), GetX(B)), Dot(GetY(A), GetX(B))};
    const auto c2 = Vec2{Dot(GetX(A), GetY(B)), Dot(GetY(A), GetY(B))};
    return Mat22{c1, c2};
}

/// @brief Multiplies a matrix by a vector.
constexpr inline Vec3 Transform(const Vec3& v, const Mat33& A) noexcept
{
    return (GetX(v) * GetX(A)) + (GetY(v) * GetY(A)) + (GetZ(v) * GetZ(A));
}

/// @brief Multiplies a matrix by a vector.
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
constexpr inline auto Rotate(const Vector2<T> vector, const UnitVec2& angle) noexcept
{
    const auto newX = (angle.cos() * GetX(vector)) - (angle.sin() * GetY(vector));
    const auto newY = (angle.sin() * GetX(vector)) + (angle.cos() * GetY(vector));
    return Vector2<T>{newX, newY};
}

/// @brief Inverse rotates a vector.
/// @details This is the inverse of rotating a vector - it undoes what rotate does. I.e.
///   this effectively subtracts from the angle of the given vector the angle that's
///   expressed by the angle parameter.
/// @param vector Vector to reverse rotate.
/// @param angle Expresses the angle to reverse rotate the given vector by.
/// @sa Rotate.
template <class T>
constexpr inline auto InverseRotate(const Vector2<T> vector, const UnitVec2& angle) noexcept
{
    const auto newX = (angle.cos() * GetX(vector)) + (angle.sin() * GetY(vector));
    const auto newY = (angle.cos() * GetY(vector)) - (angle.sin() * GetX(vector));
    return Vector2<T>{newX, newY};
}

/// @brief Transforms the given 2-D vector with the given transformation.
/// @details
/// Rotate and translate the given 2-D linear position according to the rotation and translation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>InverseTransform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @note For a 2-D linear position of the origin (0, 0), the result is simply the translation.
/// @sa <code>InverseTransform</code>.
/// @param v 2-D position to transform (to rotate and then translate).
/// @param xfm Transformation (a translation and rotation) to apply to the given vector.
/// @return Rotated and translated vector.
constexpr inline Length2 Transform(const Length2 v, const Transformation xfm) noexcept
{
    return Rotate(v, xfm.q) + xfm.p;
}

/// @brief Inverse transforms the given 2-D vector with the given transformation.
/// @details
/// Inverse translate and rotate the given 2-D vector according to the translation and rotation
/// defined by the given transformation.
/// @note Passing the output of this function to <code>Transform</code> (with the same
/// transformation again) will result in the original vector being returned.
/// @sa <code>Transform</code>.
/// @param v 2-D vector to inverse transform (inverse translate and inverse rotate).
/// @param T Transformation (a translation and rotation) to inversely apply to the given vector.
/// @return Inverse transformed vector.
constexpr inline Length2 InverseTransform(const Length2 v, const Transformation T) noexcept
{
    const auto v2 = v - T.p;
    return InverseRotate(v2, T.q);
}

/// @brief Multiplies a given transformation by another given transformation.
/// @note v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///          = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
constexpr inline Transformation Mul(const Transformation& A, const Transformation& B) noexcept
{
    return Transformation{A.p + Rotate(B.p, A.q), A.q.Rotate(B.q)};
}

/// @brief Inverse multiplies a given transformation by another given transformation.
/// @note v2 = A.q' * (B.q * v1 + B.p - A.p)
///          = A.q' * B.q * v1 + A.q' * (B.p - A.p)
constexpr inline Transformation MulT(const Transformation& A, const Transformation& B) noexcept
{
    const auto dp = B.p - A.p;
    return Transformation{InverseRotate(dp, A.q), B.q.Rotate(A.q.FlipY())};
}

/// @brief Gets the absolute value of the given value.
template <>
inline Vec2 Abs(Vec2 a)
{
    return Vec2{Abs(a[0]), Abs(a[1])};
}

/// @brief Gets the absolute value of the given value.
template <>
inline UnitVec2 Abs(UnitVec2 a)
{
    return a.Absolute();
}

/// @brief Gets the absolute value of the given value.
inline Mat22 Abs(const Mat22& A)
{
    return Mat22{Abs(GetX(A)), Abs(GetY(A))};
}

/// @brief Clamps the given value within the given range (inclusive).
/// @param value Value to clamp.
/// @param low Lowest value to return or NaN to keep the low-end unbounded.
/// @param high Highest value to return or NaN to keep the high-end unbounded.
template <typename T>
constexpr inline T Clamp(T value, T low, T high) noexcept
{
    const auto tmp = (value > high)? high: value; // std::isnan(high)? a: Min(a, high);
    return (tmp < low)? low: tmp; // std::isnan(low)? b: Max(b, low);
}

/// @brief Gets the next largest power of 2
/// @details
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 64-bit value:"
inline std::uint64_t NextPowerOfTwo(std::uint64_t x)
{
    x |= (x >>  1u);
    x |= (x >>  2u);
    x |= (x >>  4u);
    x |= (x >>  8u);
    x |= (x >> 16u);
    x |= (x >> 32u);
    return x + 1;
}

/// @brief Gets the transformation for the given values.
constexpr inline Transformation GetTransformation(const Length2 ctr, const UnitVec2 rot,
                                                  const Length2 localCtr) noexcept
{
    assert(IsValid(rot));
    return Transformation{ctr - (Rotate(localCtr, rot)), rot};
}

/// @brief Gets the transformation for the given values.
inline Transformation GetTransformation(const Position pos, const Length2 local_ctr) noexcept
{
    assert(IsValid(pos));
    assert(IsValid(local_ctr));
    return GetTransformation(pos.linear, UnitVec2::Get(pos.angular), local_ctr);
}

/// @brief Gets the interpolated transform at a specific time.
/// @param sweep Sweep data to get the transform from.
/// @param beta Time factor in [0,1], where 0 indicates alpha0.
/// @return Transformation of the given sweep at the specified time.
inline Transformation GetTransformation(const Sweep& sweep, const Real beta) noexcept
{
    assert(beta >= 0);
    assert(beta <= 1);
    return GetTransformation(GetPosition(sweep.pos0, sweep.pos1, beta), sweep.GetLocalCenter());
}

/// @brief Gets the transform at "time" zero.
/// @note This is like calling GetTransformation(sweep, 0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, Real beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time zero.
inline Transformation GetTransform0(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos0, sweep.GetLocalCenter());
}

/// @brief Gets the transform at "time" one.
/// @note This is like calling GetTransformation(sweep, 1.0), except more efficiently.
/// @sa GetTransformation(const Sweep& sweep, Real beta).
/// @param sweep Sweep data to get the transform from.
/// @return Transformation of the given sweep at time one.
inline Transformation GetTransform1(const Sweep& sweep) noexcept
{
    return GetTransformation(sweep.pos1, sweep.GetLocalCenter());
}

/// @brief Gets the "normalized" value of the given angle.
inline Angle GetNormalized(Angle value)
{
    const auto angleInRadians = Real(value / Radian);
    return Angle{std::fmod(angleInRadians, Real(2 * Pi)) * Radian};
}

/// @brief Gets a sweep with the given sweeps's angles normalized.
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

/// @brief Converts the given vector into a unit vector and returns its original length.
inline Real Normalize(Vec2& vector)
{
    const auto length = GetLength(vector);
    if (!AlmostZero(length))
    {
        const auto invLength = 1 / length;
        vector[0] *= invLength;
        vector[1] *= invLength;
        return length;
    }
    return 0;
}

/// @brief Gets the contact relative velocity.
/// @note If relA and relB are the zero vectors, the resulting value is simply
///    velB.linear - velA.linear.
inline LinearVelocity2
GetContactRelVelocity(const Velocity velA, const Length2 relA,
                      const Velocity velB, const Length2 relB) noexcept
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

/// @brief Computes the centroid of a counter-clockwise array of 3 or more vertices.
/// @note Behavior is undefined if there are less than 3 vertices or the vertices don't
///   go counter-clockwise.
Length2 ComputeCentroid(const Span<const Length2>& vertices);

/// @brief Gets the modulo next value.
template <typename T>
constexpr inline T GetModuloNext(T value, T count) noexcept
{
    assert(value < count);
    return (value + 1) % count;
}

/// @brief Gets the modulo previous value.
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
/// @sa AlmostEqual.
template <class T>
inline UnitVec2 GetUnitVector(const Vector2<T> value,
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
/// @sa AlmostEqual.
template <class T>
inline UnitVec2 GetUnitVector(Vector2<T> value, T& magnitude,
                              UnitVec2 fallback = UnitVec2::GetDefaultFallback());

/// @brief Gets the unit vector of the given value.
template <>
inline UnitVec2 GetUnitVector(Vector2<Real> value, Real& magnitude, UnitVec2 fallback)
{
    return UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), magnitude, fallback);
}

#ifdef USE_BOOST_UNITS

/// @brief Gets the unit vector of the given value.
template <>
inline UnitVec2 GetUnitVector(Vector2<Length> value, Length& magnitude, UnitVec2 fallback)
{
    auto tmp = Real{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * Meter;
    return uv;
}

/// @brief Gets the unit vector of the given value.
template <>
inline UnitVec2 GetUnitVector(Vector2<LinearVelocity> value, LinearVelocity& magnitude,
                              UnitVec2 fallback)
{
    auto tmp = Real{0};
    const auto uv = UnitVec2::Get(StripUnit(GetX(value)), StripUnit(GetY(value)), tmp, fallback);
    magnitude = tmp * MeterPerSecond;
    return uv;
}

#endif // USE_BOOST_UNITS

/// @brief Gets the vertices for a circle described by the given parameters.
std::vector<Length2> GetCircleVertices(Length radius, unsigned slices,
                                        Angle start = Angle{0}, Real turns = Real{1});

/// @brief Gets the area of a cirlce.
NonNegative<Area> GetAreaOfCircle(Length radius);

/// @brief Gets the area of a polygon.
/// @note This function is valid for any non-self-intersecting (simple) polygon,
///   which can be convex or concave.
/// @note Winding order doesn't matter.
NonNegative<Area> GetAreaOfPolygon(Span<const Length2> vertices);

/// @brief Gets the polar moment of the area enclosed by the given vertices.
///
/// @warning Behavior is undefined if given collection has less than 3 vertices.
///
/// @param vertices Collection of three or more vertices.
///
SecondMomentOfArea GetPolarMoment(Span<const Length2> vertices);

/// @}

/// @brief Gets whether the given velocity is "under active" based on the given tolerances.
inline bool IsUnderActive(Velocity velocity,
                          LinearVelocity linSleepTol, AngularVelocity angSleepTol) noexcept
{
    const auto linVelSquared = GetLengthSquared(velocity.linear);
    const auto angVelSquared = Square(velocity.angular);
    return (angVelSquared <= Square(angSleepTol)) && (linVelSquared <= Square(linSleepTol));
}

} // namespace playrho

#endif // PLAYRHO_COMMON_MATH_HPP
