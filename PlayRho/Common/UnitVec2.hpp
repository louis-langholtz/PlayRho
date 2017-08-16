/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef UnitVec2_hpp
#define UnitVec2_hpp

/// @file
/// Declarations of the UnitVec2 class and free functions associated with it.

#include <PlayRho/Common/Settings.hpp>
#include <cmath>

namespace playrho
{
/// @brief 2-D unit vector.
/// @details This is a 2-dimensional directional vector.
class UnitVec2
{
public:
    /// @brief Value type used for the coordinate values of this vector.
    using value_type = Real;

    /// @brief Gets the right-ward oriented unit vector.
    /// @note This is the value for the 0/4 turned (0 angled) unit vector.
    /// @note This is the reverse perpendicular unit vector of the bottom oriented vector.
    /// @note This is the forward perpendicular unit vector of the top oriented vector.
    static constexpr UnitVec2 GetRight() noexcept { return UnitVec2{1, 0}; }

    /// @brief Gets the top-ward oriented unit vector.
    /// @note This is the actual value for the 1/4 turned (90 degree angled) unit vector.
    /// @note This is the reverse perpendicular unit vector of the right oriented vector.
    /// @note This is the forward perpendicular unit vector of the left oriented vector.
    static constexpr UnitVec2 GetTop() noexcept { return UnitVec2{0, 1}; }

    /// @brief Gets the left-ward oriented unit vector.
    /// @note This is the actual value for the 2/4 turned (180 degree angled) unit vector.
    /// @note This is the reverse perpendicular unit vector of the top oriented vector.
    /// @note This is the forward perpendicular unit vector of the bottom oriented vector.
    static constexpr UnitVec2 GetLeft() noexcept { return UnitVec2{-1, 0}; }

    /// @brief Gets the bottom-ward oriented unit vector.
    /// @note This is the actual value for the 3/4 turned (270 degree angled) unit vector.
    /// @note This is the reverse perpendicular unit vector of the left oriented vector.
    /// @note This is the forward perpendicular unit vector of the right oriented vector.
    static constexpr UnitVec2 GetBottom() noexcept { return UnitVec2{0, -1}; }

    /// @brief Gets the non-oriented unit vector.
    static constexpr UnitVec2 GetZero() noexcept { return UnitVec2{0, 0}; }

    /// @brief Gets the 45 degree unit vector.
    /// @details This is the unit vector in the positive X and Y quadrant where X == Y.
    static constexpr UnitVec2 GetTopRight() noexcept
    {
        return UnitVec2{+SquareRootTwo/Real(2), +SquareRootTwo/Real(2)};
    }
    
    /// @brief Gets the -45 degree unit vector.
    /// @details This is the unit vector in the positive X and negative Y quadrant
    ///   where |X| == |Y|.
    static constexpr UnitVec2 GetBottomRight() noexcept
    {
        return UnitVec2{+SquareRootTwo/Real(2), -SquareRootTwo/Real(2)};
    }
    
    static constexpr UnitVec2 GetDefaultFallback() noexcept { return UnitVec2{}; }

    static UnitVec2 Get(const Real x, const Real y, Real& magnitude,
                        const UnitVec2 fallback = GetDefaultFallback()) noexcept
    {
        // XXX perhaps this should use std::hypot() instead like so:
        //    magnitude = std::hypot(x, y);
        magnitude = std::sqrt(x * x + y * y);
        return (std::isnormal(magnitude)) ? UnitVec2{x / magnitude, y / magnitude} : fallback;
    }

    /// @brief Gets the given angled unit vector.
    ///
    /// @note For angles that are meant to be at exact multiples of the quarter turn,
    ///   better accuracy will be had by using one of the four oriented unit
    ///   vector returning methods - for the right, top, left, bottom orientations.
    ///
    static UnitVec2 Get(const Angle angle) noexcept
    {
        return UnitVec2{std::cos(angle / Radian), std::sin(angle / Radian)};
    }

    constexpr UnitVec2() noexcept
    {
        // Intentionally empty.
    }

    constexpr auto GetX() const noexcept { return m_elems[0]; }

    constexpr auto GetY() const noexcept { return m_elems[1]; }

    constexpr auto cos() const noexcept { return m_elems[0]; }

    constexpr auto sin() const noexcept { return m_elems[1]; }

    constexpr inline UnitVec2 FlipXY() const noexcept { return UnitVec2{-GetX(), -GetY()}; }

    constexpr inline UnitVec2 FlipX() const noexcept { return UnitVec2{-GetX(), GetY()}; }

    constexpr inline UnitVec2 FlipY() const noexcept { return UnitVec2{GetX(), -GetY()}; }

    /// @brief Rotates the unit vector by the given amount.
    ///
    /// @param amount Expresses the angular difference from the right-ward oriented unit
    ///   vector to rotate this unit vector by.
    ///
    /// @return Result of rotating this unit vector by the given amount.
    ///
    constexpr inline UnitVec2 Rotate(UnitVec2 amount) const noexcept
    {
        return UnitVec2{GetX() * amount.GetX() - GetY() * amount.GetY(),
                        GetY() * amount.GetX() + GetX() * amount.GetY()};
    }

    /// @brief Gets a vector counter-clockwise (reverse-clockwise) perpendicular to this vector.
    /// @details This returns the unit vector (-y, x).
    /// @return A counter-clockwise 90-degree rotation of this vector.
    /// @sa GetFwdPerpendicular.
    constexpr inline UnitVec2 GetRevPerpendicular() const noexcept
    {
        // See http://mathworld.wolfram.com/PerpendicularVector.html
        return UnitVec2{-GetY(), GetX()};
    }

    /// @brief Gets a vector clockwise (forward-clockwise) perpendicular to this vector.
    /// @details This returns the unit vector (y, -x).
    /// @return A clockwise 90-degree rotation of this vector.
    /// @sa GetRevPerpendicular.
    constexpr inline UnitVec2 GetFwdPerpendicular() const noexcept
    {
        // See http://mathworld.wolfram.com/PerpendicularVector.html
        return UnitVec2{GetY(), -GetX()};
    }

    constexpr inline UnitVec2 operator-() const noexcept { return UnitVec2{-GetX(), -GetY()}; }

    constexpr inline UnitVec2 operator+() const noexcept { return UnitVec2{+GetX(), +GetY()}; }

    constexpr inline UnitVec2 Absolute() const noexcept
    {
        return UnitVec2{std::abs(GetX()), std::abs(GetY())};
    }

private:
    constexpr UnitVec2(value_type x, value_type y) noexcept : m_elems{x, y}
    {
        // Intentionally empty.
    }

    value_type m_elems[2] = { value_type{0}, value_type{0} };
};

/// @brief Gets the "X-axis".
constexpr inline UnitVec2 GetXAxis(UnitVec2 rot) noexcept { return rot; }

/// @brief Gets the "Y-axis".
/// @note This is the reverse perpendicular vector of the given unit vector.
constexpr inline UnitVec2 GetYAxis(UnitVec2 rot) noexcept { return rot.GetRevPerpendicular(); }

constexpr inline bool operator==(const UnitVec2 a, const UnitVec2 b) noexcept
{
    return (a.GetX() == b.GetX()) && (a.GetY() == b.GetY());
}

constexpr inline bool operator!=(const UnitVec2 a, const UnitVec2 b) noexcept
{
    return (a.GetX() != b.GetX()) || (a.GetY() != b.GetY());
}

/// @brief Gets a vector counter-clockwise (reverse-clockwise) perpendicular to the
///   given vector.
/// @details This takes a vector of form (x, y) and returns the vector (-y, x).
/// @param vector Vector to return a counter-clockwise perpendicular equivalent for.
/// @return A counter-clockwise 90-degree rotation of the given vector.
/// @sa GetFwdPerpendicular.
constexpr inline UnitVec2 GetRevPerpendicular(const UnitVec2 vector) noexcept
{
    return vector.GetRevPerpendicular();
}

/// @brief Gets a vector clockwise (forward-clockwise) perpendicular to the given vector.
/// @details This takes a vector of form (x, y) and returns the vector (y, -x).
/// @param vector Vector to return a clockwise perpendicular equivalent for.
/// @return A clockwise 90-degree rotation of the given vector.
/// @sa GetRevPerpendicular.
constexpr inline UnitVec2 GetFwdPerpendicular(const UnitVec2 vector) noexcept
{
    return vector.GetFwdPerpendicular();
}

/// @brief Rotates a unit vector by the angle expressed by the second unit vector.
/// @return Unit vector for the angle that's the sum of the two angles expressed by
///   the input unit vectors.
constexpr inline UnitVec2 Rotate(const UnitVec2 vector, const UnitVec2& angle) noexcept
{
    return vector.Rotate(angle);
}

/// @brief Inverse rotates a vector.
constexpr inline UnitVec2 InverseRotate(const UnitVec2 vector, const UnitVec2& angle) noexcept
{
    return vector.Rotate(angle.FlipY());
}
    
template <> constexpr UnitVec2 GetInvalid() noexcept { return UnitVec2{}; }

template <> constexpr inline bool IsValid(const UnitVec2& value) noexcept
{
    return IsValid(value.GetX()) && IsValid(value.GetY()) && (value != UnitVec2::GetZero());
}

} // namespace playrho

namespace std
{

template <size_t I>
constexpr playrho::UnitVec2::value_type get(playrho::UnitVec2 v) noexcept
{
    static_assert(I < 2, "Index out of bounds in std::get<> (playrho::UnitVec2)");
    switch (I)
    {
        case 0: return v.GetX();
        case 1: return v.GetY();
    }
}

template <>
constexpr playrho::UnitVec2::value_type get<0>(playrho::UnitVec2 v) noexcept
{
    return v.GetX();
}

template <>
constexpr playrho::UnitVec2::value_type get<1>(playrho::UnitVec2 v) noexcept
{
    return v.GetY();
}
}

#endif /* UnitVec2_hpp */
