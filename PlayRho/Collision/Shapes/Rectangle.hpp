/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP
#define PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Units.hpp>
#include <PlayRho/Common/InvalidArgument.hpp>
#include <PlayRho/Common/NonNegative.hpp>
#include <PlayRho/Common/Finite.hpp>
#include <PlayRho/Common/Settings.hpp>

#include <PlayRho/Dynamics/Filter.hpp>

#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>

#include <array>

namespace playrho::d2 {

/// @brief Experimental rectangle shape configuration.
/// @todo Consider using template mixins.
template <int W, int H, int D, int F = 2, int R = 0>
struct Rectangle
{
    /// @brief Density of the shape.
    static constexpr auto density = NonNegative<AreaDensity>{
        D * KilogramPerSquareMeter};

    /// @brief Friction of the shape.
    static constexpr auto friction = NonNegative<Real>{Real(F) / Real{10}};

    /// @brief Restitution of the shape.
    static inline const auto restitution = Finite<Real>(R);

    /// @brief Width of the rectangle.
    static constexpr auto width = W * Meter;

    /// @brief Height of the rectangle.
    static constexpr auto height = H * Meter;

    /// @brief Vertex radius of the shape.
    static constexpr auto vertexRadius = DefaultLinearSlop * 2;

    /// @brief Normals of the rectangle.
    static constexpr auto normals = std::array<UnitVec, 4u>{
        UnitVec::GetRight(),
        UnitVec::GetTop(),
        UnitVec::GetLeft(),
        UnitVec::GetBottom()
    };

    /// @brief Vertices of the rectangle.
    static constexpr auto vertices = std::array<Length2, 4u>{
        Length2{+width/2, -height/2},
        Length2{+width/2, +height/2},
        Length2{-width/2, +height/2},
        Length2{-width/2, -height/2}
    };
};

/// @brief Gets the "child" count for the given shape configuration.
/// @return 1.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr ChildCounter GetChildCount(const Rectangle<W, H, D, F, R>&) noexcept
{
    return 1;
}

/// @brief Gets the "child" shape for the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
DistanceProxy GetChild(const Rectangle<W, H, D, F, R>& arg, ChildCounter index)
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{arg.vertexRadius, static_cast<VertexCounter>(size(arg.vertices)),
        data(arg.vertices), data(arg.normals)};
}

/// @brief Gets the density of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr NonNegative<AreaDensity> GetDensity(const Rectangle<W, H, D, F, R>& arg) noexcept
{
    return arg.density;
}

/// @brief Gets the restitution of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr Finite<Real> GetRestitution(const Rectangle<W, H, D, F, R>& arg) noexcept
{
    return arg.restitution;
}

/// @brief Gets the friction of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr NonNegative<Real> GetFriction(const Rectangle<W, H, D, F, R>& arg) noexcept
{
    return arg.friction;
}

/// @brief Gets the filter of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr Filter GetFilter(const Rectangle<W, H, D, F, R>&) noexcept
{
    return Filter{};
}

/// @brief Gets the is-sensor state of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr bool IsSensor(const Rectangle<W, H, D, F, R>&) noexcept
{
    return false;
}

/// @brief Gets the vertex radius of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
constexpr NonNegative<Length> GetVertexRadius(const Rectangle<W, H, D, F, R>& arg) noexcept
{
    return arg.vertexRadius;
}

/// @brief Gets the vertex radius of the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
NonNegative<Length> GetVertexRadius(const Rectangle<W, H, D, F, R>& arg, ChildCounter) noexcept
{
    return GetVertexRadius(arg);
}

/// @brief Gets the mass data for the given shape configuration.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
MassData GetMassData(const Rectangle<W, H, D, F, R>& arg) noexcept
{
    return playrho::d2::GetMassData(arg.vertexRadius, arg.density, Span<const Length2>(arg.vertices));
}

/// @brief Transforms the given polygon configuration's vertices by the given
///   transformation matrix.
/// @relatedalso Rectangle
/// @see https://en.wikipedia.org/wiki/Transformation_matrix
template <int W, int H, int D, int F, int R>
void Transform(Rectangle<W, H, D, F, R>&, const Mat22& m)
{
    if (m != GetIdentity<Mat22>()) {
        throw InvalidArgument("transformation by non-identity matrix not supported");
    }
}

/// @brief Filter setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
void SetFilter(Rectangle<W, H, D, F, R>& arg, Filter value)
{
    if (value != GetFilter(arg)) {
        throw InvalidArgument("SetFilter by non-equivalent filter not supported");
    }
}

/// @brief Sensor setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
void SetSensor(Rectangle<W, H, D, F, R>& arg, bool value)
{
    if (value != IsSensor(arg)) {
        throw InvalidArgument("SetSensor by non-equivalent value not supported");
    }
}

/// @brief Friction setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
void SetFriction(Rectangle<W, H, D, F, R>& arg, Real value)
{
    if (value != GetFriction(arg)) {
        throw InvalidArgument("SetFriction by non-equivalent value not supported");
    }
}

/// @brief Restitution setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <int W, int H, int D, int F, int R>
void SetRestitution(Rectangle<W, H, D, F, R>& arg, Real value)
{
    if (value != GetRestitution(arg)) {
        throw InvalidArgument("SetRestitution by non-equivalent value not supported");
    }
}

/// @brief Equality operator.
template <int W1, int H1, int D1, int F1, int R1, int W2, int H2, int D2, int F2, int R2>
bool operator== (const Rectangle<W1, H1, D1, F1, R1>&,
                 const Rectangle<W2, H2, D2, F2, R2>&) noexcept
{
    return W1 == W2 && H1 == H2 && D1 == D2 && F1 == F2 && R1 == R2;
}

/// @brief Inequality operator.
template <int W1, int H1, int D1, int F1, int R1, int W2, int H2, int D2, int F2, int R2>
bool operator!= (const Rectangle<W1, H1, D1, F1, R1>& lhs,
                 const Rectangle<W2, H2, D2, F2, R2>& rhs) noexcept
{
    return !(lhs == rhs);
}

} // namespace playrho::d2

#endif // PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP
