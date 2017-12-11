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

#ifndef PLAYRHO_COLLISION_SHAPES_DISKSHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_DISKSHAPE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>

namespace playrho {

/// @brief Disk shape configuration.
///
/// @details A disk shape "is the region in a plane bounded by a circle". This is a
///   two-dimensional solid round shape. This used to be called the circle shape but
///   that's now used for hollow round shapes.
///
/// @sa https://en.wikipedia.org/wiki/Disk_(mathematics)
///
/// @ingroup PartsGroup
///
class DiskShapeConf: public ShapeDefBuilder<DiskShapeConf>
{
public:
    /// @brief Gets the default radius.
    static PLAYRHO_CONSTEXPR inline Length GetDefaultRadius() noexcept
    {
        return DefaultLinearSlop * 2;
    }

    PLAYRHO_CONSTEXPR inline DiskShapeConf(): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(GetDefaultRadius())}
    {
        // Intentionally empty.
    }
    
    PLAYRHO_CONSTEXPR inline DiskShapeConf(Length radius): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(radius)}
    {
        // Intentionally empty.
    }

    /// @brief Uses the given value as the location.
    PLAYRHO_CONSTEXPR inline DiskShapeConf& UseLocation(Length2 value) noexcept
    {
        location = value;
        return *this;
    }
    
    /// @brief Sets the radius to the given value.
    PLAYRHO_CONSTEXPR inline DiskShapeConf& SetRadius(Length radius) noexcept
    {
        vertexRadius = radius;
        return *this;
    }
    
    /// @brief Sets the location to the given value.
    PLAYRHO_CONSTEXPR inline DiskShapeConf& SetLocation(const Length2 value) noexcept
    {
        location = value;
        return *this;
    }
    
    NonNegative<Length> GetRadius() const noexcept
    {
        return vertexRadius;
    }
    
    Length2 GetLocation() const noexcept
    {
        return location;
    }
    
    /// @brief Location for the disk shape to be centered at.
    Length2 location = Length2{};
};

// Free functions...

PLAYRHO_CONSTEXPR inline ChildCounter GetChildCount(const DiskShapeConf&) noexcept
{
    return 1;
}

inline DistanceProxy GetChild(const DiskShapeConf& arg, ChildCounter index)
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{arg.vertexRadius, 1, &arg.location, nullptr};
}

inline MassData GetMassData(const DiskShapeConf& arg) noexcept
{
    return playrho::GetMassData(arg.vertexRadius, arg.density, arg.location);
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_DISKSHAPE_HPP
