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

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>

namespace playrho {

/// @brief Disk shape.
///
/// @details A disk shape "is the region in a plane bounded by a circle". This is a
///   two-dimensional solid round shape. This used to be called the circle shape but
///   that's now used for hollow round shapes.
///
/// @sa https://en.wikipedia.org/wiki/Disk_(mathematics)
/// @sa CircleShape.
///
/// @ingroup PartsGroup
///
class DiskShape : public Shape
{
public:
    
    /// @brief Gets the default radius.
    static constexpr Length GetDefaultRadius() noexcept
    {
        return DefaultLinearSlop * 2;
    }

    /// @brief Configuration data for disk shapes.
    struct Conf: public ShapeDefBuilder<Conf>
    {
        constexpr Conf(): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(GetDefaultRadius())}
        {
            // Intentionally empty.
        }
        
        /// @brief Uses the given value as the location.
        constexpr Conf& UseLocation(Length2D value) noexcept
        {
            location = value;
            return *this;
        }

        /// @brief Location for the disk shape to be centered at.
        Length2D location = Length2D{};
    };

    /// @brief Gets the default configuration.
    static constexpr Conf GetDefaultConf() noexcept;

    /// Initializing constructor.
    explicit DiskShape(const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf}, m_location{conf.location}
    {
        // Intentionally empty.
    }

    /// @brief Initializing constructor.
    explicit DiskShape(const Length radius, const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf}, m_location{conf.location}
    {
        SetVertexRadius(radius);
    }

    /// @brief Copy constructor.
    DiskShape(const DiskShape& other) = default;

    /// @brief Move constructor.
    DiskShape(DiskShape&& other) = default;
    
    ~DiskShape() override = default;

    /// @brief Copy assignment operator.
    DiskShape& operator= (const DiskShape& other) = default;
    
    /// @brief Move assignment operator.
    DiskShape& operator= (DiskShape&& other) = default;
    
    ChildCounter GetChildCount() const noexcept override;

    DistanceProxy GetChild(ChildCounter index) const override;

    MassData GetMassData() const noexcept override;
    
    void Accept(ShapeVisitor& visitor) const override;

    /// @brief Gets the "radius" of the shape.
    /// @return Non-negative radius.
    NonNegative<Length> GetRadius() const noexcept { return GetVertexRadius(); }
    
    /// @brief Sets the radius to the given value.
    void SetRadius(Length radius) noexcept
    {
        SetVertexRadius(radius);
    }

    /// @brief Gets the location of the center of this circle shape.
    /// @return The origin (0, 0) unless explicitly set otherwise on construction or via
    ///   the set location method.
    /// @sa SetPosition.
    Length2D GetLocation() const noexcept { return m_location; }
    
    /// @brief Sets the location to the given value.
    void SetLocation(const Length2D value) noexcept { m_location = value; }

private:
    /// Location of the shape as initialized on construction or as assigned using the
    ///   SetPosition method.
    Length2D m_location = Length2D{};
};

constexpr DiskShape::Conf DiskShape::GetDefaultConf() noexcept
{
    return Conf{};
}

inline ChildCounter DiskShape::GetChildCount() const noexcept
{
    return 1;
}

inline DistanceProxy DiskShape::GetChild(ChildCounter index) const
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{GetVertexRadius(), 1, &m_location, nullptr};
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_DISKSHAPE_HPP
