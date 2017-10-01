/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_SHAPES_SHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_SHAPE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

/// @defgroup PartsGroup Shape Classes
/// @details These are classes that specify physical characteristics of: shape,
///   friction, density and restitution. They've historically been called shape classes
///   but are now &mdash; with the other properties like friction and density having been
///   moved into them &mdash; maybe better thought of as "parts".

class ShapeVisitor;
struct ShapeDef;

/// @brief A base abstract class for describing a type of shape.
///
/// @details This is a polymorphic abstract base class for shapes.
///   A shape is used for collision detection. You can create a shape however you like.
///   Shapes used for simulation in World are created automatically when a Fixture
///   is created. Shapes may encapsulate zero or more child shapes.
///
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
///
/// @ingroup PartsGroup
///
class Shape
{
public:
    
    virtual ~Shape() = default;

    /// @brief Gets the number of child primitives of the shape.
    /// @return Non-negative count.
    virtual ChildCounter GetChildCount() const noexcept = 0;

    /// @brief Gets the child for the given index.
    /// @param index Index to a child element of the shape. Value must be less
    ///   than the number of child primitives of the shape.
    /// @note The shape must remain in scope while the proxy is in use.
    /// @throws InvalidArgument if the given index is out of range.
    /// @sa GetChildCount
    virtual DistanceProxy GetChild(ChildCounter index) const = 0;
    
    /// @brief Gets the mass properties of this shape using its dimensions and density.
    /// @return Mass data for this shape.
    virtual MassData GetMassData() const noexcept = 0;

    /// @brief Accepts a visitor.
    virtual void Accept(ShapeVisitor& visitor) const = 0;
    
    /// @brief Gets the vertex radius.
    ///
    /// @details This gets the radius from the vertex that the shape's "skin" should
    ///   extend outward by. While any edges - line segments between multiple vertices -
    ///   are straight, corners between them (the vertices) are rounded and treated
    ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
    ///   therefore will be more prone to rolling or having other shapes more prone
    ///   to roll off of them. Here's an image of a PolygonShape with it's skin drawn:
    ///
    /// @image html SkinnedPolygon.png
    ///
    /// @note This must be a non-negative value.
    ///
    /// @sa SetVertexRadius
    ///
    NonNegative<Length> GetVertexRadius() const noexcept;

    /// @brief Sets the vertex radius.
    ///
    /// @details This sets the radius from the vertex that the shape's "skin" should
    ///   extend outward by. While any edges - line segments between multiple vertices -
    ///   are straight, corners between them (the vertices) are rounded and treated
    ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
    ///   therefore will be more prone to rolling or having other shapes more prone
    ///   to roll off of them.
    ///
    /// @note This should be a non-negative value.
    ///
    /// @sa GetVertexRadius
    ///
    void SetVertexRadius(NonNegative<Length> vertexRadius) noexcept;

    /// @brief Gets the density of this fixture.
    /// @return Non-negative density (in mass per area).
    NonNegative<Density> GetDensity() const noexcept;

    /// @brief Sets the density of this fixture.
    /// @note This will _not_ automatically adjust the mass of the body.
    ///   You must call Body::ResetMassData to update the body's mass.
    /// @param density Non-negative density (in mass per area).
    void SetDensity(NonNegative<Density> density) noexcept;
    
    /// @brief Gets the coefficient of friction.
    /// @return Value of 0 or higher.
    Real GetFriction() const noexcept;
    
    /// @brief Sets the coefficient of friction.
    /// @note This will _not_ change the friction of existing contacts.
    /// @param friction Zero or higher (non-negative) co-efficient of friction.
    void SetFriction(NonNegative<Real> friction) noexcept;
    
    /// @brief Gets the coefficient of restitution.
    Real GetRestitution() const noexcept;
    
    /// @brief Sets the coefficient of restitution.
    /// @note This will _not_ change the restitution of existing contacts.
    void SetRestitution(Finite<Real> restitution) noexcept;

protected:

    /// @brief Default constructor.
    /// @details This is a base class that shouldn't ever be directly instantiated.
    Shape() = default;
    
    /// @brief Initializing constructor.
    ///
    explicit Shape(const ShapeDef& conf) noexcept;
    
    /// @brief Copy constructor.
    Shape(const Shape& other) = default;
    
    /// @brief Move constructor.
    Shape(Shape&& other) = default;

    /// @brief Copy assignment operator.
    Shape& operator= (const Shape& other) = default;
    
    /// @brief Move assignment operator.
    Shape& operator= (Shape&& other) = default;

private:
    
    /// @brief Vertex radius.
    NonNegative<Length> m_vertexRadius = NonNegative<Length>{Real(0) * Meter};
    
    /// @brief Density.
    NonNegative<Density> m_density = NonNegative<Density>{KilogramPerSquareMeter * Real{0}};
    
    /// @brief Friction as a coefficient.
    NonNegative<Real> m_friction = NonNegative<Real>{Real{2} / Real{10}};

    /// @brief Restitution as a coefficient.
    Finite<Real> m_restitution = Finite<Real>{0};
};

inline NonNegative<Length> Shape::GetVertexRadius() const noexcept
{
    return m_vertexRadius;
}

inline void Shape::SetVertexRadius(NonNegative<Length> vertexRadius) noexcept
{
    m_vertexRadius = vertexRadius;
}

inline NonNegative<Density> Shape::GetDensity() const noexcept
{
    return m_density;
}

inline Real Shape::GetFriction() const noexcept
{
    return m_friction;
}

inline Real Shape::GetRestitution() const noexcept
{
    return m_restitution;
}

inline void Shape::SetDensity(NonNegative<Density> density) noexcept
{
    m_density = density;
}

inline void Shape::SetFriction(NonNegative<Real> friction) noexcept
{
    m_friction = friction;
}

inline void Shape::SetRestitution(Finite<Real> restitution) noexcept
{
    m_restitution = restitution;
}

// Free functions...

/// @brief Gets the vertex radius of the given shape.
/// @details Gets the radius of every vertex of this shape.
/// This is used for collision handling.
/// @note This value should never be less than zero.
/// @relatedalso Shape
/// @sa Shape::GetVertexRadius
inline NonNegative<Length> GetVertexRadius(const Shape& shape) noexcept
{
    return shape.GetVertexRadius();
}

/// @brief Test a point for containment in the given shape.
/// @param shape Shape to use for test.
/// @param point Point in local coordinates.
/// @return <code>true</code> if the given point is contained by the given shape,
///   <code>false</code> otherwise.
/// @relatedalso Shape
/// @ingroup TestPointGroup
bool TestPoint(const Shape& shape, Length2D point) noexcept;

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_SHAPE_HPP
