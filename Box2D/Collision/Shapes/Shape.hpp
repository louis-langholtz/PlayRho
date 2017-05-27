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

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/MassData.hpp>
#include <Box2D/Common/BoundedValue.hpp>

namespace box2d {

class CircleShape;
class EdgeShape;
class PolygonShape;
class ChainShape;
class MultiShape;

/// Shape.
///
/// @details Shape is an abstract base class for shapes.
/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in World are created automatically when a Fixture
/// is created. Shapes may encapsulate one or more child shapes.
///
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
///
class Shape
{
public:
    struct Conf
    {
        constexpr Conf& UseVertexRadius(NonNegative<Length> value) noexcept;
        constexpr Conf& UseFriction(NonNegative<RealNum> value) noexcept;
        constexpr Conf& UseRestitution(Finite<RealNum> value) noexcept;
        constexpr Conf& UseDensity(NonNegative<Density> value) noexcept;

        /// Vertex radius.
        ///
        /// @details This is the radius from the vertex that the shape's "skin" should
        ///   extend outward by. While any edges - line segments between multiple vertices -
        ///   are straight, corners between them (the vertices) are rounded and treated
        ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
        ///   therefore will be more prone to rolling or having other shapes more prone
        ///   to roll off of them.
        ///
        /// @note This should be a value greater than zero.
        ///
        NonNegative<Length> vertexRadius = NonNegative<Length>{DefaultLinearSlop};

        /// Friction coefficient.
        ///
        /// @note This must be a value between 0 and +infinity.
        /// @note This is usually in the range [0,1].
        /// @note The square-root of the product of this value multiplied by a touching fixture's
        ///   friction becomes the friction coefficient for the contact.
        ///
        NonNegative<RealNum> friction = NonNegative<RealNum>{RealNum{2} / RealNum{10}};
        
        /// Restitution (elasticity) of the associated shape.
        ///
        /// @note This should be a valid finite value.
        /// @note This is usually in the range [0,1].
        ///
        Finite<RealNum> restitution = Finite<RealNum>{0};
        
        /// Density of the associated shape.
        ///
        /// @note This must be a non-negative value.
        /// @note Use 0 to indicate that the shape's associated mass should be 0.
        ///
        NonNegative<Density> density = NonNegative<Density>{0};
    };

    /// Visitor interface.
    ///
    /// @details Interface to inerit from for objects wishing to "visit" shapes.
    /// This uses the vistor design pattern.
    /// @sa https://en.wikipedia.org/wiki/Visitor_pattern .
    ///
    struct Visitor
    {
    public:
        virtual ~Visitor() = default;
        
        virtual void Visit(const CircleShape&)
        {
            // Intentionally empty (no-op).
        }

        virtual void Visit(const EdgeShape&)
        {
            // Intentionally empty (no-op).
        }

        virtual void Visit(const PolygonShape&)
        {
            // Intentionally empty (no-op).
        }

        virtual void Visit(const ChainShape&)
        {
            // Intentionally empty (no-op).
        }

        virtual void Visit(const MultiShape&)
        {
            // Intentionally empty (no-op).
        }
    };

    /// Default constructor is deleted.
    /// @details This is a base class that shouldn't ever be directly instantiated.
    Shape() = delete;

    /// Initializing constructor.
    ///
    Shape(const Conf& conf) noexcept:
        m_vertexRadius{conf.vertexRadius},
        m_density{conf.density},
        m_friction{conf.friction},
        m_restitution{conf.restitution}
    {
    }

    Shape(const Shape&) = default;

    virtual ~Shape() = default;

    /// Gets the number of child primitives of the shape.
    /// @return Positive non-zero count.
    virtual child_count_t GetChildCount() const noexcept = 0;

    /// Gets the child for the given index.
    /// @note The shape must remain in scope while the proxy is in use.
    virtual DistanceProxy GetChild(child_count_t index) const noexcept = 0;
    
    /// Tests a point for containment in this shape.
    /// @param xf the shape world transform.
    /// @param p a point in world coordinates.
    /// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
    virtual bool TestPoint(const Transformation& xf, const Length2D p) const noexcept = 0;

    /// Gets the mass properties of this shape using its dimensions and density.
    /// @return Mass data for this shape.
    virtual MassData GetMassData() const noexcept = 0;

    /// Accepts a visitor.
    virtual void Accept(Visitor& visitor) const = 0;
    
    /// @brief Gets the vertex radius.
    Length GetVertexRadius() const noexcept;

    /// Sets the vertex radius.
    ///
    /// @details This sets the radius from the vertex that the shape's "skin" should
    ///   extend outward by. While any edges - line segments between multiple vertices -
    ///   are straight, corners between them (the vertices) are rounded and treated
    ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
    ///   therefore will be more prone to rolling or having other shapes more prone
    ///   to roll off of them.
    ///
    /// @note This should be a value greater than zero.
    ///
    void SetVertexRadius(NonNegative<Length> vertexRadius) noexcept;

    /// Gets the density of this fixture.
    /// @return Non-negative density (in mass per area).
    Density GetDensity() const noexcept;

    /// Sets the density of this fixture.
    /// @note This will _not_ automatically adjust the mass of the body.
    ///   You must call Body::ResetMassData to update the body's mass.
    /// @param density Non-negative density (in mass per area).
    void SetDensity(NonNegative<Density> density) noexcept;
    
    /// @brief Gets the coefficient of friction.
    /// @return Value of 0 or higher.
    RealNum GetFriction() const noexcept;
    
    /// @brief Sets the coefficient of friction.
    /// @note This will _not_ change the friction of existing contacts.
    /// @param friction Zero or higher (non-negative) co-efficient of friction.
    void SetFriction(NonNegative<RealNum> friction) noexcept;
    
    /// Gets the coefficient of restitution.
    RealNum GetRestitution() const noexcept;
    
    /// Sets the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    void SetRestitution(Finite<RealNum> restitution) noexcept;

private:
    
    /// @brief Vertex radius.
    NonNegative<Length> m_vertexRadius;
    
    /// @brief Density.
    NonNegative<Density> m_density = NonNegative<Density>{KilogramPerSquareMeter * RealNum{0}};
    
    /// @brief Friction as a coefficient.
    NonNegative<RealNum> m_friction = NonNegative<RealNum>{RealNum{2} / RealNum{10}};

    /// @brief Restitution as a coefficient.
    Finite<RealNum> m_restitution = Finite<RealNum>{0};
};

constexpr inline Shape::Conf& Shape::Conf::UseVertexRadius(NonNegative<Length> value) noexcept
{
    vertexRadius = value;
    return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseFriction(NonNegative<RealNum> value) noexcept
{
    friction = value;
    return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseRestitution(Finite<RealNum> value) noexcept
{
    restitution = value;
    return *this;
}

constexpr inline Shape::Conf& Shape::Conf::UseDensity(NonNegative<Density> value) noexcept
{
    density = value;
    return *this;
}

inline Length Shape::GetVertexRadius() const noexcept
{
    return m_vertexRadius;
}

inline void Shape::SetVertexRadius(NonNegative<Length> vertexRadius) noexcept
{
    m_vertexRadius = vertexRadius;
}

inline Density Shape::GetDensity() const noexcept
{
    return m_density;
}

inline RealNum Shape::GetFriction() const noexcept
{
    return m_friction;
}

inline RealNum Shape::GetRestitution() const noexcept
{
    return m_restitution;
}

inline void Shape::SetDensity(NonNegative<Density> density) noexcept
{
    m_density = density;
}

inline void Shape::SetFriction(NonNegative<RealNum> friction) noexcept
{
    m_friction = friction;
}

inline void Shape::SetRestitution(Finite<RealNum> restitution) noexcept
{
    m_restitution = restitution;
}

// Free functions...

/// Gets the vertex radius of the given shape.
/// @details Gets the radius of every vertex of this shape.
/// This is used for collision handling.
/// @note This value should never be less than zero.
inline Length GetVertexRadius(const Shape& shape) noexcept
{
    return shape.GetVertexRadius();
}

} // namespace box2d

#endif
