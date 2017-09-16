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

#ifndef PLAYRHO_COLLISION_SHAPES_SHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_SHAPE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

class DiskShape;
class EdgeShape;
class PolygonShape;
class ChainShape;
class MultiShape;

/// @brief A base abstract class for describing a type of shape.
///
/// @details This is a polymorphic abstract base class for shapes.
/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in World are created automatically when a Fixture
/// is created. Shapes may encapsulate one or more child shapes.
///
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
///
/// @sa ShapeFreeFunctions
///
class Shape
{
public:
    
    /// @brief Configuration for initializing shapes.
    /// @note This is a nested base value class for initializing shapes.
    struct Conf
    {
        /// @brief Vertex radius.
        ///
        /// @details This is the radius from the vertex that the shape's "skin" should
        ///   extend outward by. While any edges - line segments between multiple vertices -
        ///   are straight, corners between them (the vertices) are rounded and treated
        ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
        ///   therefore will be more prone to rolling or having other shapes more prone
        ///   to roll off of them.
        ///
        /// @note This should be a non-negative value.
        ///
        NonNegative<Length> vertexRadius = NonNegative<Length>{DefaultLinearSlop};

        /// @brief Friction coefficient.
        ///
        /// @note This must be a value between 0 and +infinity. It is safer however to
        ///   keep the value below the square root of the max value of a Real.
        /// @note This is usually in the range [0,1].
        /// @note The square-root of the product of this value multiplied by a touching
        ///   fixture's friction becomes the friction coefficient for the contact.
        ///
        NonNegative<Real> friction = NonNegative<Real>{Real{2} / Real{10}};
        
        /// @brief Restitution (elasticity) of the associated shape.
        ///
        /// @note This should be a valid finite value.
        /// @note This is usually in the range [0,1].
        ///
        Finite<Real> restitution = Finite<Real>{0};
        
        /// @brief Density of the associated shape.
        ///
        /// @note This must be a non-negative value.
        /// @note Use 0 to indicate that the shape's associated mass should be 0.
        ///
        NonNegative<Density> density = NonNegative<Density>{0};
    };

    /// @brief Builder configuration structure.
    /// @details This is a builder structure of chainable methods for building a shape
    ///   configuration.
    /// @note This is a templated nested value class for initializing shapes that
    ///   uses the Curiously Recurring Template Pattern (CRTP) to provide method chaining
    ///   via static polymorphism.
    /// @sa https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
    template <typename ConcreteConf>
    struct Builder: Conf
    {
        /// @brief Uses the given vertex radius.
        constexpr ConcreteConf& UseVertexRadius(NonNegative<Length> value) noexcept;

        /// @brief Uses the given friction.
        constexpr ConcreteConf& UseFriction(NonNegative<Real> value) noexcept;
        
        /// @brief Uses the given restitution.
        constexpr ConcreteConf& UseRestitution(Finite<Real> value) noexcept;
        
        /// @brief Uses the given density.
        constexpr ConcreteConf& UseDensity(NonNegative<Density> value) noexcept;
    };

    class Visitor;

    /// @brief Default constructor is deleted.
    /// @details This is a base class that shouldn't ever be directly instantiated.
    Shape() = delete;

    /// @brief Initializing constructor.
    ///
    explicit Shape(const Conf& conf) noexcept:
        m_vertexRadius{conf.vertexRadius},
        m_density{conf.density},
        m_friction{conf.friction},
        m_restitution{conf.restitution}
    {
        // Intentionally empty.
    }

    /// @brief Copy constructor.
    Shape(const Shape&) = default;

    virtual ~Shape() = default;

    /// @brief Gets the number of child primitives of the shape.
    /// @return Positive non-zero count.
    virtual ChildCounter GetChildCount() const noexcept = 0;

    /// @brief Gets the child for the given index.
    /// @note The shape must remain in scope while the proxy is in use.
    /// @throws InvalidArgument if the index is out of range.
    virtual DistanceProxy GetChild(ChildCounter index) const = 0;
    
    /// @brief Gets the mass properties of this shape using its dimensions and density.
    /// @return Mass data for this shape.
    virtual MassData GetMassData() const noexcept = 0;

    /// @brief Accepts a visitor.
    virtual void Accept(Visitor& visitor) const = 0;
    
    /// @brief Gets the vertex radius.
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

template <typename ConcreteConf>
constexpr ConcreteConf&
Shape::Builder<ConcreteConf>::UseVertexRadius(NonNegative<Length> value) noexcept
{
    vertexRadius = value;
    return static_cast<ConcreteConf&>(*this);
}

template <typename ConcreteConf>
constexpr ConcreteConf&
Shape::Builder<ConcreteConf>::UseFriction(NonNegative<Real> value) noexcept
{
    friction = value;
    return static_cast<ConcreteConf&>(*this);
}

template <typename ConcreteConf>
constexpr ConcreteConf&
Shape::Builder<ConcreteConf>::UseRestitution(Finite<Real> value) noexcept
{
    restitution = value;
    return static_cast<ConcreteConf&>(*this);
}

template <typename ConcreteConf>
constexpr ConcreteConf&
Shape::Builder<ConcreteConf>::UseDensity(NonNegative<Density> value) noexcept
{
    density = value;
    return static_cast<ConcreteConf&>(*this);
}

/// @brief Visitor interface.
///
/// @details Interface to inerit from for objects wishing to "visit" shapes.
///   This uses the vistor design pattern.
/// @sa https://en.wikipedia.org/wiki/Visitor_pattern .
///
class Shape::Visitor
{
public:
    virtual ~Visitor() = default;
    
    /// @brief Visits a DiskShape.
    virtual void Visit(const DiskShape& /*shape*/)
    {
        visited = true;
    }
    
    /// @brief Visits an EdgeShape.
    virtual void Visit(const EdgeShape& /*shape*/)
    {
        visited = true;
    }
    
    /// @brief Visits a PolygonShape.
    virtual void Visit(const PolygonShape& /*shape*/)
    {
        visited = true;
    }
    
    /// @brief Visits a ChainShape.
    virtual void Visit(const ChainShape& /*shape*/)
    {
        visited = true;
    }
    
    /// @brief Visits a MultiShape.
    virtual void Visit(const MultiShape& /*shape*/)
    {
        visited = true;
    }
    
    /// @brief Is base visited.
    bool IsBaseVisited() const noexcept
    {
        return visited;
    }
    
private:
    bool visited = false;
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

/// @defgroup ShapeFreeFunctions Shape free functions.
/// @details A collection of non-member, non-friend functions that operate on Shape objects.
/// @sa Shape.
/// @{

/// @brief Gets the vertex radius of the given shape.
/// @details Gets the radius of every vertex of this shape.
/// This is used for collision handling.
/// @note This value should never be less than zero.
inline NonNegative<Length> GetVertexRadius(const Shape& shape) noexcept
{
    return shape.GetVertexRadius();
}

/// @brief Test a point for containment in the given shape.
/// @param shape Shape to use for test.
/// @param point Point in local coordinates.
/// @return <code>true</code> if the given point is contained by the given shape,
///   <code>false</code> otherwise.
bool TestPoint(const Shape& shape, Length2D point) noexcept;

/// @}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_SHAPE_HPP
