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
#include <memory>
#include <functional>
#include <utility>

namespace playrho {

/// @brief Gets the friction of the given shape.
Real GetFriction(const Shape& shape) noexcept;

/// @brief Gets the restitution value of the given shape.
Real GetRestitution(const Shape& shape) noexcept;

/// @brief Gets the density of the given shape.
NonNegative<AreaDensity> GetDensity(const Shape& shape) noexcept;

/// @brief Gets the vertex radius of the given shape.
NonNegative<Length> GetVertexRadius(const Shape& shape) noexcept;

/// @defgroup PartsGroup Shape Classes
/// @brief Classes for configuring shapes with material properties.
/// @details These are classes that specify physical characteristics of: shape,
///   friction, density and restitution. They've historically been called shape classes
///   but are now &mdash; with the other properties like friction and density having been
///   moved into them &mdash; maybe better thought of as "parts".

/// @brief Shape.
///
/// @details A shape is used for collision detection. You can create a shape however you like.
///   Shapes used for simulation in <code>World</code> are created automatically when a
///   <code>Fixture</code> is created. Shapes may encapsulate zero or more child shapes.
///
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
///
/// @ingroup PartsGroup
///
/// @sa Fixture
///
class Shape
{
public:
    
    /// @brief Visitor type alias for underlying shape configuration.
    using Visitor = std::function<void(const std::type_info& ti, const void* data)>;

    /// @brief Default constructor.
    /// @details This is a base class that shouldn't ever be directly instantiated.
    Shape() = delete;

    /// @brief Initializing constructor.
    template <typename T>
    Shape(T v): m_self{std::make_shared<Model<T>>(std::move(v))}
    {}
    
    /// @brief Copy constructor.
    Shape(const Shape& other) = default;
    
    /// @brief Move constructor.
    Shape(Shape&& other) = default;
    
    /// @brief Copy assignment operator.
    Shape& operator= (const Shape& other) = default;
    
    /// @brief Move assignment operator.
    Shape& operator= (Shape&& other) = default;

    /// @brief Gets the number of child primitives of the shape.
    /// @return Non-negative count.
    friend ChildCounter GetChildCount(const Shape& shape) noexcept
    {
        return shape.m_self->GetChildCount_();
    }

    /// @brief Gets the "child" for the given index.
    /// @param shape Shape to get "child" shape of.
    /// @param index Index to a child element of the shape. Value must be less
    ///   than the number of child primitives of the shape.
    /// @note The shape must remain in scope while the proxy is in use.
    /// @throws InvalidArgument if the given index is out of range.
    /// @sa GetChildCount
    friend DistanceProxy GetChild(const Shape& shape, ChildCounter index)
    {
        return shape.m_self->GetChild_(index);
    }
    
    /// @brief Gets the mass properties of this shape using its dimensions and density.
    /// @return Mass data for this shape.
    friend MassData2D GetMassData(const Shape& shape) noexcept
    {
        return shape.m_self->GetMassData_();
    }
    
    /// @brief Gets the vertex radius.
    ///
    /// @details This gets the radius from the vertex that the shape's "skin" should
    ///   extend outward by. While any edges - line segments between multiple vertices -
    ///   are straight, corners between them (the vertices) are rounded and treated
    ///   as rounded. Shapes with larger vertex radiuses compared to edge lengths
    ///   therefore will be more prone to rolling or having other shapes more prone
    ///   to roll off of them. Here's an image of a shape configured via a
    ///   <code>PolygonShapeConf</code> with it's skin drawn:
    ///
    /// @image html SkinnedPolygon.png
    ///
    /// @note This must be a non-negative value.
    ///
    /// @sa UseVertexRadius
    ///
    friend NonNegative<Length> GetVertexRadius(const Shape& shape) noexcept
    {
        return shape.m_self->GetVertexRadius_();
    }
    
    /// @brief Gets the coefficient of friction.
    /// @return Value of 0 or higher.
    friend Real GetFriction(const Shape& shape) noexcept
    {
        return shape.m_self->GetFriction_();
    }
    
    /// @brief Gets the coefficient of restitution.
    friend Real GetRestitution(const Shape& shape) noexcept
    {
        return shape.m_self->GetRestitution_();
    }

    /// @brief Gets the density of this fixture.
    /// @return Non-negative density (in mass per area).
    friend NonNegative<AreaDensity> GetDensity(const Shape& shape) noexcept
    {
        return shape.m_self->GetDensity_();
    }
    
    /// @brief Gets a pointer to the underlying data.
    /// @note Provided for introspective purposes like visitation.
    /// @note Generally speaking, try to avoid using this method unless there's
    ///   no other way to access the underlying data.
    friend const void* GetData(const Shape& shape) noexcept
    {
        return shape.m_self->GetData_();
    }
    
    /// @brief Accepts a visitor.
    /// @details This is the "accept" method definition of a "visitor design pattern"
    ///   for doing shape configuration specific types of processing for a constant shape.
    /// @sa https://en.wikipedia.org/wiki/Visitor_pattern
    friend void Accept(const Shape& shape, const Visitor& visitor)
    {
        const auto self = shape.m_self;
        visitor(self->GetTypeInfo_(), self->GetData_());
    }
    
    /// @brief Equality operator for shape to shape comparisons.
    friend bool operator== (const Shape& lhs, const Shape& rhs) noexcept
    {
        return lhs.m_self == rhs.m_self || *lhs.m_self == *rhs.m_self;
    }

    /// @brief Inequality operator for shape to shape comparisons.
    friend bool operator!= (const Shape& lhs, const Shape& rhs) noexcept
    {
        return !(lhs == rhs);
    }

private:

    /// @brief Internal shape configuration concept.
    /// @note Provides an interface for runtime polymorphism for shape configuration.
    struct Concept
    {
        virtual ~Concept() = default;

        /// @brief Gets the "child" count.
        virtual ChildCounter GetChildCount_() const noexcept = 0;
        
        /// @brief Gets the "child" specified by the given index.
        virtual DistanceProxy GetChild_(ChildCounter index) const = 0;
        
        /// @brief Gets the mass data.
        virtual MassData2D GetMassData_() const noexcept = 0;
        
        /// @brief Gets the vertex radius.
        virtual NonNegative<Length> GetVertexRadius_() const noexcept = 0;

        /// @brief Gets the density.
        virtual NonNegative<AreaDensity> GetDensity_() const noexcept = 0;
        
        /// @brief Gets the friction.
        virtual Real GetFriction_() const noexcept = 0;
        
        /// @brief Gets the restitution.
        virtual Real GetRestitution_() const noexcept = 0;
        
        /// @brief Equality checking method.
        virtual bool IsEqual_(const Concept& other) const noexcept = 0;
        
        /// @brief Gets the type information of the underlying configuration.
        virtual const std::type_info& GetTypeInfo_() const = 0;
        
        /// @brief Gets the data for the underlying configuration.
        virtual const void* GetData_() const noexcept = 0;
        
        /// @brief Equality operator.
        friend bool operator== (const Concept& lhs, const Concept &rhs) noexcept
        {
            return &lhs == &rhs || lhs.IsEqual_(rhs);
        }
        
        /// @brief Inequality operator.
        friend bool operator!= (const Concept& lhs, const Concept &rhs) noexcept
        {
            return !(lhs == rhs);
        }
    };

    /// @brief Internal model configuration concept.
    /// @note Provides an implementation for runtime polymorphism for shape configuration.
    template <typename T>
    struct Model final: Concept
    {
        /// @brief Type alias for the type of the data held.
        using data_type = T;

        /// @brief Initializing constructor.
        Model(T arg): data{std::move(arg)} {}
        
        ChildCounter GetChildCount_() const noexcept override
        {
            return GetChildCount(data);
        }

        DistanceProxy GetChild_(ChildCounter index) const override
        {
            return GetChild(data, index);
        }

        MassData2D GetMassData_() const noexcept override
        {
            return GetMassData(data);
        }
        
        NonNegative<Length> GetVertexRadius_() const noexcept override
        {
            return GetVertexRadius(data);
        }
        
        NonNegative<AreaDensity> GetDensity_() const noexcept override
        {
            return GetDensity(data);
        }
        
        Real GetFriction_() const noexcept override
        {
            return GetFriction(data);
        }
        
        Real GetRestitution_() const noexcept override
        {
            return GetRestitution(data);
        }
        
        bool IsEqual_(const Concept& other) const noexcept override
        {
            return (GetTypeInfo_() == other.GetTypeInfo_()) &&
            (data == *static_cast<const T*>(other.GetData_()));
        }
        
        const std::type_info& GetTypeInfo_() const override
        {
            return typeid(data_type);
        }
        
        const void* GetData_() const noexcept override
        {
            // Note address of "data" not necessarily same as address of "this" since
            // base class is virtual.
            return &data;
        }

        T data; ///< Data.
    };

    std::shared_ptr<const Concept> m_self; ///< Self shared pointer.
};

// Free functions...

/// @brief Test a point for containment in the given shape.
/// @param shape Shape to use for test.
/// @param point Point in local coordinates.
/// @return <code>true</code> if the given point is contained by the given shape,
///   <code>false</code> otherwise.
/// @relatedalso Shape
/// @ingroup TestPointGroup
bool TestPoint(const Shape& shape, Length2 point) noexcept;

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_SHAPE_HPP
