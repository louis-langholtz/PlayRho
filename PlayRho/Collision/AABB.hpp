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

#ifndef PLAYRHO_COLLISION_AABB_HPP
#define PLAYRHO_COLLISION_AABB_HPP

/// @file
/// Declaration of the AABB class and free functions that return instances of it.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/ValueRange.hpp>

namespace playrho {

    class Shape;
    class Fixture;
    class Body;
    class DistanceProxy;
    
    /// @brief Axis Aligned Bounding Box.
    ///
    /// @details This is a concrete value class for an axis aligned bounding box (AABB)
    ///   which is a type of bounding volume.
    ///
    /// @note This class satisfies at least the following concepts: all the basic concepts,
    ///   EqualityComparable, and Swappable.
    /// @note This class is composed of &mdash; as in contains and owns &mdash two
    ///   <code>ValueRange<Length></code> variables.
    /// @note Non-defaulted methods of this class are marked noexcept and expect that
    ///   the Length type doesn't throw.
    /// @note This data structure is 16-bytes large (on at least one 64-bit platform).
    ///
    /// @sa https://en.wikipedia.org/wiki/Bounding_volume
    /// @sa http://en.cppreference.com/w/cpp/concept
    ///
    struct AABB
    {
        /// @brief Default constructor.
        /// @details Constructs an "unset" AABB.
        /// @note If an unset AABB is added to another AABB, the result will be the other AABB.
        constexpr AABB() = default;
        
        /// @brief Initializing copy constructor.
        constexpr AABB(const ValueRange<Length>& x, const ValueRange<Length>& y) noexcept:
            rangeX{x}, rangeY{y}
        {
            // Intentionally empty.
        }

        /// @brief Initializing move constructor.
        constexpr AABB(ValueRange<Length>&& x, ValueRange<Length>&& y) noexcept:
            rangeX{x}, rangeY{y}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor for a single point.
        /// @param p Point location to initialize this AABB with.
        /// @post <code>rangeX</code> will have its min and max values both set to the
        ///   given point's X value.
        /// @post <code>rangeY</code> will have its min and max values both set to the
        ///   given point's Y value.
        constexpr explicit AABB(const Length2D p) noexcept:
            rangeX{GetX(p)}, rangeY{GetY(p)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor for two points.
        /// @param a Point location "A" to initialize this AABB with.
        /// @param b Point location "B" to initialize this AABB with.
        constexpr AABB(const Length2D a, const Length2D b) noexcept:
            rangeX{GetX(a), GetX(b)}, rangeY{GetY(a), GetY(b)}
        {
            // Intentionally empty.
        }
        
        /// @brief Gets the lower bound.
        constexpr Length2D GetLowerBound() const noexcept
        {
            return Length2D{rangeX.GetMin(), rangeY.GetMin()};
        }
        
        /// @brief Gets the upper bound.
        constexpr Length2D GetUpperBound() const noexcept
        {
            return Length2D{rangeX.GetMax(), rangeY.GetMax()};
        }

        /// @brief Checks whether this AABB fully contains the given AABB.
        constexpr bool Contains(const AABB& aabb) const noexcept
        {
            return IsWithin(rangeX, aabb.rangeX) && IsWithin(rangeY, aabb.rangeY);
        }
        
        /// @brief Includes an AABB into this one.
        /// @note If an unset AABB is added to this AABB, the result will be this AABB.
        /// @note If this AABB is unset and another AABB is added to it, the result will be
        ///   the other AABB.
        constexpr AABB& Include(const AABB aabb) noexcept
        {
            rangeX.Include(aabb.rangeX);
            rangeY.Include(aabb.rangeY);
            return *this;
        }
        
        /// @brief Includes a point into this AABB.
        constexpr AABB& Include(const Length2D value) noexcept
        {
            rangeX.Include(GetX(value));
            rangeY.Include(GetY(value));
            return *this;
        }
        
        /// @brief Moves this AABB by the given value.
        constexpr AABB& Move(const Length2D value) noexcept
        {
            rangeX.Move(GetX(value));
            rangeY.Move(GetY(value));
            return *this;
        }

        /// @brief Displaces this AABB by the given value.
        constexpr AABB& Displace(const Length2D value) noexcept
        {
            rangeX.Expand(GetX(value));
            rangeY.Expand(GetY(value));
            return *this;
        }
        
        /// @brief Fattens this AABB by the given amount.
        constexpr AABB& Fatten(const NonNegative<Length> amount) noexcept
        {
            rangeX.ExpandEqually(amount);
            rangeY.ExpandEqually(amount);
            return *this;
        }

        /// @brief Holds the value range of "X".
        ValueRange<Length> rangeX;
        
        /// @brief Holds the value range of "Y".
        ValueRange<Length> rangeY;
    };
    
    /// @brief Gets whether the two AABB objects are equal.
    /// @return <code>true</code> if the two values are equal, <code>false</code> otherwise.
    /// @relatedalso AABB
    constexpr bool operator== (const AABB& lhs, const AABB& rhs)
    {
        return (lhs.rangeX == rhs.rangeX) && (lhs.rangeY == rhs.rangeY);
    }
    
    /// @brief Gets whether the two AABB objects are not equal.
    /// @return <code>true</code> if the two values are not equal, <code>false</code> otherwise.
    /// @relatedalso AABB
    constexpr bool operator!= (const AABB& lhs, const AABB& rhs)
    {
        return !(lhs == rhs);
    }
    
    /// @brief Tests for overlap between two axis aligned bounding boxes.
    /// @note This function's complexity is constant.
    /// @relatedalso AABB
    constexpr bool TestOverlap(const AABB& a, const AABB& b) noexcept
    {
        return IsIntersecting(a.rangeX, b.rangeX) && IsIntersecting(a.rangeY, b.rangeY);
    }
    
    /// @brief Gets the center of the AABB.
    /// @relatedalso AABB
    constexpr Length2D GetCenter(const AABB& aabb) noexcept
    {
        return Length2D{GetCenter(aabb.rangeX), GetCenter(aabb.rangeY)};
    }
    
    /// @brief Gets dimensions of the given AABB.
    /// @relatedalso AABB
    constexpr Length2D GetDimensions(const AABB& aabb) noexcept
    {
        return Length2D{GetSize(aabb.rangeX), GetSize(aabb.rangeY)};
    }
    
    /// @brief Gets the extents of the AABB (half-widths).
    /// @relatedalso AABB
    constexpr Length2D GetExtents(const AABB& aabb) noexcept
    {
        return GetDimensions(aabb) / Real{2};
    }

    /// @brief Gets the perimeter length of the AABB.
    /// @warning Behavior is undefined for an invalid AABB.
    /// @return Twice the sum of the width and height.
    /// @relatedalso AABB
    constexpr Length GetPerimeter(const AABB& aabb) noexcept
    {
        return (GetSize(aabb.rangeX) + GetSize(aabb.rangeY)) * Real{2};
    }

    /// @brief Gets an invalid AABB value.
    /// @relatedalso AABB
    template <>
    constexpr AABB GetInvalid() noexcept
    {
        return {ValueRange<Length>{GetInvalid<Length>()}, ValueRange<Length>{GetInvalid<Length>()}};
    }
    
    /// @brief Gets the AABB that minimally encloses the given AABBs.
    /// @relatedalso AABB
    constexpr AABB GetEnclosingAABB(AABB a, const AABB& b)
    {
        return a.Include(b);
    }

    /// @brief Gets the AABB that the result of displacing the given AABB by the given
    ///   displacement amount.
    /// @relatedalso AABB
    constexpr AABB GetDisplacedAABB(AABB aabb, const Length2D displacement)
    {
        aabb.Displace(displacement);
        return aabb;
    }
    
    /// @brief Gets the fattened AABB result.
    /// @relatedalso AABB
    constexpr AABB GetFattenedAABB(AABB aabb, const Length amount)
    {
        aabb.Fatten(amount);
        return aabb;
    }

    /// @brief Checks whether the first AABB fully contains the second AABB.
    /// @details Whether the first AABB contains the entirety of the second AABB where
    ///   containment is defined as being equal-to or within an AABB.
    /// @note The "unset" AABB is contained by all valid AABBs including the "unset"
    ///   AABB itself.
    /// @param a AABB to test whether it constains the second AABB.
    /// @param b AABB to test whether it's contained by the first AABB.
    /// @relatedalso AABB
    constexpr bool Contains(const AABB& a, const AABB& b) noexcept
    {
        return IsWithin(a.rangeX, b.rangeX) && IsWithin(a.rangeY, b.rangeY);
    }

    /// @brief Includes the given location into the given AABB.
    constexpr AABB& Include(AABB& var, const Length2D& val) noexcept
    {
        return var.Include(val);
    }

    /// @brief Includes another AABB into this one.
    /// @note If an unset AABB is added to this AABB, the result will be this AABB.
    /// @note If this AABB is unset and another AABB is added to it, the result will be
    ///   the other AABB.
    constexpr AABB& Include(AABB& var, const AABB& val) noexcept
    {
        return var.Include(val);
    }
    
    /// @brief Fattens an AABB by the given amount.
    constexpr AABB& Fatten(AABB& var, const NonNegative<Length> amount) noexcept
    {
        var.rangeX.ExpandEqually(amount);
        var.rangeY.ExpandEqually(amount);
        return var;
    }

    /// @brief Gets the lower bound.
    /// @relatedalso AABB
    constexpr Length2D GetLowerBound(const AABB& aabb) noexcept
    {
        return Length2D{aabb.rangeX.GetMin(), aabb.rangeY.GetMin()};
    }
    
    /// @brief Gets the upper bound.
    /// @relatedalso AABB
    constexpr Length2D GetUpperBound(const AABB& aabb) noexcept
    {
        return Length2D{aabb.rangeX.GetMax(), aabb.rangeY.GetMax()};
    }
    
    /// @brief Computes the AABB.
    /// @details Computes the Axis Aligned Bounding Box (AABB) for the given child shape
    ///   at a given a transform.
    /// @warning Behavior is undefined if the given transformation is invalid.
    /// @param proxy Distance proxy for the child shape.
    /// @param xf World transform of the shape.
    /// @return AABB for the proxy shape or the default AABB if the proxy has a zero vertex count.
    /// @relatedalso DistanceProxy
    AABB ComputeAABB(const DistanceProxy& proxy, const Transformation& xf) noexcept;
    
    /// @brief Computes the AABB for the given shape with the given transformation.
    /// @relatedalso Shape
    AABB ComputeAABB(const Shape& shape, const Transformation& xf);

    /// @brief Computes the AABB for the given body.
    /// @relatedalso Body
    AABB ComputeAABB(const Body& body);

    /// @brief Gets the fixture's AABB.
    /// @note This AABB may be enlarged and/or stale. If you need a more accurate AABB,
    ///   compute it using the shape and the body transform.
    /// @warning Behavior is undefined is child index is not a valid proxy index.
    /// @sa Fixture::GetProxy.
    /// @relatedalso Fixture
    AABB GetAABB(const Fixture& fixture, ChildCounter childIndex) noexcept;

    /// @brief Output stream operator.
    inline ::std::ostream& operator<< (::std::ostream& os, const AABB& value)
    {
        os << "{";
        os << value.rangeX;
        os << ',';
        os << value.rangeY;
        os << "}";
        return os;
    }

} // namespace playrho

#endif // PLAYRHO_COLLISION_AABB_HPP
