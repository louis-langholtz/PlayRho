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

#include <PlayRho/Common/Intervals.hpp>
#include <PlayRho/Common/Vector2.hpp>
#include <PlayRho/Common/Templates.hpp>

namespace playrho {

    class Shape;
    class Fixture;
    class Body;
    class Contact;
    class DistanceProxy;
    struct Transformation;
    
    /// @brief Axis Aligned Bounding Box.
    ///
    /// @details This is a concrete value class for an axis aligned bounding box (AABB)
    ///   which is a type of bounding volume.
    ///
    /// @note This class satisfies at least the following concepts: all the basic concepts,
    ///   EqualityComparable, and Swappable.
    /// @note This class is composed of &mdash; as in contains and owns &mdash two
    ///   <code>LengthInterval</code> variables.
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
        constexpr AABB(const LengthInterval& x, const LengthInterval& y) noexcept:
            rangeX{x}, rangeY{y}
        {
            // Intentionally empty.
        }

        /// @brief Initializing move constructor.
        constexpr AABB(LengthInterval&& x, LengthInterval&& y) noexcept:
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
            rangeX{Get<0>(p)}, rangeY{Get<1>(p)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor for two points.
        /// @param a Point location "A" to initialize this AABB with.
        /// @param b Point location "B" to initialize this AABB with.
        constexpr AABB(const Length2D a, const Length2D b) noexcept:
            rangeX{Get<0>(a), Get<0>(b)}, rangeY{Get<1>(a), Get<1>(b)}
        {
            // Intentionally empty.
        }
        
        /// @brief Holds the value range of "X".
        LengthInterval rangeX;
        
        /// @brief Holds the value range of "Y".
        LengthInterval rangeY;
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
    
    /// @brief Less-than operator.
    /// @relatedalso AABB
    constexpr bool operator< (const AABB& lhs, const AABB& rhs) noexcept
    {
        return (lhs.rangeX < rhs.rangeX) ||
            ((lhs.rangeX == rhs.rangeX) && (lhs.rangeY < rhs.rangeY));
    }
    
    /// @brief Less-than or equal-to operator.
    /// @relatedalso AABB
    constexpr bool operator<= (const AABB& lhs, const AABB& rhs) noexcept
    {
        return (lhs.rangeX < rhs.rangeX) ||
            ((lhs.rangeX == rhs.rangeX) && (lhs.rangeY <= rhs.rangeY));
    }
    
    /// @brief Greater-than operator.
    /// @relatedalso AABB
    constexpr bool operator> (const AABB& lhs, const AABB& rhs) noexcept
    {
        return (lhs.rangeX > rhs.rangeX) ||
            ((lhs.rangeX == rhs.rangeX) && (lhs.rangeY > rhs.rangeY));
    }
    
    /// @brief Greater-than or equal-to operator.
    /// @relatedalso AABB
    constexpr bool operator>= (const AABB& lhs, const AABB& rhs) noexcept
    {
        return (lhs.rangeX > rhs.rangeX) ||
            ((lhs.rangeX == rhs.rangeX) && (lhs.rangeY >= rhs.rangeY));
    }

    /// @brief Tests for overlap between two axis aligned bounding boxes.
    /// @note This function's complexity is constant.
    /// @relatedalso AABB
    constexpr bool TestOverlap(const AABB& a, const AABB& b) noexcept
    {
        return IsIntersecting(a.rangeX, b.rangeX) && IsIntersecting(a.rangeY, b.rangeY);
    }
    
    /// @brief Gets the intersecting AABB of the two given AABBs'.
    constexpr AABB GetIntersectingAABB(const AABB& a, const AABB& b) noexcept
    {
        const auto rangeX = GetIntersection(a.rangeX, b.rangeX);
        const auto rangeY = GetIntersection(a.rangeY, b.rangeY);
        return AABB{rangeX, rangeY};
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
        return {LengthInterval{GetInvalid<Length>()}, LengthInterval{GetInvalid<Length>()}};
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
        return IsEntirelyEnclosing(a.rangeX, b.rangeX) && IsEntirelyEnclosing(a.rangeY, b.rangeY);
    }

    /// @brief Includes the given location into the given AABB.
    /// @relatedalso AABB
    constexpr AABB& Include(AABB& var, const Length2D& value) noexcept
    {
        var.rangeX.Include(Get<0>(value));
        var.rangeY.Include(Get<1>(value));
        return var;
    }

    /// @brief Includes the second AABB into the first one.
    /// @note If an unset AABB is added to the first AABB, the result will be the first AABB.
    /// @note If the first AABB is unset and another AABB is added to it, the result will be
    ///   the other AABB.
    /// @relatedalso AABB
    constexpr AABB& Include(AABB& var, const AABB& val) noexcept
    {
        var.rangeX.Include(val.rangeX);
        var.rangeY.Include(val.rangeY);
        return var;
    }
    
    /// @brief Moves the given AABB by the given value.
    constexpr AABB& Move(AABB& var, const Length2D value) noexcept
    {
        var.rangeX.Move(Get<0>(value));
        var.rangeY.Move(Get<1>(value));
        return var;
    }
    
    /// @brief Fattens an AABB by the given amount.
    /// @relatedalso AABB
    constexpr AABB& Fatten(AABB& var, const NonNegative<Length> amount) noexcept
    {
        var.rangeX.ExpandEqually(amount);
        var.rangeY.ExpandEqually(amount);
        return var;
    }
    
    /// @brief Gets the AABB that the result of displacing the given AABB by the given
    ///   displacement amount.
    /// @relatedalso AABB
    constexpr AABB GetDisplacedAABB(AABB aabb, const Length2D displacement)
    {
        aabb.rangeX.Expand(Get<0>(displacement));
        aabb.rangeY.Expand(Get<1>(displacement));
        return aabb;
    }

    /// @brief Gets the fattened AABB result.
    /// @relatedalso AABB
    constexpr AABB GetFattenedAABB(AABB aabb, const Length amount)
    {
        return Fatten(aabb, amount);
    }

    /// @brief Gets the result of moving the given AABB by the given value.
    /// @relatedalso AABB
    constexpr AABB GetMovedAABB(AABB aabb, const Length2D value) noexcept
    {
        return Move(aabb, value);
    }

    /// @brief Gets the AABB that minimally encloses the given AABBs.
    /// @relatedalso AABB
    constexpr AABB GetEnclosingAABB(AABB a, const AABB& b)
    {
        return Include(a, b);
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

    /// @brief Computes the AABB.
    /// @details Computes the Axis Aligned Bounding Box (AABB) for the given child shape
    ///   at the given transforms.
    /// @warning Behavior is undefined if a given transformation is invalid.
    /// @param proxy Distance proxy for the child shape.
    /// @param xfm0 World transform 0 of the shape.
    /// @param xfm1 World transform 1 of the shape.
    /// @return AABB for the proxy shape or the default AABB if the proxy has a zero vertex count.
    /// @relatedalso DistanceProxy
    AABB ComputeAABB(const DistanceProxy& proxy,
                     const Transformation& xfm0, const Transformation& xfm1) noexcept;

    /// @brief Computes the AABB for the given shape with the given transformation.
    /// @relatedalso Shape
    AABB ComputeAABB(const Shape& shape, const Transformation& xf) noexcept;

    /// @brief Computes the AABB for the given fixture.
    /// @details This is the AABB of the entire shape of the given fixture at the body's
    ///   location for the given fixture.
    /// @relatedalso Fixture
    AABB ComputeAABB(const Fixture& fixture) noexcept;

    /// @brief Computes the AABB for the given body.
    /// @relatedalso Body
    AABB ComputeAABB(const Body& body);

    /// @brief Computes the intersecting AABB for the given pair of fixtures and indexes.
    /// @details The intersecting AABB for the given pair of fixtures is the intersection
    ///   of the AABB for child A of the shape of fixture A with the AABB for child B of
    ///   the shape of fixture B.
    AABB ComputeIntersectingAABB(const Fixture& fA, ChildCounter iA,
                                 const Fixture& fB, ChildCounter iB) noexcept;

    /// @brief Computes the intersecting AABB for the given contact.
    AABB ComputeIntersectingAABB(const Contact& contact);

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
