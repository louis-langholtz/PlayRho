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

#ifndef AABB_hpp
#define AABB_hpp

/// @file
/// Declaration of the AABB class and free functions that return instances of it.

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/BoundedValue.hpp>

namespace box2d
{
    class Shape;
    class Fixture;
    class Body;
    class DistanceProxy;
    
    /// @brief Axis Aligned Bounding Box.
    ///
    /// @details This is a value class for an axis aligned bounding box which is a type
    ///   of bounding volume.
    ///
    /// @note This data structure is 16-bytes large (on at least one 64-bit platform).
    ///
    /// @invariant The lower bound always has lower x and y values than the upper bound's
    ///   x and y values for any non-empty valid AABB.
    ///
    /// @sa https://en.wikipedia.org/wiki/Bounding_volume
    ///
    class AABB
    {
    public:
        
        /// Default constructor.
        /// @details Constructs an empty AABB. If an empty AABB is added to another AABB, the
        ///   result will always be the other AABB.
        AABB() = default;
        
        /// Initializing constructor for a single point.
        constexpr AABB(const Length2D p) noexcept:
            lowerBound{p}, upperBound{p}
        {
            // Intentionally empty.
        }
        
        /// Initializing constructor for two points.
        constexpr AABB(const Length2D a, const Length2D b) noexcept:
            lowerBound{Length2D{Min(a.x, b.x), Min(a.y, b.y)}},
            upperBound{Length2D{Max(a.x, b.x), Max(a.y, b.y)}}
        {
            // Intentionally empty.
        }
        
        constexpr Length2D GetLowerBound() const noexcept { return lowerBound; }
        
        constexpr Length2D GetUpperBound() const noexcept { return upperBound; }
        
        /// Does this AABB fully contain the given AABB.
        constexpr bool Contains(const AABB aabb) const noexcept
        {
            const auto lower = GetLowerBound();
            const auto upper = GetUpperBound();
            const auto other_lower = aabb.GetLowerBound();
            const auto other_upper = aabb.GetUpperBound();
            return
            (lower.x <= other_lower.x) && (lower.y <= other_lower.y) &&
            (other_upper.x <= upper.x) && (other_upper.y <= upper.y);
        }
        
        /// Combine an AABB into this one.
        constexpr AABB& Include(const AABB aabb) noexcept
        {
            lowerBound = Length2D{Min(lowerBound.x, aabb.lowerBound.x), Min(lowerBound.y, aabb.lowerBound.y)};
            upperBound = Length2D{Max(upperBound.x, aabb.upperBound.x), Max(upperBound.y, aabb.upperBound.y)};
            return *this;
        }
        
        constexpr AABB& Include(const Length2D value) noexcept
        {
            lowerBound = Length2D{Min(lowerBound.x, value.x), Min(lowerBound.y, value.y)};
            upperBound = Length2D{Max(upperBound.x, value.x), Max(upperBound.y, value.y)};
            return *this;
        }

        constexpr AABB& Move(const Length2D value) noexcept
        {
            lowerBound += value;
            upperBound += value;
            return *this;
        }
        
        constexpr AABB& Displace(const Length2D value) noexcept
        {
            if (value.x < decltype(value.x){0})
            {
                lowerBound.x += value.x;
            }
            else
            {
                upperBound.x += value.x;
            }
            
            if (value.y < decltype(value.y){0})
            {
                lowerBound.y += value.y;
            }
            else
            {
                upperBound.y += value.y;
            }
            return *this;
        }
        
        /// Fattens an AABB by the given amount.
        constexpr AABB& Fatten(const NonNegative<Length> amount) noexcept
        {
            const auto value = Length{amount};
            lowerBound.x -= value;
            lowerBound.y -= value;
            upperBound.x += value;
            upperBound.y += value;
            return *this;
        }
        
    private:
        
        /// @brief Lower vertex.
        Length2D lowerBound = Length2D{
            std::numeric_limits<RealNum>::infinity() * Meter,
            std::numeric_limits<RealNum>::infinity() * Meter
        };

        /// @brief Upper vertex.
        Length2D upperBound = Length2D{
            -std::numeric_limits<RealNum>::infinity() * Meter,
            -std::numeric_limits<RealNum>::infinity() * Meter
        };
    };
    
    template <>
    constexpr AABB GetInvalid() noexcept
    {
        return AABB{GetInvalid<Length2D>(), GetInvalid<Length2D>()};
    }
    
    /// Gets the center of the AABB.
    constexpr Length2D GetCenter(const AABB aabb) noexcept
    {
        return (aabb.GetLowerBound() + aabb.GetUpperBound()) / RealNum{2};
    }
    
    constexpr Length2D GetDimensions(const AABB aabb) noexcept
    {
        return aabb.GetUpperBound() - aabb.GetLowerBound();
    }

    /// Gets the extents of the AABB (half-widths).
    constexpr Length2D GetExtents(const AABB aabb) noexcept
    {
        return GetDimensions(aabb) / RealNum{2};
    }
    
    /// Gets the perimeter length of the AABB.
    /// @return Twice the sum of the width and height.
    constexpr Length GetPerimeter(const AABB aabb) noexcept
    {
        const auto upper = aabb.GetUpperBound();
        const auto lower = aabb.GetLowerBound();
        const auto wx = upper.x - lower.x;
        const auto wy = upper.y - lower.y;
        return (wx + wy) * RealNum{2};
    }

    constexpr AABB GetEnclosingAABB(AABB a, AABB b)
    {
        return a.Include(b);
    }
    
    constexpr AABB GetDisplacedAABB(AABB aabb, const Length2D displacement)
    {
        aabb.Displace(displacement);
        return aabb;
    }
        
    constexpr AABB GetFattenedAABB(AABB aabb, const Length amount)
    {
        aabb.Fatten(amount);
        return aabb;
    }

    constexpr bool operator== (const AABB lhs, const AABB rhs)
    {
        return (lhs.GetLowerBound() == rhs.GetLowerBound()) && (lhs.GetUpperBound() == rhs.GetUpperBound());
    }
    
    constexpr bool operator!= (const AABB lhs, const AABB rhs)
    {
        return !(lhs == rhs);
    }

    // Tests for overlap between two axis aligned bounding boxes.
    // @note This function's complexity is constant.
    constexpr bool TestOverlap(const AABB a, const AABB b) noexcept
    {
        const auto d1 = b.GetLowerBound() - a.GetUpperBound();
        const auto d2 = a.GetLowerBound() - b.GetUpperBound();

        return (d1.x <= Length{0}) && (d1.y <= Length{0}) && (d2.x <= Length{0}) && (d2.y <= Length{0});
    }

    /// @brief Computes the AABB.
    /// @details Computes the Axis Aligned Bounding Box (AABB) for the given child shape
    ///   at a given a transform.
    /// @param proxy Distance proxy for the child shape.
    /// @param xf World transform of the shape.
    /// @return AABB for the proxy shape or the default AABB if the proxy has a zero vertex count.
    AABB ComputeAABB(const DistanceProxy& proxy, const Transformation xf) noexcept;
    
    AABB ComputeAABB(const Shape& shape, const Transformation xf);

    AABB ComputeAABB(const Body& body);

    /// Gets the fixture's AABB.
    /// @note This AABB may be enlarged and/or stale. If you need a more accurate AABB,
    ///   compute it using the shape and the body transform.
    /// @warning Behavior is undefined is child index is not a valid proxy index.
    /// @sa Fixture::GetProxy.
    AABB GetAABB(const Fixture& fixture, child_count_t childIndex) noexcept;

} // namespace box2d

#endif /* AABB_hpp */
