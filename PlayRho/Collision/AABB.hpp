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

#ifndef AABB_hpp
#define AABB_hpp

/// @file
/// Declaration of the AABB class and free functions that return instances of it.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho
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
        
        /// @brief Non-throwing default constructor.
        /// @details Constructs an empty AABB.
        /// @note If an empty AABB is added to another AABB, the result will be the other AABB.
        constexpr AABB() noexcept
        {
            // Intentionally empty.
        }
        
        /// @brief Non-throwing initializing constructor for a single point.
        constexpr AABB(const Length2D p) noexcept:
            m_lowerBound{p}, m_upperBound{p}
        {
            // Intentionally empty.
        }
        
        /// @brief Non-throwing initializing constructor for two points.
        constexpr AABB(const Length2D a, const Length2D b) noexcept:
            m_lowerBound{Length2D{std::min(GetX(a), GetX(b)), std::min(GetY(a), GetY(b))}},
            m_upperBound{Length2D{std::max(GetX(a), GetX(b)), std::max(GetY(a), GetY(b))}}
        {
            // Intentionally empty.
        }
        
        /// @brief Explicitly defined non-throwing copy constructor.
        constexpr AABB(const AABB& copy) noexcept:
            m_lowerBound{copy.m_lowerBound}, m_upperBound{copy.m_upperBound}
        {
            // Intentionally empty.
        }

        /// @brief Explicitly defined non-throwing copy assignment operator.
        constexpr AABB& operator= (const AABB copy) noexcept
        {
            m_lowerBound = copy.m_lowerBound;
            m_upperBound = copy.m_upperBound;
            return *this;
        }

        /// @brief Gets the lower bound.
        constexpr Length2D GetLowerBound() const noexcept { return m_lowerBound; }
        
        /// @brief Gets the upper bound.
        constexpr Length2D GetUpperBound() const noexcept { return m_upperBound; }
        
        /// @brief Checks whether this AABB fully contains the given AABB.
        constexpr bool Contains(const AABB aabb) const noexcept
        {
            const auto lower = GetLowerBound();
            const auto upper = GetUpperBound();
            const auto other_lower = aabb.GetLowerBound();
            const auto other_upper = aabb.GetUpperBound();
            return
            (GetX(lower) <= GetX(other_lower)) && (GetY(lower) <= GetY(other_lower)) &&
            (GetX(other_upper) <= GetX(upper)) && (GetY(other_upper) <= GetY(upper));
        }
        
        /// @brief Includes an AABB into this one.
        constexpr AABB& Include(const AABB aabb) noexcept
        {
            m_lowerBound = Length2D{
                std::min(GetX(m_lowerBound), GetX(aabb.m_lowerBound)),
                std::min(GetY(m_lowerBound), GetY(aabb.m_lowerBound))
            };
            m_upperBound = Length2D{
                std::max(GetX(m_upperBound), GetX(aabb.m_upperBound)),
                std::max(GetY(m_upperBound), GetY(aabb.m_upperBound))
            };
            return *this;
        }
        
        /// @brief Includes a point into this AABB.
        constexpr AABB& Include(const Length2D value) noexcept
        {
            m_lowerBound = Length2D{
                std::min(GetX(m_lowerBound), GetX(value)),
                std::min(GetY(m_lowerBound), GetY(value))
            };
            m_upperBound = Length2D{
                std::max(GetX(m_upperBound), GetX(value)),
                std::max(GetY(m_upperBound), GetY(value))
            };
            return *this;
        }

        /// @brief Moves this AABB by the given value.
        constexpr AABB& Move(const Length2D value) noexcept
        {
            m_lowerBound += value;
            m_upperBound += value;
            return *this;
        }
        
        /// @brief Displaces this AABB by the given value.
        constexpr AABB& Displace(const Length2D value) noexcept
        {
            if (GetX(value) < decltype(GetX(value)){0})
            {
                GetX(m_lowerBound) += GetX(value);
            }
            else
            {
                GetX(m_upperBound) += GetX(value);
            }
            
            if (GetY(value) < decltype(GetY(value)){0})
            {
                GetY(m_lowerBound) += GetY(value);
            }
            else
            {
                GetY(m_upperBound) += GetY(value);
            }
            return *this;
        }
        
        /// @brief Fattens an AABB by the given amount.
        constexpr AABB& Fatten(const NonNegative<Length> amount) noexcept
        {
            const auto value = Length{amount};
            GetX(m_lowerBound) -= value;
            GetY(m_lowerBound) -= value;
            GetX(m_upperBound) += value;
            GetY(m_upperBound) += value;
            return *this;
        }
        
    private:
        
        /// @brief Lower vertex.
        Length2D m_lowerBound = Length2D{
            +std::numeric_limits<Real>::infinity() * Meter,
            +std::numeric_limits<Real>::infinity() * Meter
        };

        /// @brief Upper vertex.
        Length2D m_upperBound = Length2D{
            -std::numeric_limits<Real>::infinity() * Meter,
            -std::numeric_limits<Real>::infinity() * Meter
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
        return (aabb.GetLowerBound() + aabb.GetUpperBound()) / Real{2};
    }
    
    constexpr Length2D GetDimensions(const AABB aabb) noexcept
    {
        return aabb.GetUpperBound() - aabb.GetLowerBound();
    }

    /// Gets the extents of the AABB (half-widths).
    constexpr Length2D GetExtents(const AABB aabb) noexcept
    {
        return GetDimensions(aabb) / Real{2};
    }
    
    /// @brief Gets the perimeter length of the AABB.
    /// @warning Behavior is undefined for an invalid AABB.
    /// @return Twice the sum of the width and height.
    constexpr Length GetPerimeter(const AABB aabb) noexcept
    {
        const auto dimensions = GetDimensions(aabb);
        return (GetX(dimensions) + GetY(dimensions)) * Real{2};
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

        return (GetX(d1) <= Length{0}) && (GetY(d1) <= Length{0})
            && (GetX(d2) <= Length{0}) && (GetY(d2) <= Length{0});
    }

    /// @brief Computes the AABB.
    /// @details Computes the Axis Aligned Bounding Box (AABB) for the given child shape
    ///   at a given a transform.
    /// @warning Behavior is undefined if the given transformation is invalid.
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
    AABB GetAABB(const Fixture& fixture, ChildCounter childIndex) noexcept;

} // namespace playrho

#endif /* AABB_hpp */
