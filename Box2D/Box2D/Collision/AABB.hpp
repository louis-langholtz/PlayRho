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

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	class Shape;
	class EdgeShape;
	class PolygonShape;
	class ChainShape;
	class CircleShape;
	class Fixture;
	class Body;
	
	/// An axis aligned bounding box.
	/// @note This data structure is 16-bytes large (on at least one 64-bit platform).
	class AABB
	{
	public:
		
		/// Default constructor.
		/// @detail Constructs an empty AABB. If an empty AABB is added to another AABB, the
		///   result will always be the other AABB.
		AABB() = default;
		
		constexpr AABB(const Vec2 a, const Vec2 b) noexcept:
			lowerBound{Vec2{Min(a.x, b.x), Min(a.y, b.y)}},
			upperBound{Vec2{Max(a.x, b.x), Max(a.y, b.y)}}
		{
			// Intentionally empty.
		}
		
		/// Combine an AABB into this one.
		constexpr AABB& operator += (const AABB aabb)
		{
			lowerBound = Vec2{Min(lowerBound.x, aabb.lowerBound.x), Min(lowerBound.y, aabb.lowerBound.y)};
			upperBound = Vec2{Max(upperBound.x, aabb.upperBound.x), Max(upperBound.y, aabb.upperBound.y)};
			return *this;
		}
		
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
		
		constexpr Vec2 GetLowerBound() const noexcept { return lowerBound; }

		constexpr Vec2 GetUpperBound() const noexcept { return upperBound; }
		
		constexpr AABB& Move(const Vec2 value) noexcept
		{
			lowerBound += value;
			upperBound += value;
			return *this;
		}
		
		constexpr AABB& Displace(const Vec2 value) noexcept
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
		/// @warning Behavior is undefined if given a negative value.
		constexpr AABB& Fatten(const RealNum value) noexcept
		{
			assert(value >= 0);
			lowerBound.x -= value;
			lowerBound.y -= value;
			upperBound.x += value;
			upperBound.y += value;
			return *this;
		}
		
	private:
		static constexpr auto infinity = std::numeric_limits<RealNum>::infinity();
		Vec2 lowerBound = Vec2{infinity, infinity}; ///< the lower vertex
		Vec2 upperBound = Vec2{-infinity, -infinity}; ///< the upper vertex
	};
	
	template <>
	constexpr AABB GetInvalid() noexcept
	{
		return AABB{GetInvalid<Vec2>(), GetInvalid<Vec2>()};
	}
	
	/// Gets the center of the AABB.
	constexpr Vec2 GetCenter(const AABB aabb) noexcept
	{
		return (aabb.GetLowerBound() + aabb.GetUpperBound()) / 2;
	}
	
	constexpr Vec2 GetDimensions(const AABB aabb) noexcept
	{
		return aabb.GetUpperBound() - aabb.GetLowerBound();
	}

	/// Gets the extents of the AABB (half-widths).
	constexpr Vec2 GetExtents(const AABB aabb) noexcept
	{
		return GetDimensions(aabb) / 2;
	}
	
	/// Gets the perimeter length of the AABB.
	/// @return Twice the sum of the width and height.
	constexpr RealNum GetPerimeter(const AABB aabb) noexcept
	{
		const auto upper = aabb.GetUpperBound();
		const auto lower = aabb.GetLowerBound();
		const auto wx = upper.x - lower.x;
		const auto wy = upper.y - lower.y;
		return (wx + wy) * 2;
	}

	constexpr AABB GetEnclosingAABB(AABB a, AABB b)
	{
		a += b;
		return a;
	}
	
	constexpr AABB GetDisplacedAABB(AABB aabb, const Vec2 displacement)
	{
		aabb.Displace(displacement);
		return aabb;
	}
		
	constexpr AABB GetFattenedAABB(AABB aabb, const RealNum amount)
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

		return (d1.x <= 0) && (d1.y <= 0) && (d2.x <= 0) && (d2.y <= 0);
	}

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	AABB ComputeAABB(const EdgeShape& shape, const Transformation& xf, child_count_t childIndex);
	
	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	AABB ComputeAABB(const PolygonShape& shape, const Transformation& xf, child_count_t childIndex);
	
	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	AABB ComputeAABB(const ChainShape& shape, const Transformation& xf, child_count_t childIndex);
	
	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	AABB ComputeAABB(const CircleShape& shape, const Transformation& xf, child_count_t childIndex);
	
	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	/// @return the axis aligned box.
	AABB ComputeAABB(const Shape& shape, const Transformation& xf, child_count_t childIndex);

	AABB ComputeAABB(const Shape& shape, const Transformation& xf);

	AABB ComputeAABB(const Fixture& fixture, const Transformation& xf);

	AABB ComputeAABB(const Body& body);

} // namespace box2d

#endif /* AABB_hpp */
