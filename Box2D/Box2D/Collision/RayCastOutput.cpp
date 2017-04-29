/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/RayCastOutput.hpp>
#include <Box2D/Collision/RayCastInput.hpp>
#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <utility>

using namespace box2d;

namespace
{
	inline RayCastOutput RayCast(const Length radius, const Length2D v0,
								 const RayCastInput& input, const Transformation& transform) noexcept
	{
		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		
		const auto position = transform.p + Rotate(v0, transform.q);
		const auto s = input.p1 - position;
		const auto sUnitless = StripUnits(s);
		const auto b = GetLengthSquared(sUnitless) - Square(radius / Meter);
		
		// Solve quadratic equation.
		const auto r = input.p2 - input.p1;
		const auto rUnitless = StripUnits(r);
		const auto c =  Dot(sUnitless, rUnitless);
		const auto rr = GetLengthSquared(rUnitless);
		const auto sigma = Square(c) - rr * b;
		
		// Check for negative discriminant and short segment.
		if ((sigma < RealNum{0}) || almost_zero(rr))
		{
			return RayCastOutput{};
		}
		
		// Find the point of intersection of the line with the circle.
		const auto a = -(c + Sqrt(sigma));
		
		// Is the intersection point on the segment?
		if ((a >= RealNum{0}) && (a <= (input.maxFraction * rr)))
		{
			const auto fraction = a / rr;
			return RayCastOutput{
				GetUnitVector(sUnitless + fraction * rUnitless, UnitVec2::GetZero()),
				fraction
			};
		}
		
		return RayCastOutput{};
	}

	inline RayCastOutput RayCast(const Length2D v1, const Length2D v2,
								 const UnitVec2 normal,
								 const RayCastInput& input, const Transformation& transform) noexcept
	{
		// p = p1 + t * d
		// v = v1 + s * e
		// p1 + t * d = v1 + s * e
		// s * e - t * d = p1 - v1
		
		// Put the ray into the edge's frame of reference.
		const auto d1 = input.p1 - transform.p;
		const auto p1 = InverseRotate(StripUnits(d1), transform.q);
		const auto d2 = input.p2 - transform.p;
		const auto p2 = InverseRotate(StripUnits(d2), transform.q);
		const auto d = p2 - p1;
		
		const auto e = v2 - v1;
		const auto eUnitless = StripUnits(e);
		
		// q = p1 + t * d
		// dot(normal, q - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		const auto v1p1 = v1 - p1 * Meter;
		const auto numerator = Dot(normal, StripUnits(v1p1));
		const auto denominator = Dot(normal, d);
		
		if (denominator == 0)
		{
			return RayCastOutput{};
		}
		
		const auto t = numerator / denominator;
		if ((t < 0) || (t > input.maxFraction))
		{
			return RayCastOutput{};
		}
		
		const auto q = p1 + t * d;
		
		// q = v1 + s * e
		// s = dot(q - v1, e) / dot(e, e)
		const auto ee = GetLengthSquared(eUnitless);
		if (ee == 0)
		{
			return RayCastOutput{};
		}
		
		const auto qv1 = q * Meter - v1;
		const auto s = Dot(StripUnits(qv1), eUnitless) / ee;
		if ((s < 0) || (s > 1))
		{
			return RayCastOutput{};
		}
		
		const auto normalFound = (numerator > 0)? -normal: normal;
		return RayCastOutput{Rotate(normalFound, transform.q), t};
	}
	
} // anonymous namespace

RayCastOutput box2d::RayCast(const AABB& aabb, const RayCastInput& input)
{
	// From Real-time Collision Detection, p179.

	auto tmin = -MaxFloat;
	auto tmax = MaxFloat;
	
	const auto p1 = input.p1;
	const auto pDelta = input.p2 - input.p1;
	
	UnitVec2 normal;
	
	for (auto i = decltype(pDelta.max_size()){0}; i < pDelta.max_size(); ++i)
	{
		const auto p1i = p1[i];
		const auto pdi = pDelta[i];
		const auto lbi = aabb.GetLowerBound()[i];
		const auto ubi = aabb.GetUpperBound()[i];

		if (almost_zero(pdi / Meter))
		{
			// Parallel.
			if ((p1i < lbi) || (ubi < p1i))
			{
				return RayCastOutput{};
			}
		}
		else
		{
			auto t1 = RealNum{(lbi - p1i) / pdi};
			auto t2 = RealNum{(ubi - p1i) / pdi};
			
			// Sign of the normal vector.
			auto s = -1;
			
			if (t1 > t2)
			{
				Swap(t1, t2);
				s = 1;
			}
			
			// Push the min up
			if (tmin < t1)
			{
				normal = (i == 0)?
					((s < 0)? UnitVec2::GetLeft(): UnitVec2::GetRight()):
					((s < 0)? UnitVec2::GetBottom(): UnitVec2::GetTop());
				tmin = t1;
			}
			
			// Pull the max down
			tmax = Min(tmax, t2);
			
			if (tmin > tmax)
			{
				return RayCastOutput{};
			}
		}
	};
	
	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ((tmin < 0) || (tmin > input.maxFraction))
	{
		return RayCastOutput{};
	}
	
	// Intersection.
	return RayCastOutput{normal, tmin};
}

RayCastOutput box2d::RayCast(const DistanceProxy& proxy, const RayCastInput& input,
							 const Transformation& transform) noexcept
{
	const auto vertexCount = proxy.GetVertexCount();
	assert(vertexCount > 0);
	switch (vertexCount)
	{
		case 0:
			return RayCastOutput{};
		case 1:
			return ::RayCast(proxy.GetVertexRadius(), proxy.GetVertex(0), input, transform);
		case 2:
			return ::RayCast(proxy.GetVertex(0), proxy.GetVertex(1), proxy.GetNormal(0),
							 input, transform);
		default:
		{
			// Put the ray into the polygon's frame of reference.
			const auto p1 = InverseRotate(input.p1 - transform.p, transform.q);
			const auto p2 = InverseRotate(input.p2 - transform.p, transform.q);
			const auto d = StripUnits(p2 - p1);
			
			auto lower = RealNum{0};
			auto upper = input.maxFraction;
			auto normalFound = GetInvalid<UnitVec2>();
			
			for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
			{
				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0
				const auto normal = proxy.GetNormal(i);
				const auto vertex = proxy.GetVertex(i);
				const auto numerator = Dot(normal, StripUnits(vertex - p1));
				const auto denominator = Dot(normal, d);
				
				if (denominator == RealNum{0})
				{
					if (numerator < RealNum{0})
					{
						return RayCastOutput{};
					}
				}
				else
				{
					const auto t = numerator / denominator;
					
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower > numerator.
					if (denominator < RealNum{0} && numerator < lower * denominator)
					{
						// Increase lower. The segment enters this half-space.
						lower = t;
						normalFound = normal;
					}
					else if (denominator > RealNum{0} && numerator < upper * denominator)
					{
						// Decrease upper. The segment exits this half-space.
						upper = t;
					}
				}
				
				if (upper < lower)
				{
					if (!almost_equal(upper, lower))
					{
						return RayCastOutput{};
					}
					std::swap(upper, lower);
				}
			}
			assert(RealNum{0} <= lower);
			assert(lower <= input.maxFraction);
			
			if (IsValid(normalFound))
			{
				return RayCastOutput{Rotate(normalFound, transform.q), lower};
			}
			return RayCastOutput{};
		}
	}
}

RayCastOutput box2d::RayCast(const Fixture& f, const RayCastInput& input, child_count_t childIndex)
{
	const auto child = f.GetShape()->GetChild(childIndex);
	return RayCast(child, input, GetTransformation(f));
}
