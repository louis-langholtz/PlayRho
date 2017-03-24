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
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

using namespace box2d;
	
// From Real-time Collision Detection, p179.
RayCastOutput box2d::RayCast(const AABB& aabb, const RayCastInput& input)
{
	auto tmin = -MaxFloat;
	auto tmax = MaxFloat;
	
	const auto p = input.p1;
	const auto d = input.p2 - input.p1;
	
	UnitVec2 normal;
	
	for (auto i = decltype(d.max_size()){0}; i < d.max_size(); ++i)
	{
		if (almost_zero(d[i]))
		{
			// Parallel.
			if ((p[i] < aabb.GetLowerBound()[i]) || (aabb.GetUpperBound()[i] < p[i]))
			{
				return RayCastOutput{};
			}
		}
		else
		{
			const auto inv_d = RealNum{1} / d[i];
			auto t1 = (aabb.GetLowerBound()[i] - p[i]) * inv_d;
			auto t2 = (aabb.GetUpperBound()[i] - p[i]) * inv_d;
			
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
				normal = (i == 0)? ((s < 0)? UnitVec2::GetLeft(): UnitVec2::GetRight()): ((s < 0)? UnitVec2::GetBottom(): UnitVec2::GetTop());
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

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
RayCastOutput box2d::RayCast(const CircleShape& shape, const RayCastInput& input,
							 const Transformation& transform, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	const auto position = transform.p + Rotate(shape.GetLocation(), transform.q);
	const auto s = input.p1 - position;
	const auto b = GetLengthSquared(s) - Square(shape.GetRadius());
	
	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto c =  Dot(s, r);
	const auto rr = GetLengthSquared(r);
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
		return RayCastOutput{GetUnitVector(s + fraction * r, UnitVec2::GetZero()), fraction};
	}
	
	return RayCastOutput{};
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
RayCastOutput box2d::RayCast(const EdgeShape& shape, const RayCastInput& input,
							 const Transformation& xf, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	// Put the ray into the edge's frame of reference.
	const auto p1 = InverseRotate(input.p1 - xf.p, xf.q);
	const auto p2 = InverseRotate(input.p2 - xf.p, xf.q);
	const auto d = p2 - p1;
	
	const auto v1 = shape.GetVertex1();
	const auto v2 = shape.GetVertex2();
	const auto e = v2 - v1;
	const auto normal = GetUnitVector(GetFwdPerpendicular(e), UnitVec2::GetZero());
	
	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	const auto numerator = Dot(normal, v1 - p1);
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
	const auto ee = GetLengthSquared(e);
	if (ee == 0)
	{
		return RayCastOutput{};
	}
	
	const auto s = Dot(q - v1, e) / ee;
	if ((s < 0) || (s > 1))
	{
		return RayCastOutput{};
	}
	
	const auto n = (numerator > 0)? -Rotate(normal, xf.q): Rotate(normal, xf.q);
	return RayCastOutput{n, t};
}

RayCastOutput box2d::RayCast(const PolygonShape& shape, const RayCastInput& input,
							 const Transformation& xf, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	// Put the ray into the polygon's frame of reference.
	const auto p1 = InverseRotate(input.p1 - xf.p, xf.q);
	const auto p2 = InverseRotate(input.p2 - xf.p, xf.q);
	const auto d = p2 - p1;
	
	auto lower = RealNum{0};
	auto upper = input.maxFraction;
	constexpr auto InvalidIndex = static_cast<PolygonShape::vertex_count_t>(-1);
	auto index = InvalidIndex;
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		const auto numerator = Dot(shape.GetNormal(i), shape.GetVertex(i) - p1);
		const auto denominator = Dot(shape.GetNormal(i), d);
		
		if (denominator == RealNum{0})
		{	
			if (numerator < RealNum{0})
			{
				return RayCastOutput{};
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < RealNum{0} && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > RealNum{0} && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}
		
		if (upper < lower)
		{
			return RayCastOutput{};
		}
	}
	
	assert((RealNum{0} <= lower) && (lower <= input.maxFraction));
	
	if (index != InvalidIndex)
	{
		return RayCastOutput{Rotate(shape.GetNormal(index), xf.q), lower};
	}
	
	return RayCastOutput{};
}

RayCastOutput box2d::RayCast(const ChainShape& shape, const RayCastInput& input,
							 const Transformation& xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());
	
	const auto i1 = childIndex;
	const auto i2 = GetNextIndex(shape, childIndex);
	auto conf = EdgeShape::Conf{};
	conf.vertexRadius = shape.GetVertexRadius();
	const auto edgeShape = EdgeShape{shape.GetVertex(i1), shape.GetVertex(i2), conf};
	return RayCast(edgeShape, input, xf, 0);
}

RayCastOutput box2d::RayCast(const Shape& shape, const RayCastInput& input,
							 const Transformation& xf, child_count_t childIndex)
{
	assert(shape.GetType() < Shape::e_typeCount);
	switch (shape.GetType())
	{
		case Shape::e_edge: return RayCast(static_cast<const EdgeShape&>(shape), input, xf, childIndex);
		case Shape::e_chain: return RayCast(static_cast<const ChainShape&>(shape), input, xf, childIndex);
		case Shape::e_circle: return RayCast(static_cast<const CircleShape&>(shape), input, xf, childIndex);
		case Shape::e_polygon: return RayCast(static_cast<const PolygonShape&>(shape), input, xf, childIndex);
		case Shape::e_typeCount: return RayCastOutput{};
	}
}

RayCastOutput box2d::RayCast(const Fixture& f, const RayCastInput& input, child_count_t childIndex)
{
	return RayCast(*f.GetShape(), input, GetTransformation(f), childIndex);
}
