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

#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

AABB box2d::ComputeAABB(const EdgeShape& shape, const Transformation& xf, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	const auto v1 = Transform(shape.GetVertex1(), xf);
	const auto v2 = Transform(shape.GetVertex2(), xf);
	
	const auto lower = Vec2{Min(v1.x, v2.x), Min(v1.y, v2.y)};
	const auto upper = Vec2{Max(v1.x, v2.x), Max(v1.y, v2.y)};
	
	const auto vertexRadius = GetVertexRadius(shape);
	const auto r = Vec2{vertexRadius, vertexRadius};
	return AABB{lower - r, upper + r};
}

AABB box2d::ComputeAABB(const PolygonShape& shape, const Transformation& xf, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	assert(shape.GetVertexCount() > 0);
	
	auto lower = Transform(shape.GetVertex(0), xf);
	auto upper = lower;
	
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		const auto v = Transform(shape.GetVertex(i), xf);
		lower = Vec2{Min(lower.x, v.x), Min(lower.y, v.y)};
		upper = Vec2{Max(upper.x, v.x), Max(upper.y, v.y)};
	}
	
	const auto vertexRadius = GetVertexRadius(shape);
	const auto r = Vec2{vertexRadius, vertexRadius};
	return AABB{lower - r, upper + r};
}

AABB box2d::ComputeAABB(const ChainShape& shape, const Transformation& xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());
	
	const auto v1 = Transform(shape.GetVertex(childIndex), xf);
	const auto v2 = Transform(shape.GetVertex(GetNextIndex(shape, childIndex)), xf);
	
	const auto lower = Vec2{Min(v1.x, v2.x), Min(v1.y, v2.y)};
	const auto upper = Vec2{Max(v1.x, v2.x), Max(v1.y, v2.y)};

	const auto vertexRadius = GetVertexRadius(shape);
	const auto r = Vec2{vertexRadius, vertexRadius};
	return AABB{lower - r, upper + r};
}

AABB box2d::ComputeAABB(const CircleShape& shape, const Transformation& transform, child_count_t childIndex)
{
	NOT_USED(childIndex);
	
	const auto p = transform.p + Rotate(shape.GetLocation(), transform.q);
	return GetFattenedAABB(AABB{p, p}, shape.GetRadius());
}

AABB box2d::ComputeAABB(const Shape& shape, const Transformation& xf, child_count_t childIndex)
{
	assert(shape.GetType() < Shape::e_typeCount);
	switch (shape.GetType())
	{
		case Shape::e_edge: return ComputeAABB(static_cast<const EdgeShape&>(shape), xf, childIndex);
		case Shape::e_chain: return ComputeAABB(static_cast<const ChainShape&>(shape), xf, childIndex);
		case Shape::e_circle: return ComputeAABB(static_cast<const CircleShape&>(shape), xf, childIndex);
		case Shape::e_polygon: return ComputeAABB(static_cast<const PolygonShape&>(shape), xf, childIndex);
		case Shape::e_typeCount: return GetInvalid<AABB>();
	}
}

AABB box2d::ComputeAABB(const Shape& shape, const Transformation& xf)
{
	const auto childCount = GetChildCount(shape);
	auto sum = AABB{};
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		sum += ComputeAABB(shape, xf, i);
	}
	return sum;
}

AABB box2d::ComputeAABB(const Fixture& fixture, const Transformation& xf)
{
	return ComputeAABB(*fixture.GetShape(), Mul(xf, fixture.GetTransformation()));
}

AABB box2d::ComputeAABB(const Body& body)
{
	auto sum = AABB{};
	const auto xf = body.GetTransformation();
	for (auto&& f: body.GetFixtures())
	{
		sum += ComputeAABB(*f, xf);
	}
	return sum;
}
