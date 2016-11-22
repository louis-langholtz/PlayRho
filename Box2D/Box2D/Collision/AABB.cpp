/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

using namespace box2d;

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

AABB box2d::ComputeAABB(const EdgeShape& shape, const Transformation& xf, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);
	
	const auto v1 = Transform(shape.GetVertex1(), xf);
	const auto v2 = Transform(shape.GetVertex2(), xf);
	
	const auto lower = Min(v1, v2);
	const auto upper = Max(v1, v2);
	
	const auto r = Vec2{GetVertexRadius(shape), GetVertexRadius(shape)};
	return AABB{lower - r, upper + r};
}

AABB box2d::ComputeAABB(const PolygonShape& shape, const Transformation& xf, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);
	
	assert(shape.GetVertexCount() > 0);
	
	auto lower = Transform(shape.GetVertex(0), xf);
	auto upper = lower;
	
	const auto count = shape.GetVertexCount();
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		const auto v = Transform(shape.GetVertex(i), xf);
		lower = Min(lower, v);
		upper = Max(upper, v);
	}
	
	const auto r = Vec2{GetVertexRadius(shape), GetVertexRadius(shape)};
	return AABB{lower - r, upper + r};
}

AABB box2d::ComputeAABB(const ChainShape& shape, const Transformation& xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());
	
	const auto i1 = childIndex;
	const auto i2 = GetNextIndex(shape, childIndex);
	const auto v1 = Transform(shape.GetVertex(i1), xf);
	const auto v2 = Transform(shape.GetVertex(i2), xf);
	return AABB{v1, v2};
}

AABB box2d::ComputeAABB(const CircleShape& shape, const Transformation& transform, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);
	
	const auto p = transform.p + Rotate(shape.GetLocation(), transform.q);
	return AABB{p, p} + Vec2{shape.GetRadius(), shape.GetRadius()};
}
