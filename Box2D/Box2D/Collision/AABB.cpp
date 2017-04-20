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
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

AABB box2d::ComputeAABB(const EdgeShape& shape, const Transformation xf, child_count_t)
{
	auto result = AABB{Transform(shape.GetVertex1(), xf)};
	result.Include(Transform(shape.GetVertex2(), xf));
	return result.Fatten(GetVertexRadius(shape));
}

AABB box2d::ComputeAABB(const PolygonShape& shape, const Transformation xf, child_count_t)
{
	const auto count = shape.GetVertexCount();
	assert(count > 0);
	auto result = AABB{Transform(shape.GetVertex(0), xf)};
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		result.Include(Transform(shape.GetVertex(i), xf));
	}
	return result.Fatten(GetVertexRadius(shape));
}

AABB box2d::ComputeAABB(const DistanceProxy& proxy, const Transformation xf)
{
	const auto count = proxy.GetVertexCount();
	assert(count > 0);
	auto result = AABB{Transform(proxy.GetVertex(0), xf)};
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		result.Include(Transform(proxy.GetVertex(i), xf));
	}
	return result.Fatten(proxy.GetVertexRadius());
}

AABB box2d::ComputeAABB(const ChainShape& shape, const Transformation xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());
	
	auto result = AABB{Transform(shape.GetVertex(childIndex), xf)};
	result.Include(Transform(shape.GetVertex(GetNextIndex(shape, childIndex)), xf));
	return result.Fatten(GetVertexRadius(shape));
}

AABB box2d::ComputeAABB(const CircleShape& shape, const Transformation transform, child_count_t)
{
	return GetFattenedAABB(AABB{Transform(shape.GetLocation(), transform)}, shape.GetRadius());
}

AABB box2d::ComputeAABB(const Shape& shape, const Transformation xf, child_count_t childIndex)
{
	assert(shape.GetType() < Shape::e_typeCount);
	switch (shape.GetType())
	{
		case Shape::e_edge: return ComputeAABB(static_cast<const EdgeShape&>(shape), xf, childIndex);
		case Shape::e_chain: return ComputeAABB(static_cast<const ChainShape&>(shape), xf, childIndex);
		case Shape::e_circle: return ComputeAABB(static_cast<const CircleShape&>(shape), xf, childIndex);
		case Shape::e_polygon: return ComputeAABB(static_cast<const PolygonShape&>(shape), xf, childIndex);
		default: break;
	}
	return GetInvalid<AABB>();
}

AABB box2d::ComputeAABB(const Shape& shape, const Transformation xf)
{
	const auto childCount = GetChildCount(shape);
	auto sum = AABB{};
	for (auto i = decltype(childCount){0}; i < childCount; ++i)
	{
		sum.Include(ComputeAABB(shape, xf, i));
	}
	return sum;
}

AABB box2d::ComputeAABB(const Body& body)
{
	auto sum = AABB{};
	const auto xf = body.GetTransformation();
	for (auto&& f: body.GetFixtures())
	{
		sum.Include(ComputeAABB(*(f->GetShape()), xf));
	}
	return sum;
}
