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

#include <Box2D/Collision/Shapes/Shape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {
	
	float_t GetVertexRadius(const Shape& shape)
	{
		assert(shape.GetType() < Shape::e_typeCount);
		switch (shape.GetType())
		{
			case Shape::e_circle: return GetVertexRadius(static_cast<const CircleShape&>(shape));
			case Shape::e_edge: return GetVertexRadius(static_cast<const EdgeShape&>(shape));
			case Shape::e_chain: return GetVertexRadius(static_cast<const ChainShape&>(shape));
			case Shape::e_polygon: return GetVertexRadius(static_cast<const PolygonShape&>(shape));
			case Shape::e_typeCount: return 0;
		}
	}

	child_count_t GetChildCount(const Shape& shape)
	{
		assert(shape.GetType() < Shape::e_typeCount);
		switch (shape.GetType())
		{
			case Shape::e_edge: return GetChildCount(static_cast<const EdgeShape&>(shape));
			case Shape::e_chain: return GetChildCount(static_cast<const ChainShape&>(shape));
			case Shape::e_circle: return GetChildCount(static_cast<const CircleShape&>(shape));
			case Shape::e_polygon: return GetChildCount(static_cast<const PolygonShape&>(shape));
			case Shape::e_typeCount: return 0;
		}
	}
	
	bool TestPoint(const Shape& shape, const Transformation& xf, const Vec2& p)
	{
		assert(shape.GetType() < Shape::e_typeCount);
		switch (shape.GetType())
		{
			case Shape::e_edge: return TestPoint(static_cast<const EdgeShape&>(shape), xf, p);
			case Shape::e_chain: return TestPoint(static_cast<const ChainShape&>(shape), xf, p);
			case Shape::e_circle: return TestPoint(static_cast<const CircleShape&>(shape), xf, p);
			case Shape::e_polygon: return TestPoint(static_cast<const PolygonShape&>(shape), xf, p);
			case Shape::e_typeCount: return false;
		}
	}
	
	RayCastOutput RayCast(const Shape& shape, const RayCastInput& input,
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
	
	AABB ComputeAABB(const Shape& shape, const Transformation& xf, child_count_t childIndex)
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
	
	MassData ComputeMass(const Shape& shape, float_t density)
	{
		assert(shape.GetType() < Shape::e_typeCount);
		switch (shape.GetType())
		{
			case Shape::e_edge: return ComputeMass(static_cast<const EdgeShape&>(shape), density);
			case Shape::e_chain: return ComputeMass(static_cast<const ChainShape&>(shape), density);
			case Shape::e_circle: return ComputeMass(static_cast<const CircleShape&>(shape), density);
			case Shape::e_polygon: return ComputeMass(static_cast<const PolygonShape&>(shape), density);
			case Shape::e_typeCount: return MassData{0, GetInvalid<Vec2>(), 0};
		}		
	}

} // namespace box2d
