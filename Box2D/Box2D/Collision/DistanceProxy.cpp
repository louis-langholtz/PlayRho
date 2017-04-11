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

#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

namespace box2d {

namespace {
	
	inline DistanceProxy GetDistanceProxy(const CircleShape& shape, child_count_t index)
	{
		NOT_USED(index);
		return DistanceProxy{GetVertexRadius(shape), shape.GetLocation()};
	}

	inline DistanceProxy GetDistanceProxy(const PolygonShape& shape, child_count_t index)
	{
		NOT_USED(index);
		return DistanceProxy{GetVertexRadius(shape), shape.GetVertices()};
	}

	inline DistanceProxy GetDistanceProxy(const ChainShape& shape, child_count_t index)
	{
		return DistanceProxy{GetVertexRadius(shape), shape.GetVertex(index), shape.GetVertex(GetNextIndex(shape, index))};
	}

	inline DistanceProxy GetDistanceProxy(const EdgeShape& shape, child_count_t index)
	{
		NOT_USED(index);
		return DistanceProxy{GetVertexRadius(shape), shape.GetVertex1(), shape.GetVertex2()};
	}
	
} // anonymous namespace
	
DistanceProxy::size_type GetSupportIndex(const DistanceProxy& proxy, const Length2D d) noexcept
{
	auto index = DistanceProxy::InvalidIndex; ///< Index of vertex that when dotted with d has the max value.
	auto maxValue = -MaxFloat * SquareMeter; ///< Max dot value.
	const auto count = proxy.GetVertexCount();
	for (auto i = decltype(count){0}; i < count; ++i)
	{
		const auto value = Dot(proxy.GetVertex(i), d);
		if (maxValue < value)
		{
			maxValue = value;
			index = i;
		}
	}
	return index;
}

DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index)
{
	switch (shape.GetType())
	{
		case Shape::e_circle: return GetDistanceProxy(*static_cast<const CircleShape*>(&shape), index);
		case Shape::e_polygon: return GetDistanceProxy(*static_cast<const PolygonShape*>(&shape), index);
		case Shape::e_chain: return GetDistanceProxy(*static_cast<const ChainShape*>(&shape), index);
		case Shape::e_edge: return GetDistanceProxy(*static_cast<const EdgeShape*>(&shape), index);
		case Shape::e_typeCount: break;
	}
	assert(false);
	return DistanceProxy{};
}

} // namespace box2d
