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

#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Dynamics/Fixture.hpp>

namespace box2d {
	
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
	
	bool TestPoint(const Shape& shape, const Transformation& xf, const Length2D p)
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

	bool TestOverlap(const Shape& shapeA, child_count_t indexA, const Transformation& xfA,
					 const Shape& shapeB, child_count_t indexB, const Transformation& xfB)
	{
		const auto proxyA = GetDistanceProxy(shapeA, indexA);
		const auto proxyB = GetDistanceProxy(shapeB, indexB);
		
		const auto distanceInfo = Distance(proxyA, xfA, proxyB, xfB);
		assert(distanceInfo.state != DistanceOutput::Unknown && distanceInfo.state != DistanceOutput::HitMaxIters);

		const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);
		const auto distanceSquared = GetLengthSquared(StripUnits(witnessPoints.a - witnessPoints.b));
		const auto totalRadiusSquared = Square((proxyA.GetRadius() + proxyB.GetRadius()) / Meter);
		const auto separation_amount = distanceSquared - totalRadiusSquared;
		return (separation_amount < 0) || almost_zero(separation_amount);
	}

	Shape::Type GetType(const Fixture& fixture) noexcept
	{
		return fixture.GetShape()->GetType();
	}

} // namespace box2d
