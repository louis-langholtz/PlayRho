/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

namespace box2d {

b2Manifold b2CollideShapes(const b2CircleShape& shapeA, const Transform& xfA, const b2CircleShape& shapeB, const Transform& xfB)
{
	const auto pA = Mul(xfA, shapeA.GetPosition());
	const auto pB = Mul(xfB, shapeB.GetPosition());
	const auto d = pB - pA;
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	if (d.LengthSquared() > Square(totalRadius))
	{
		return b2Manifold{};
	}

	auto manifold = b2Manifold{b2Manifold::e_circles, Vec2_zero, shapeA.GetPosition()};
	manifold.AddPoint(shapeB.GetPosition());
	return manifold;
}

b2Manifold b2CollideShapes(const b2PolygonShape& shapeA, const Transform& xfA, const b2CircleShape& shapeB, const Transform& xfB)
{
	// Compute circle position in the frame of the polygon.
	const auto c = Mul(xfB, shapeB.GetPosition());
	const auto cLocal = MulT(xfA, c);

	// Find the min separating edge.
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();
	const auto vertexCount = shapeA.GetVertexCount();
	auto normalIndex = decltype(vertexCount){0};
	auto separation = -MaxFloat;

	for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
	{
		const auto s = Dot(shapeA.GetNormal(i), cLocal - shapeA.GetVertex(i));

		if (s > totalRadius)
		{
			// Early out.
			return b2Manifold{};
		}

		if (separation < s)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	const auto vertIndex1 = normalIndex;
	const auto vertIndex2 = ((vertIndex1 + 1) < vertexCount) ? vertIndex1 + 1 : 0;
	const auto v1 = shapeA.GetVertex(vertIndex1);
	const auto v2 = shapeA.GetVertex(vertIndex2);

	// If the center is inside the polygon ...
	if (separation < Epsilon)
	{
		auto manifold = b2Manifold{b2Manifold::e_faceA, shapeA.GetNormal(normalIndex), (v1 + v2) / float_t(2)};
		manifold.AddPoint(shapeB.GetPosition());
		return manifold;
	}

	// Compute barycentric coordinates
	const auto u1 = Dot(cLocal - v1, v2 - v1);
	const auto u2 = Dot(cLocal - v2, v1 - v2);
	if (u1 <= float_t{0})
	{
		if (DistanceSquared(cLocal, v1) > Square(totalRadius))
		{
			return b2Manifold{};
		}

		auto manifold = b2Manifold{b2Manifold::e_faceA, Normalize(cLocal - v1), v1};
		manifold.AddPoint(shapeB.GetPosition());
		return manifold;
	}
	if (u2 <= float_t{0})
	{
		if (DistanceSquared(cLocal, v2) > Square(totalRadius))
		{
			return b2Manifold{};
		}

		auto manifold = b2Manifold{b2Manifold::e_faceA, Normalize(cLocal - v2), v2};
		manifold.AddPoint(shapeB.GetPosition());
		return manifold;
	}

	const auto faceCenter = (v1 + v2) / float_t(2);
	separation = Dot(cLocal - faceCenter, shapeA.GetNormal(vertIndex1));
	if (separation > totalRadius)
	{
		return b2Manifold{};
	}

	auto manifold = b2Manifold{b2Manifold::e_faceA};
	manifold.SetLocalNormal(shapeA.GetNormal(vertIndex1));
	manifold.SetLocalPoint(faceCenter);
	manifold.AddPoint(shapeB.GetPosition());
	return manifold;
}
	
} // namespace box2d

