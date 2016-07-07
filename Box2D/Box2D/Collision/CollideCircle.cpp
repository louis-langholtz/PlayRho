/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Collision.h>
#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Collision/Shapes/PolygonShape.h>

namespace box2d {

Manifold CollideShapes(const CircleShape& shapeA, const Transformation& xfA, const CircleShape& shapeB, const Transformation& xfB)
{
	const auto pA = Transform(shapeA.GetPosition(), xfA);
	const auto pB = Transform(shapeB.GetPosition(), xfB);
	const auto d = pB - pA;
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();

	if (LengthSquared(d) > Square(totalRadius))
	{
		return Manifold{};
	}
	return Manifold::GetForCircles(shapeA.GetPosition(), ManifoldPoint{shapeB.GetPosition()});
}

Manifold CollideShapes(const PolygonShape& shapeA, const Transformation& xfA, const CircleShape& shapeB, const Transformation& xfB)
{
	// Compute circle position in the frame of the polygon.
	const auto cLocal = InverseTransform(Transform(shapeB.GetPosition(), xfB), xfA); ///< Center of the circle in the frame of the polygon.

	// Find the min separating edge.
	const auto totalRadius = shapeA.GetRadius() + shapeB.GetRadius();
	const auto vertexCount = shapeA.GetVertexCount();
	auto normalIndex = decltype(vertexCount){0};
	auto maxSeparation = -MaxFloat;

	for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
	{
		const auto s = Dot(shapeA.GetNormal(i), cLocal - shapeA.GetVertex(i));
		if (s > totalRadius)
		{
			// Early out.
			return Manifold{};
		}
		if (maxSeparation < s)
		{
			maxSeparation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	const auto vertIndex1 = normalIndex;
	const auto vertIndex2 = static_cast<decltype(vertIndex1)>((vertIndex1 + 1) % vertexCount);
	const auto v1 = shapeA.GetVertex(vertIndex1);
	const auto v2 = shapeA.GetVertex(vertIndex2);

	// If the center is inside the polygon ...
	if (maxSeparation < Epsilon)
	{
		return Manifold::GetForFaceA(shapeA.GetNormal(normalIndex), (v1 + v2) / 2, ManifoldPoint{shapeB.GetPosition()});
	}

	// Compute barycentric coordinates

	if (Dot(cLocal - v1, v2 - v1) <= float_t{0})
	{
		if (LengthSquared(cLocal - v1) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForFaceA(GetUnitVector(cLocal - v1), v1, ManifoldPoint{shapeB.GetPosition()});
	}

	if (Dot(cLocal - v2, v1 - v2) <= float_t{0})
	{
		if (LengthSquared(cLocal - v2) > Square(totalRadius))
		{
			return Manifold{};
		}
		return Manifold::GetForFaceA(GetUnitVector(cLocal - v2), v2, ManifoldPoint{shapeB.GetPosition()});
	}

	const auto faceCenter = (v1 + v2) / 2;
	if (Dot(cLocal - faceCenter, shapeA.GetNormal(vertIndex1)) > totalRadius)
	{
		return Manifold{};
	}
	return Manifold::GetForFaceA(shapeA.GetNormal(vertIndex1), faceCenter, ManifoldPoint{shapeB.GetPosition()});
}
	
} // namespace box2d

