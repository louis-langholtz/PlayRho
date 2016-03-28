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

void b2CollideCircles(
	b2Manifold* manifold,
	const b2CircleShape* circleA, const b2Transform& xfA,
	const b2CircleShape* circleB, const b2Transform& xfB)
{
	const auto pA = b2Mul(xfA, circleA->GetPosition());
	const auto pB = b2Mul(xfB, circleB->GetPosition());

	const auto d = pB - pA;
	const auto distSqr = b2Dot(d, d);
	const auto rA = circleA->GetRadius(), rB = circleB->GetRadius();
	const auto radius = rA + rB;
	if (distSqr > (radius * radius))
	{
		manifold->ClearPoints();
		return;
	}

	manifold->SetType(b2Manifold::e_circles);
	manifold->SetLocalPoint(circleA->GetPosition());
	manifold->SetLocalNormal(b2Vec2_zero);
	manifold->ClearPoints();
	manifold->AddPoint(circleB->GetPosition());
}

void b2CollidePolygonAndCircle(
	b2Manifold* manifold,
	const b2PolygonShape* polygonA, const b2Transform& xfA,
	const b2CircleShape* circleB, const b2Transform& xfB)
{
	// Compute circle position in the frame of the polygon.
	const auto c = b2Mul(xfB, circleB->GetPosition());
	const auto cLocal = b2MulT(xfA, c);

	// Find the min separating edge.
	auto normalIndex = int32{0};
	auto separation = -b2_maxFloat;
	const auto radius = polygonA->GetRadius() + circleB->GetRadius();
	const auto vertexCount = polygonA->GetVertexCount();
	const auto vertices = polygonA->GetVertices();
	const auto normals = polygonA->GetNormals();

	for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
	{
		const auto s = b2Dot(normals[i], cLocal - vertices[i]);

		if (s > radius)
		{
			// Early out.
			manifold->ClearPoints();
			return;
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
	const auto v1 = vertices[vertIndex1];
	const auto v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < b2_epsilon)
	{
		manifold->SetType(b2Manifold::e_faceA);
		manifold->SetLocalNormal(normals[normalIndex]);
		manifold->SetLocalPoint(0.5f * (v1 + v2));
		manifold->ClearPoints();
		manifold->AddPoint(circleB->GetPosition());
		return;
	}

	// Compute barycentric coordinates
	const auto u1 = b2Dot(cLocal - v1, v2 - v1);
	const auto u2 = b2Dot(cLocal - v2, v1 - v2);
	if (u1 <= 0.0f)
	{
		if (b2DistanceSquared(cLocal, v1) > (radius * radius))
		{
			manifold->ClearPoints();
			return;
		}

		manifold->SetType(b2Manifold::e_faceA);
		manifold->SetLocalNormal(b2Normalize(cLocal - v1));
		manifold->SetLocalPoint(v1);
		manifold->ClearPoints();
		manifold->AddPoint(circleB->GetPosition());
	}
	else if (u2 <= 0.0f)
	{
		if (b2DistanceSquared(cLocal, v2) > (radius * radius))
		{
			manifold->ClearPoints();
			return;
		}

		manifold->SetType(b2Manifold::e_faceA);
		manifold->SetLocalNormal(b2Normalize(cLocal - v2));
		manifold->SetLocalPoint(v2);
		manifold->ClearPoints();
		manifold->AddPoint(circleB->GetPosition());
	}
	else
	{
		const auto faceCenter = 0.5f * (v1 + v2);
		separation = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
		if (separation > radius)
		{
			manifold->ClearPoints();
			return;
		}

		manifold->SetType(b2Manifold::e_faceA);
		manifold->SetLocalNormal(normals[vertIndex1]);
		manifold->SetLocalPoint(faceCenter);
		manifold->ClearPoints();
		manifold->AddPoint(circleB->GetPosition());
	}
}
