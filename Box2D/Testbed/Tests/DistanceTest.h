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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

namespace box2d {

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		{
			m_transformA = Transform_identity;
			m_transformA.p = Vec2(0.0f, -0.2f);
			m_polygonA.SetAsBox(10.0f, 0.2f);
		}

		{
			m_positionB = Vec2(12.017401f, 0.13678508f);
			m_angleB = -0.0109265_rad;
			m_transformB = Transformation{m_positionB, UnitVec2{m_angleB}};

			m_polygonB.SetAsBox(2.0f, 0.1f);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		const auto proxyA = GetDistanceProxy(m_polygonA, 0);
		const auto proxyB = GetDistanceProxy(m_polygonB, 0);
		const auto transformA = m_transformA;
		const auto transformB = m_transformB;

		SimplexCache cache;
		auto output = Distance(proxyA, transformA, proxyB, transformB, cache);
		auto distance = Sqrt(GetLengthSquared(output.witnessPoints.a - output.witnessPoints.b));
		
		const auto rA = proxyA.GetRadius();
		const auto rB = proxyB.GetRadius();
		const auto totalRadius = rA + rB;
		
		if ((distance > totalRadius) && !almost_zero(distance))
		{
			// Shapes are still not overlapped.
			// Move the witness points to the outer surface.
			distance -= totalRadius;
			const auto normal = GetUnitVector(output.witnessPoints.b - output.witnessPoints.a);
			output.witnessPoints.a += rA * normal;
			output.witnessPoints.b -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			const auto p = (output.witnessPoints.a + output.witnessPoints.b) / float_t{2};
			output.witnessPoints.a = p;
			output.witnessPoints.b = p;
			distance = float_t{0};
		}
		
		drawer.DrawString(5, m_textLine, "distance = %g", distance);
		m_textLine += DRAW_STRING_NEW_LINE;

		drawer.DrawString(5, m_textLine, "iterations = %d", output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;

		{
			Color color(0.9f, 0.9f, 0.9f);
			Vec2 v[MaxPolygonVertices];
			{
				const auto vertexCount = m_polygonA.GetVertexCount();
				for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
				{
					v[i] = Transform(m_polygonA.GetVertex(i), m_transformA);
				}
				drawer.DrawPolygon(v, vertexCount, color);
			}

			{
				const auto vertexCount = m_polygonB.GetVertexCount();
				for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
				{
					v[i] = Transform(m_polygonB.GetVertex(i), m_transformB);
				}
				drawer.DrawPolygon(v, m_polygonB.GetVertexCount(), color);
			}
		}

		Vec2 x1 = output.witnessPoints.a;
		Vec2 x2 = output.witnessPoints.b;

		Color c1(1.0f, 0.0f, 0.0f);
		drawer.DrawPoint(x1, 4.0f, c1);

		Color c2(1.0f, 1.0f, 0.0f);
		drawer.DrawPoint(x2, 4.0f, c2);
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_positionB.x -= 0.1f;
			break;

		case Key_D:
			m_positionB.x += 0.1f;
			break;

		case Key_S:
			m_positionB.y -= 0.1f;
			break;

		case Key_W:
			m_positionB.y += 0.1f;
			break;

		case Key_Q:
			m_angleB += 0.1_rad * Pi;
			break;

		case Key_E:
			m_angleB -= 0.1_rad * Pi;
			break;
				
		default:
			break;
		}

		m_transformB = Transformation{m_positionB, UnitVec2{m_angleB}};
	}

	Vec2 m_positionB;
	Angle m_angleB;

	Transformation m_transformA;
	Transformation m_transformB;
	PolygonShape m_polygonA;
	PolygonShape m_polygonB;
};
	
} // namespace box2d

#endif
