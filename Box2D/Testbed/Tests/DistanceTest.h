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
			m_angleB = -0.0109265f;
			m_transformB = Transform{m_positionB, Rot(m_angleB)};

			m_polygonB.SetAsBox(2.0f, 0.1f);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		DistanceInput input;
		input.proxyA = GetDistanceProxy(m_polygonA, 0);
		input.proxyB = GetDistanceProxy(m_polygonB, 0);
		input.transformA = m_transformA;
		input.transformB = m_transformB;
		input.useRadii = true;
		SimplexCache cache;
		const auto output = Distance(cache, input);

		g_debugDraw.DrawString(5, m_textLine, "distance = %g", output.distance);
		m_textLine += DRAW_STRING_NEW_LINE;

		g_debugDraw.DrawString(5, m_textLine, "iterations = %d", output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;

		{
			Color color(0.9f, 0.9f, 0.9f);
			Vec2 v[MaxPolygonVertices];
			{
				const auto vertexCount = m_polygonA.GetVertexCount();
				for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
				{
					v[i] = Mul(m_transformA, m_polygonA.GetVertex(i));
				}
				g_debugDraw.DrawPolygon(v, vertexCount, color);
			}

			{
				const auto vertexCount = m_polygonB.GetVertexCount();
				for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
				{
					v[i] = Mul(m_transformB, m_polygonB.GetVertex(i));
				}
				g_debugDraw.DrawPolygon(v, m_polygonB.GetVertexCount(), color);
			}
		}

		Vec2 x1 = output.witnessPoints.a;
		Vec2 x2 = output.witnessPoints.b;

		Color c1(1.0f, 0.0f, 0.0f);
		g_debugDraw.DrawPoint(x1, 4.0f, c1);

		Color c2(1.0f, 1.0f, 0.0f);
		g_debugDraw.DrawPoint(x2, 4.0f, c2);
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_positionB.x -= 0.1f;
			break;

		case GLFW_KEY_D:
			m_positionB.x += 0.1f;
			break;

		case GLFW_KEY_S:
			m_positionB.y -= 0.1f;
			break;

		case GLFW_KEY_W:
			m_positionB.y += 0.1f;
			break;

		case GLFW_KEY_Q:
			m_angleB += 0.1f * Pi;
			break;

		case GLFW_KEY_E:
			m_angleB -= 0.1f * Pi;
			break;
		}

		m_transformB = Transform{m_positionB, Rot(m_angleB)};
	}

	Vec2 m_positionB;
	float_t m_angleB;

	Transform m_transformA;
	Transform m_transformB;
	PolygonShape m_polygonA;
	PolygonShape m_polygonB;
};
	
} // namespace box2d

#endif
