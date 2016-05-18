/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
			m_transformA = b2Transform_identity;
			m_transformA.p = b2Vec2(0.0f, -0.2f);
			m_polygonA.SetAsBox(10.0f, 0.2f);
		}

		{
			m_positionB = b2Vec2(12.017401f, 0.13678508f);
			m_angleB = -0.0109265f;
			m_transformB = b2Transform{m_positionB, b2Rot(m_angleB)};

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

		b2DistanceInput input;
		input.proxyA = b2DistanceProxy(m_polygonA, 0);
		input.proxyB = b2DistanceProxy(m_polygonB, 0);
		input.transformA = m_transformA;
		input.transformB = m_transformB;
		input.useRadii = true;
		b2SimplexCache cache;
		const auto output = b2Distance(cache, input);

		g_debugDraw.DrawString(5, m_textLine, "distance = %g", output.distance);
		m_textLine += DRAW_STRING_NEW_LINE;

		g_debugDraw.DrawString(5, m_textLine, "iterations = %d", output.iterations);
		m_textLine += DRAW_STRING_NEW_LINE;

		{
			b2Color color(0.9f, 0.9f, 0.9f);
			b2Vec2 v[MaxPolygonVertices];
			for (int32 i = 0; i < m_polygonA.GetVertexCount(); ++i)
			{
				v[i] = b2Mul(m_transformA, m_polygonA.GetVertex(i));
			}
			g_debugDraw.DrawPolygon(v, m_polygonA.GetVertexCount(), color);

			for (int32 i = 0; i < m_polygonB.GetVertexCount(); ++i)
			{
				v[i] = b2Mul(m_transformB, m_polygonB.GetVertex(i));
			}
			g_debugDraw.DrawPolygon(v, m_polygonB.GetVertexCount(), color);
		}

		b2Vec2 x1 = output.pointA;
		b2Vec2 x2 = output.pointB;

		b2Color c1(1.0f, 0.0f, 0.0f);
		g_debugDraw.DrawPoint(x1, 4.0f, c1);

		b2Color c2(1.0f, 1.0f, 0.0f);
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

		m_transformB = b2Transform{m_positionB, b2Rot(m_angleB)};
	}

	b2Vec2 m_positionB;
	float_t m_angleB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;
};
	
} // namespace box2d

#endif
