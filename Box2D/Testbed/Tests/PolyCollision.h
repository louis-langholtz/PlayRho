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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			m_polygonA.SetAsBox(0.2f, 0.4f);
			m_transformA.Set(b2Vec2(0.0f, 0.0f), 0.0f);
		}

		{
			m_polygonB.SetAsBox(0.5f, 0.5f);
			m_positionB = b2Vec2(19.345284f, 1.5632932f);
			m_angleB = 1.9160721f;
			m_transformB.Set(m_positionB, m_angleB);
		}
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(Settings* settings)
	{
		B2_NOT_USED(settings);

		b2Manifold manifold;
		b2CollideShapes(&manifold, m_polygonA, m_transformA, m_polygonB, m_transformB);

		g_debugDraw.DrawString(5, m_textLine, "point count = %d", manifold.GetPointCount());
		m_textLine += DRAW_STRING_NEW_LINE;

		{
			b2Color color(0.9f, 0.9f, 0.9f);
			b2Vec2 v[b2_maxPolygonVertices];
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

		const b2WorldManifold worldManifold(manifold, m_transformA, m_polygonA.GetRadius(), m_transformB, m_polygonB.GetRadius());
		for (int32 i = 0; i < manifold.GetPointCount(); ++i)
		{
			g_debugDraw.DrawPoint(worldManifold.GetPoint(i), 4.0f, b2Color(0.9f, 0.3f, 0.3f));
		}
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
			m_angleB += 0.1f * b2_pi;
			break;

		case GLFW_KEY_E:
			m_angleB -= 0.1f * b2_pi;
			break;
		}

		m_transformB.Set(m_positionB, m_angleB);
	}

	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;

	b2Transform m_transformA;
	b2Transform m_transformB;

	b2Vec2 m_positionB;
	b2Float m_angleB;
};

#endif
