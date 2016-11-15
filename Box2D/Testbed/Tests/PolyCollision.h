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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

namespace box2d {

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			m_polygonA.SetAsBox(0.2f, 0.4f);
			m_transformA = Transformation{Vec2(0.0f, 0.0f), UnitVec2{0_rad}};
		}

		{
			m_polygonB.SetAsBox(0.5f, 0.5f);
			m_positionB = Vec2(19.345284f, 1.5632932f);
			m_angleB = 1.9160721_rad;
			m_transformB = Transformation{m_positionB, UnitVec2{m_angleB}};
		}
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		BOX2D_NOT_USED(settings);

		const auto manifold = CollideShapes(m_polygonA, m_transformA, m_polygonB, m_transformB);
		const auto pointCount = manifold.GetPointCount();

		drawer.DrawString(5, m_textLine, "point count = %d", pointCount);
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
				drawer.DrawPolygon(v, vertexCount, color);
			}
		}

		const auto worldManifold = GetWorldManifold(manifold,
													m_transformA, GetVertexRadius(m_polygonA),
													m_transformB, GetVertexRadius(m_polygonB));
		for (auto i = decltype(pointCount){0}; i < pointCount; ++i)
		{
			drawer.DrawPoint(worldManifold.GetPoint(i), 4.0f, Color(0.9f, 0.3f, 0.3f));
		}
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

	PolygonShape m_polygonA;
	PolygonShape m_polygonB;

	Transformation m_transformA;
	Transformation m_transformB;

	Vec2 m_positionB;
	Angle m_angleB;
};

} // namespace box2d

#endif
