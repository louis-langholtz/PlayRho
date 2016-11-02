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

#ifndef POLY_SHAPES_H
#define POLY_SHAPES_H

/// This tests stacking. It also shows how to use World::Query
/// and TestOverlap.

namespace box2d {

/// This callback is called by World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use TestOverlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
class PolyShapesCallback : public QueryFixtureReporter
{
public:
	
	enum
	{
		e_maxCount = 4
	};

	PolyShapesCallback()
	{
		m_count = 0;
	}

	void DrawFixture(Fixture* fixture)
	{
		Color color(0.95f, 0.95f, 0.6f);
		const Transformation& xf = fixture->GetBody()->GetTransformation();

		switch (GetType(*fixture))
		{
		case Shape::e_circle:
			{
				CircleShape* circle = (CircleShape*)fixture->GetShape();

				Vec2 center = Transform(circle->GetPosition(), xf);
				float_t radius = circle->GetRadius();

				g_debugDraw->DrawCircle(center, radius, color);
			}
			break;

		case Shape::e_polygon:
			{
				PolygonShape* poly = (PolygonShape*)fixture->GetShape();
				const auto vertexCount = poly->GetVertexCount();
				assert(vertexCount <= MaxPolygonVertices);
				Vec2 vertices[MaxPolygonVertices];

				for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
				{
					vertices[i] = Transform(poly->GetVertex(i), xf);
				}

				g_debugDraw->DrawPolygon(vertices, vertexCount, color);
			}
			break;
				
		default:
			break;
		}
	}

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	bool ReportFixture(Fixture* fixture)
	{
		if (m_count == e_maxCount)
		{
			return false;
		}

		Body* body = fixture->GetBody();
		Shape* shape = fixture->GetShape();

		bool overlap = TestOverlap(*shape, 0, body->GetTransformation(), m_circle, 0, m_transform);

		if (overlap)
		{
			DrawFixture(fixture);
			++m_count;
		}

		return true;
	}

	CircleShape m_circle;
	Transformation m_transform;
	Drawer* g_debugDraw;
	int32 m_count;
};

class PolyShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	PolyShapes()
	{
		// Ground body
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			m_polygons[0].Set({Vec2(-0.5f, 0.0f), Vec2(0.5f, 0.0f), Vec2(0.0f, 1.5f)});
		}
		
		{
			m_polygons[1].Set({Vec2(-0.1f, 0.0f), Vec2(0.1f, 0.0f), Vec2(0.0f, 1.5f)});
		}

		{
			float_t w = 1.0f;
			float_t b = w / (2.0f + Sqrt(2.0f));
			float_t s = Sqrt(2.0f) * b;

			m_polygons[2].Set({
				Vec2(0.5f * s, 0.0f),
				Vec2(0.5f * w, b),
				Vec2(0.5f * w, b + s),
				Vec2(0.5f * s, w),
				Vec2(-0.5f * s, w),
				Vec2(-0.5f * w, b + s),
				Vec2(-0.5f * w, b),
				Vec2(-0.5f * s, 0.0f)
			});
		}

		{
			m_polygons[3].SetAsBox(0.5f, 0.5f);
		}

		{
			m_circle.SetRadius(float_t(0.5));
		}

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));
	}

	void Create(int32 index)
	{
		if (m_bodies[m_bodyIndex] != nullptr)
		{
			m_world->Destroy(m_bodies[m_bodyIndex]);
			m_bodies[m_bodyIndex] = nullptr;
		}

		BodyDef bd;
		bd.type = BodyType::Dynamic;

		float_t x = RandomFloat(-2.0f, 2.0f);
		bd.position = Vec2(x, 10.0f);
		bd.angle = RandomFloat(-Pi, Pi);

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = m_world->CreateBody(bd);

		if (index < 4)
		{
			FixtureDef fd;
			fd.shape = m_polygons + index;
			fd.density = 1.0f;
			fd.friction = 0.3f;
			m_bodies[m_bodyIndex]->CreateFixture(fd);
		}
		else
		{
			FixtureDef fd;
			fd.shape = &m_circle;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			m_bodies[m_bodyIndex]->CreateFixture(fd);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
	}

	void Destroy()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (m_bodies[i] != nullptr)
			{
				m_world->Destroy(m_bodies[i]);
				m_bodies[i] = nullptr;
				return;
			}
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
		case GLFW_KEY_5:
			Create(key - GLFW_KEY_1);
			break;

		case GLFW_KEY_A:
			for (int32 i = 0; i < e_maxBodies; i += 2)
			{
				if (m_bodies[i])
				{
					bool active = m_bodies[i]->IsActive();
					m_bodies[i]->SetActive(!active);
				}
			}
			break;

		case GLFW_KEY_D:
			Destroy();
			break;
		}
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		PolyShapesCallback callback;
		callback.m_circle.SetRadius(float_t(2.0));
		callback.m_circle.SetPosition(Vec2(0.0f, 1.1f));
		callback.m_transform = Transform_identity;
		callback.g_debugDraw = &drawer;

		AABB aabb = ComputeAABB(callback.m_circle, callback.m_transform, 0);

		m_world->QueryAABB(&callback, aabb);

		Color color(0.4f, 0.7f, 0.8f);
		drawer.DrawCircle(callback.m_circle.GetPosition(), callback.m_circle.GetRadius(), color);

		drawer.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Press 'a' to (de)activate some bodies");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Press 'd' to destroy a body");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new PolyShapes;
	}

	int32 m_bodyIndex;
	Body* m_bodies[e_maxBodies];
	PolygonShape m_polygons[4];
	CircleShape m_circle;
};

} // namespace box2d

#endif
