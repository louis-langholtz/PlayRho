/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef EDGE_SHAPES_H
#define EDGE_SHAPES_H

namespace box2d {

class EdgeShapesCallback : public RayCastFixtureReporter
{
public:
	EdgeShapesCallback()
	{
		m_fixture = nullptr;
	}

	float_t ReportFixture(Fixture* fixture, const Vec2& point,
						  const Vec2& normal, float_t fraction)
	{
		m_fixture = fixture;
		m_point = point;
		m_normal = normal;

		return fraction;
	}

	Fixture* m_fixture;
	Vec2 m_point;
	Vec2 m_normal;
};

class EdgeShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	EdgeShapes()
	{
		// Ground body
		{
			BodyDef bd;
			Body* ground = m_world->Create(bd);

			float_t x1 = -20.0f;
			float_t y1 = 2.0f * cosf(x1 / 10.0f * Pi);
			for (int32 i = 0; i < 80; ++i)
			{
				float_t x2 = x1 + 0.5f;
				float_t y2 = 2.0f * cosf(x2 / 10.0f * Pi);

				EdgeShape shape;
				shape.Set(Vec2(x1, y1), Vec2(x2, y2));
				ground->CreateFixture(FixtureDef{&shape, 0.0f});

				x1 = x2;
				y1 = y2;
			}
		}

		{
		Vec2 vertices[3];
		vertices[0] = Vec2(-0.5f, 0.0f);
		vertices[1] = Vec2(0.5f, 0.0f);
		vertices[2] = Vec2(0.0f, 1.5f);
		m_polygons[0].Set(vertices, 3);
	}

		{
			Vec2 vertices[3];
			vertices[0] = Vec2(-0.1f, 0.0f);
			vertices[1] = Vec2(0.1f, 0.0f);
			vertices[2] = Vec2(0.0f, 1.5f);
			m_polygons[1].Set(vertices, 3);
		}

		{
			float_t w = 1.0f;
			float_t b = w / (2.0f + Sqrt(2.0f));
			float_t s = Sqrt(2.0f) * b;

			Vec2 vertices[8];
			vertices[0] = Vec2(0.5f * s, 0.0f);
			vertices[1] = Vec2(0.5f * w, b);
			vertices[2] = Vec2(0.5f * w, b + s);
			vertices[3] = Vec2(0.5f * s, w);
			vertices[4] = Vec2(-0.5f * s, w);
			vertices[5] = Vec2(-0.5f * w, b + s);
			vertices[6] = Vec2(-0.5f * w, b);
			vertices[7] = Vec2(-0.5f * s, 0.0f);

			m_polygons[2].Set(vertices, 8);
		}

		{
			m_polygons[3].SetAsBox(0.5f, 0.5f);
		}

		{
			m_circle.SetRadius(float_t(0.5));
		}

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));

		m_angle = 0.0f;
	}

	void Create(int32 index)
	{
		if (m_bodies[m_bodyIndex] != nullptr)
		{
			m_world->Destroy(m_bodies[m_bodyIndex]);
			m_bodies[m_bodyIndex] = nullptr;
		}

		BodyDef bd;

		float_t x = RandomFloat(-10.0f, 10.0f);
		float_t y = RandomFloat(10.0f, 20.0f);
		bd.position = Vec2(x, y);
		bd.angle = RandomFloat(-Pi, Pi);
		bd.type = BodyType::Dynamic;

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = m_world->Create(bd);

		if (index < 4)
		{
			FixtureDef fd;
			fd.shape = m_polygons + index;
			fd.friction = 0.3f;
			fd.density = 20.0f;
			m_bodies[m_bodyIndex]->CreateFixture(fd);
		}
		else
		{
			FixtureDef fd;
			fd.shape = &m_circle;
			fd.friction = 0.3f;
			fd.density = 20.0f;
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

	void Keyboard(int key)
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

		case GLFW_KEY_D:
			Destroy();
			break;
		}
	}

	void Step(Settings* settings)
	{
		bool advanceRay = !settings->pause || settings->singleStep;

		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += DRAW_STRING_NEW_LINE;

		float_t L = 25.0f;
		Vec2 point1(0.0f, 10.0f);
		Vec2 d(L * cosf(m_angle), -L * Abs(sinf(m_angle)));
		Vec2 point2 = point1 + d;

		EdgeShapesCallback callback;

		m_world->RayCast(&callback, point1, point2);

		if (callback.m_fixture)
		{
			g_debugDraw.DrawPoint(callback.m_point, 5.0f, Color(0.4f, 0.9f, 0.4f));

			g_debugDraw.DrawSegment(point1, callback.m_point, Color(0.8f, 0.8f, 0.8f));

			Vec2 head = callback.m_point + 0.5f * callback.m_normal;
			g_debugDraw.DrawSegment(callback.m_point, head, Color(0.9f, 0.9f, 0.4f));
		}
		else
		{
			g_debugDraw.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
		}

		if (advanceRay)
		{
			m_angle += 0.25f * Pi / 180.0f;
		}
	}

	static Test* Create()
	{
		return new EdgeShapes;
	}

	int32 m_bodyIndex;
	Body* m_bodies[e_maxBodies];
	PolygonShape m_polygons[4];
	CircleShape m_circle;

	float_t m_angle;
};

} // namespace box2d

#endif
