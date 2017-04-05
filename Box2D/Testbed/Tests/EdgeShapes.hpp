/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <cmath>

namespace box2d {

class EdgeShapesCallback : public RayCastFixtureReporter
{
public:
	EdgeShapesCallback()
	{
		m_fixture = nullptr;
	}

	RealNum ReportFixture(Fixture* fixture, const Vec2& point,
						  const UnitVec2& normal, RealNum fraction) override
	{
		m_fixture = fixture;
		m_point = point;
		m_normal = normal;

		return fraction;
	}

	Fixture* m_fixture;
	Vec2 m_point;
	UnitVec2 m_normal;
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
		m_circle->SetFriction(0.3f);
		m_circle->SetDensity(RealNum{20} * KilogramPerSquareMeter);

		// Ground body
		{
			const auto ground = m_world->CreateBody();
			auto x1 = -20.0f;
			auto y1 = 2.0f * cosf(x1 / 10.0f * Pi);
			for (auto i = 0; i < 80; ++i)
			{
				const auto x2 = x1 + 0.5f;
				const auto y2 = 2.0f * cosf(x2 / 10.0f * Pi);
				ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(x1, y1), Vec2(x2, y2)));
				x1 = x2;
				y1 = y2;
			}
		}

		for (auto i = 0; i < 4; ++i)
		{
			m_polygons[i].SetFriction(0.3f);
			m_polygons[i].SetDensity(RealNum{20} * KilogramPerSquareMeter);
		}
		
		m_polygons[0].Set({Vec2(-0.5f, 0.0f), Vec2(0.5f, 0.0f), Vec2(0.0f, 1.5f)});
		m_polygons[1].Set({Vec2(-0.1f, 0.0f), Vec2(0.1f, 0.0f), Vec2(0.0f, 1.5f)});

		{
			const auto w = 1.0f;
			const auto b = w / (2.0f + Sqrt(2.0f));
			const auto s = Sqrt(2.0f) * b;

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

		m_polygons[3].SetAsBox(0.5f, 0.5f);

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));

		m_angle = 0.0f;
	}

	void Create(int32 index)
	{
		if (m_bodies[m_bodyIndex])
		{
			m_world->Destroy(m_bodies[m_bodyIndex]);
			m_bodies[m_bodyIndex] = nullptr;
		}

		BodyDef bd;

		const auto x = RandomFloat(-10.0f, 10.0f);
		const auto y = RandomFloat(10.0f, 20.0f);
		bd.position = Vec2(x, y);
		bd.angle = Radian * RandomFloat(-Pi, Pi);
		bd.type = BodyType::Dynamic;

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = m_world->CreateBody(bd);

		if (index < 4)
		{
			m_bodies[m_bodyIndex]->CreateFixture(std::make_shared<PolygonShape>(m_polygons[index]));
		}
		else
		{
			m_bodies[m_bodyIndex]->CreateFixture(m_circle);
		}

		m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
	}

	void Destroy()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (m_bodies[i])
			{
				m_world->Destroy(m_bodies[i]);
				m_bodies[i] = nullptr;
				return;
			}
		}
	}

	void KeyboardDown(Key key) override
	{
		switch (key)
		{
		case Key_1:
		case Key_2:
		case Key_3:
		case Key_4:
		case Key_5:
			Create(key - Key_1);
			break;

		case Key_D:
			Destroy();
			break;
				
		default:
			break;
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += DRAW_STRING_NEW_LINE;

		const auto L = RealNum(25);
		const auto point1 = Vec2(0.0f, 10.0f);
		const auto d = Vec2(L * std::cos(m_angle), -L * Abs(std::sin(m_angle)));
		const auto point2 = point1 + d;

		EdgeShapesCallback callback;

		m_world->RayCast(&callback, point1, point2);

		if (callback.m_fixture)
		{
			drawer.DrawPoint(callback.m_point, 5.0f, Color(0.4f, 0.9f, 0.4f));

			drawer.DrawSegment(point1, callback.m_point, Color(0.8f, 0.8f, 0.8f));

			const auto head = callback.m_point + 0.5f * callback.m_normal;
			drawer.DrawSegment(callback.m_point, head, Color(0.9f, 0.9f, 0.4f));
		}
		else
		{
			drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
		}

		const auto advanceRay = !settings.pause || settings.singleStep;
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
	std::shared_ptr<CircleShape> m_circle = std::make_shared<CircleShape>(0.5f);

	RealNum m_angle;
};

} // namespace box2d

#endif
