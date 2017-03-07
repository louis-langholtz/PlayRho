/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef BULLET_TEST_H
#define BULLET_TEST_H

namespace box2d {

class BulletTest : public Test
{
public:

	BulletTest()
	{
		{
			BodyDef bd;
			bd.position = Vec2(0.0f, 0.0f);
			Body* body = m_world->CreateBody(bd);

			body->CreateFixture(std::make_shared<EdgeShape>(Vec2(-10.0f, 0.0f), Vec2(10.0f, 0.0f)));

			PolygonShape shape;
			SetAsBox(shape, 0.2f, 1.0f, Vec2(0.5f, 1.0f), 0.0_rad);
			body->CreateFixture(std::make_shared<PolygonShape>(shape));
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 4.0f);

			PolygonShape box;
			box.SetAsBox(2.0f, 0.1f);

			m_body = m_world->CreateBody(bd);
			m_body->CreateFixture(std::make_shared<PolygonShape>(box), FixtureDef{}.UseDensity(1));

			box.SetAsBox(0.25f, 0.25f);

			//m_x = RandomFloat(-1.0f, 1.0f);
			m_x = 0.20352793f;
			bd.position = Vec2(m_x, 10.0f);
			bd.bullet = true;

			m_bullet = m_world->CreateBody(bd);
			m_bullet->CreateFixture(std::make_shared<PolygonShape>(box), FixtureDef{}.UseDensity(100));

			m_bullet->SetVelocity(Velocity{Vec2{0.0f, -50.0f}, 0_rad});
		}
	}

	void Launch()
	{
		m_body->SetTransform(Vec2(0.0f, 4.0f), 0.0_rad);
		m_body->SetVelocity(Velocity{Vec2_zero, 0_rad});

		m_x = RandomFloat(-1.0f, 1.0f);
		m_bullet->SetTransform(Vec2(m_x, 10.0f), 0.0_rad);
		m_bullet->SetVelocity(Velocity{Vec2(0.0f, -50.0f), 0_rad});

		uint32 gjkCalls, gjkIters, gjkMaxIters;
		std::remove_const<decltype(DefaultMaxToiIters)>::type toiMaxIters;
		int32 toiRootIters, toiMaxRootIters;

		gjkCalls = 0;
		gjkIters = 0;
		gjkMaxIters = 0;

		toiMaxIters = 0;
		toiRootIters = 0;
		toiMaxRootIters = 0;
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		uint32 gjkCalls = 0, gjkIters = 0, gjkMaxIters = 0;
		int32 toiRootIters = 0, toiMaxRootIters = 0;

		if (gjkCalls > 0)
		{
			drawer.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				gjkCalls, float(gjkIters) / gjkCalls, gjkMaxIters);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		unsigned toiCalls = 0;
		unsigned toiIters = 0;
#if 0
		for (auto&& c: m_world->GetContacts())
		{
			c.GetToiCount();
		}
#endif
		if (toiCalls > 0)
		{
			drawer.DrawString(5, m_textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
				toiCalls, float(toiIters) / toiCalls, toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;

			drawer.DrawString(5, m_textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
				float(toiRootIters) / toiCalls, toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (GetStepCount() % 60 == 0)
		{
			Launch();
		}
	}

	static Test* Create()
	{
		return new BulletTest;
	}

	Body* m_body;
	Body* m_bullet;
	RealNum m_x;
};

} // namespace box2d

#endif
