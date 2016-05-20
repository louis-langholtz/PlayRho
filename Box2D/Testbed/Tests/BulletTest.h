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
			Body* body = m_world->CreateBody(&bd);

			EdgeShape edge;

			edge.Set(Vec2(-10.0f, 0.0f), Vec2(10.0f, 0.0f));
			body->CreateFixture(&edge, 0.0f);

			PolygonShape shape;
			shape.SetAsBox(0.2f, 1.0f, Vec2(0.5f, 1.0f), 0.0f);
			body->CreateFixture(&shape, 0.0f);
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 4.0f);

			PolygonShape box;
			box.SetAsBox(2.0f, 0.1f);

			m_body = m_world->CreateBody(&bd);
			m_body->CreateFixture(&box, 1.0f);

			box.SetAsBox(0.25f, 0.25f);

			//m_x = RandomFloat(-1.0f, 1.0f);
			m_x = 0.20352793f;
			bd.position = Vec2(m_x, 10.0f);
			bd.bullet = true;

			m_bullet = m_world->CreateBody(&bd);
			m_bullet->CreateFixture(&box, 100.0f);

			m_bullet->SetLinearVelocity(Vec2(0.0f, -50.0f));
		}
	}

	void Launch()
	{
		m_body->SetTransform(Vec2(0.0f, 4.0f), 0.0f);
		m_body->SetLinearVelocity(Vec2_zero);
		m_body->SetAngularVelocity(0.0f);

		m_x = RandomFloat(-1.0f, 1.0f);
		m_bullet->SetTransform(Vec2(m_x, 10.0f), 0.0f);
		m_bullet->SetLinearVelocity(Vec2(0.0f, -50.0f));
		m_bullet->SetAngularVelocity(0.0f);

		extern int32 gjkCalls, gjkIters, gjkMaxIters;
		extern int32 toiCalls, toiIters, toiMaxIters;
		extern int32 toiRootIters, toiMaxRootIters;

		gjkCalls = 0;
		gjkIters = 0;
		gjkMaxIters = 0;

		toiCalls = 0;
		toiIters = 0;
		toiMaxIters = 0;
		toiRootIters = 0;
		toiMaxRootIters = 0;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		extern int32 gjkCalls, gjkIters, gjkMaxIters;
		extern int32 toiCalls, toiIters;
		extern int32 toiRootIters, toiMaxRootIters;

		if (gjkCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				gjkCalls, gjkIters / float_t(gjkCalls), gjkMaxIters);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (toiCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
				toiCalls, toiIters / float_t(toiCalls), toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;

			g_debugDraw.DrawString(5, m_textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
				toiRootIters / float_t(toiCalls), toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (m_stepCount % 60 == 0)
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
	float_t m_x;
};

} // namespace box2d

#endif
