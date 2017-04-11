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

#ifndef CONTINUOUS_TEST_H
#define CONTINUOUS_TEST_H

namespace box2d {

class ContinuousTest : public Test
{
public:

	ContinuousTest()
	{
		{
			BodyDef bd;
			bd.position = Vec2(0.0f, 0.0f) * Meter;
			Body* body = m_world->CreateBody(bd);

			body->CreateFixture(std::make_shared<EdgeShape>(Vec2(-10.0f, 0.0f) * Meter, Vec2(10.0f, 0.0f) * Meter));

			PolygonShape shape;
			SetAsBox(shape, 0.2f * Meter, 1.0f * Meter, Vec2(0.5f, 1.0f) * Meter, 0.0f * Radian);
			body->CreateFixture(std::make_shared<PolygonShape>(shape));
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 20.0f) * Meter;
			//bd.angle = 0.1f;

			const auto shape = std::make_shared<PolygonShape>(2.0f * Meter, 0.1f * Meter);
			shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);

			m_body = m_world->CreateBody(bd);
			m_body->CreateFixture(shape);

			m_angularVelocity = RadianPerSecond * RandomFloat(-50.0f, 50.0f);
			//m_angularVelocity = 46.661274f;
			m_body->SetVelocity(Velocity{Vec2(0.0f, -100.0f) * MeterPerSecond, m_angularVelocity});
		}
	}

	void Launch()
	{
		uint32 gjkCalls, gjkIters, gjkMaxIters;

		gjkCalls = 0; gjkIters = 0; gjkMaxIters = 0;

		m_body->SetTransform(Vec2(0.0f, 20.0f) * Meter, Angle{0});
		m_angularVelocity = RadianPerSecond * RandomFloat(-50.0f, 50.0f);
		m_body->SetVelocity(Velocity{Vec2(0.0f, -100.0f) * MeterPerSecond, m_angularVelocity});
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		uint32 gjkCalls = 0, gjkIters = 0, gjkMaxIters = 0;

		if (gjkCalls > 0)
		{
			drawer.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				gjkCalls, float(gjkIters) / gjkCalls, gjkMaxIters);
			m_textLine += DRAW_STRING_NEW_LINE;
		}

		int32 toiCalls = 0, toiIters = 0;
		int32 toiRootIters = 0, toiMaxRootIters = 0;

		if (toiCalls > 0)
		{
			drawer.DrawString(5, m_textLine, "toi calls = %d, ave [max] toi iters = %3.1f [%d]",
								toiCalls, float(toiIters) / toiCalls, toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;
			
			drawer.DrawString(5, m_textLine, "ave [max] toi root iters = %3.1f [%d]",
				float(toiRootIters) / toiCalls, toiMaxRootIters);
			m_textLine += DRAW_STRING_NEW_LINE;

			m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (GetStepCount() % 60 == 0)
		{
			//Launch();
		}
	}

	static Test* Create()
	{
		return new ContinuousTest;
	}

	Body* m_body;
	AngularVelocity m_angularVelocity;
};

} // namespace box2d

#endif
