/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef TUMBLER_H
#define TUMBLER_H

namespace box2d {

class Tumbler : public Test
{
public:

	enum
	{
		e_count = 800
	};

	Tumbler()
	{
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(bd);
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.allowSleep = false;
			bd.position = Vec2(0.0f, 10.0f);
			Body* body = m_world->CreateBody(bd);

			PolygonShape shape;
			SetAsBox(shape, 0.5f, 10.0f, Vec2( 10.0f, 0.0f), 0_rad);
			body->CreateFixture(FixtureDef{&shape, 5.0f});
			SetAsBox(shape, 0.5f, 10.0f, Vec2(-10.0f, 0.0f), 0_rad);
			body->CreateFixture(FixtureDef{&shape, 5.0f});
			SetAsBox(shape, 10.0f, 0.5f, Vec2(0.0f, 10.0f), 0_rad);
			body->CreateFixture(FixtureDef{&shape, 5.0f});
			SetAsBox(shape, 10.0f, 0.5f, Vec2(0.0f, -10.0f), 0_rad);
			body->CreateFixture(FixtureDef{&shape, 5.0f});

			RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.bodyB = body;
			jd.localAnchorA = Vec2(0.0f, 10.0f);
			jd.localAnchorB = Vec2(0.0f, 0.0f);
			jd.referenceAngle = 0_rad;
			jd.motorSpeed = 0.05f * Pi;
			jd.maxMotorTorque = 1e8f;
			jd.enableMotor = true;
			m_joint = (RevoluteJoint*)m_world->CreateJoint(jd);
		}

		m_count = 0;
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		if (m_count < e_count)
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 10.0f);
			Body* body = m_world->CreateBody(bd);

			PolygonShape shape;
			shape.SetAsBox(0.125f, 0.125f);
			body->CreateFixture(FixtureDef{&shape, 1.0f});

			++m_count;
		}
	}

	static Test* Create()
	{
		return new Tumbler;
	}

	RevoluteJoint* m_joint;
	int32 m_count;
};

} // namespace box2d

#endif
