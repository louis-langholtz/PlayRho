/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef MOTOR_JOINT_H
#define MOTOR_JOINT_H

namespace box2d {

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJointTest : public Test
{
public:
	MotorJointTest()
	{
		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f)));

		// Define motorized body
		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.position = Vec2(0.0f, 8.0f);
		const auto body = m_world->CreateBody(bd);

		auto conf = PolygonShape::Conf{};
		conf.friction = 0.6f;
		conf.density = 2.0f;
		body->CreateFixture(std::make_shared<PolygonShape>(2.0f, 0.5f, conf));

		MotorJointDef mjd;
		mjd.Initialize(ground, body);
		mjd.maxForce = 1000.0f;
		mjd.maxTorque = 1000.0f;
		m_joint = (MotorJoint*)m_world->CreateJoint(mjd);
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_S:
			m_go = !m_go;
			break;
		default:
			break;
		}
	}

	void PreStep(const Settings& settings, Drawer& drawer) override
	{
		if (m_go && settings.dt > 0)
		{
			m_time += settings.dt;
		}

		const auto linearOffset = Vec2{6.0f * sinf(2.0f * m_time), 8.0f + 4.0f * sinf(1.0f * m_time)};

		m_joint->SetLinearOffset(linearOffset);
		m_joint->SetAngularOffset(4_rad * m_time);

		drawer.DrawPoint(linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: (s) pause");
		m_textLine += 15;
	}
	
	static Test* Create()
	{
		return new MotorJointTest;
	}

	MotorJoint* m_joint;
	float m_time = 0;
	bool m_go = false;
};

} // namespace box2d

#endif
