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

#ifndef PINBALL_H
#define PINBALL_H

namespace box2d {

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
class Pinball : public Test
{
public:
	Pinball()
	{
		// Ground body
		const auto ground = m_world->CreateBody();
		{
			Vec2 vs[5];
			vs[0] = Vec2(0.0f, -2.0f);
			vs[1] = Vec2(8.0f, 6.0f);
			vs[2] = Vec2(8.0f, 20.0f);
			vs[3] = Vec2(-8.0f, 20.0f);
			vs[4] = Vec2(-8.0f, 6.0f);

			ChainShape loop;
			loop.CreateLoop(Span<const Vec2>(vs, 5));
			ground->CreateFixture(std::make_shared<ChainShape>(loop), FixtureDef{}.UseDensity(0));
		}

		// Flippers
		{
			const auto p1 = Vec2(-2.0f, 0.0f);
			const auto p2 = Vec2(+2.0f, 0.0f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = p1;
			const auto leftFlipper = m_world->CreateBody(bd);

			bd.position = p2;
			const auto rightFlipper = m_world->CreateBody(bd);

			const auto box = std::make_shared<PolygonShape>(1.75f, 0.1f);

			FixtureDef fd;
			fd.density = 1.0f;

			leftFlipper->CreateFixture(box, fd);
			rightFlipper->CreateFixture(box, fd);

			RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.localAnchorB = Vec2_zero;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0f;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0_deg;
			jd.upperAngle = 5.0_deg;
			m_leftJoint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0_deg;
			jd.upperAngle = 30.0_deg;
			m_rightJoint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));
		}

		// Circle character
		{
			BodyDef bd;
			bd.position = Vec2(1.0f, 15.0f);
			bd.type = BodyType::Dynamic;
			bd.bullet = true;

			m_ball = m_world->CreateBody(bd);

			FixtureDef fd;
			fd.density = 1.0f;
			m_ball->CreateFixture(std::make_shared<CircleShape>(0.2f), fd);
		}
	}

	void PreStep(const Settings&, Drawer&) override
	{
		if (m_button)
		{
			m_leftJoint->SetMotorSpeed(20.0f);
			m_rightJoint->SetMotorSpeed(-20.0f);
		}
		else
		{
			m_leftJoint->SetMotorSpeed(-10.0f);
			m_rightJoint->SetMotorSpeed(10.0f);
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press 'a' to control the flippers");
		m_textLine += DRAW_STRING_NEW_LINE;
	}
	
	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_button = true;
			break;
		default:
			break;
		}
	}

	void KeyboardUp(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_button = false;
			break;
		default:
			break;
		}
	}

	static Test* Create()
	{
		return new Pinball;
	}

	RevoluteJoint* m_leftJoint;
	RevoluteJoint* m_rightJoint;
	Body* m_ball;
	bool m_button = false;
};

} // namespace box2d

#endif
