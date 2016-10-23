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
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(bd);

			Vec2 vs[5];
			vs[0] = Vec2(0.0f, -2.0f);
			vs[1] = Vec2(8.0f, 6.0f);
			vs[2] = Vec2(8.0f, 20.0f);
			vs[3] = Vec2(-8.0f, 20.0f);
			vs[4] = Vec2(-8.0f, 6.0f);

			ChainShape loop;
			loop.CreateLoop(Span<const Vec2>(vs, 5));
			FixtureDef fd;
			fd.shape = &loop;
			fd.density = 0.0f;
			ground->CreateFixture(fd);
		}

		// Flippers
		{
			Vec2 p1(-2.0f, 0.0f), p2(2.0f, 0.0f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = p1;
			Body* leftFlipper = m_world->CreateBody(bd);

			bd.position = p2;
			Body* rightFlipper = m_world->CreateBody(bd);

			const auto box = PolygonShape(1.75f, 0.1f);

			FixtureDef fd;
			fd.shape = &box;
			fd.density = 1.0f;

			leftFlipper->CreateFixture(fd);
			rightFlipper->CreateFixture(fd);

			RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.localAnchorB = Vec2_zero;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0f;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0f * Pi / 180.0f;
			jd.upperAngle = 5.0f * Pi / 180.0f;
			m_leftJoint = (RevoluteJoint*)m_world->CreateJoint(jd);

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0f * Pi / 180.0f;
			jd.upperAngle = 30.0f * Pi / 180.0f;
			m_rightJoint = (RevoluteJoint*)m_world->CreateJoint(jd);
		}

		// Circle character
		{
			BodyDef bd;
			bd.position = Vec2(1.0f, 15.0f);
			bd.type = BodyType::Dynamic;
			bd.bullet = true;

			m_ball = m_world->CreateBody(bd);

			CircleShape shape;
			shape.SetRadius(float_t(0.2));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			m_ball->CreateFixture(fd);
		}

		m_button = false;
	}

	void Step(Settings& settings, Drawer& drawer) override
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

		Test::Step(settings, drawer);

		drawer.DrawString(5, m_textLine, "Press 'a' to control the flippers");
		m_textLine += DRAW_STRING_NEW_LINE;

	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = true;
			break;
		}
	}

	void KeyboardUp(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = false;
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
	bool m_button;
};

} // namespace box2d

#endif
