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
			Length2D vs[5];
			vs[0] = Vec2(0.0f, -2.0f) * Meter;
			vs[1] = Vec2(8.0f, 6.0f) * Meter;
			vs[2] = Vec2(8.0f, 20.0f) * Meter;
			vs[3] = Vec2(-8.0f, 20.0f) * Meter;
			vs[4] = Vec2(-8.0f, 6.0f) * Meter;

			ChainShape loop;
			loop.CreateLoop(Span<const Length2D>(vs, 5));
			loop.SetDensity(0);
			ground->CreateFixture(std::make_shared<ChainShape>(loop));
		}

		// Flippers
		{
			const auto p1 = Vec2(-2.0f, 0.0f) * Meter;
			const auto p2 = Vec2(+2.0f, 0.0f) * Meter;

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = p1;
			const auto leftFlipper = m_world->CreateBody(bd);

			bd.position = p2;
			const auto rightFlipper = m_world->CreateBody(bd);

			const auto box = std::make_shared<PolygonShape>(RealNum{1.75f} * Meter, RealNum{0.1f} * Meter);
			box->SetDensity(RealNum{1} * KilogramPerSquareMeter);

			leftFlipper->CreateFixture(box);
			rightFlipper->CreateFixture(box);

			RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.localAnchorB = Vec2_zero * Meter;
			jd.enableMotor = true;
			jd.maxMotorTorque = RealNum{1000.0f} * NewtonMeter;
			jd.enableLimit = true;

			jd.motorSpeed = AngularVelocity{0};
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = RealNum{-30.0f} * Degree;
			jd.upperAngle = RealNum{5.0f} * Degree;
			m_leftJoint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));

			jd.motorSpeed = AngularVelocity{0};
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = RealNum{-5.0f} * Degree;
			jd.upperAngle = RealNum{30.0f} * Degree;
			m_rightJoint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));
		}

		// Circle character
		{
			BodyDef bd;
			bd.position = Vec2(1.0f, 15.0f) * Meter;
			bd.type = BodyType::Dynamic;
			bd.bullet = true;

			m_ball = m_world->CreateBody(bd);

			auto conf = CircleShape::Conf{};
			conf.density = RealNum{1} * KilogramPerSquareMeter;
			conf.vertexRadius = RealNum{0.2f} * Meter;
			m_ball->CreateFixture(std::make_shared<CircleShape>(conf));
		}
	}

	void PreStep(const Settings&, Drawer&) override
	{
		if (m_button)
		{
			m_leftJoint->SetMotorSpeed(RealNum{20.0f} * RadianPerSecond);
			m_rightJoint->SetMotorSpeed(RealNum{-20.0f} * RadianPerSecond);
		}
		else
		{
			m_leftJoint->SetMotorSpeed(RealNum{-10.0f} * RadianPerSecond);
			m_rightJoint->SetMotorSpeed(RealNum{10.0f} * RadianPerSecond);
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press 'a' to control the flippers");
		m_textLine += DRAW_STRING_NEW_LINE;
	}
	
	void KeyboardDown(Key key) override
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
