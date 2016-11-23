/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

// Inspired by a contribution by roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

#ifndef THEO_JANSEN_H
#define THEO_JANSEN_H

namespace box2d {

class TheoJansen : public Test
{
public:

	void CreateLeg(float_t s, const Vec2& wheelAnchor)
	{
		Vec2 p1(5.4f * s, -6.1f);
		Vec2 p2(7.2f * s, -1.2f);
		Vec2 p3(4.3f * s, -1.9f);
		Vec2 p4(3.1f * s, 0.8f);
		Vec2 p5(6.0f * s, 1.5f);
		Vec2 p6(2.5f * s, 3.7f);

		FixtureDef fd1, fd2;
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0f;
		fd2.density = 1.0f;

		PolygonShape poly1, poly2;
		if (s > 0.0f)
		{
			poly1.Set({p1, p2, p3});
			poly2.Set({Vec2_zero, p5 - p4, p6 - p4});
		}
		else
		{
			poly1.Set({p1, p3, p2});
			poly2.Set({Vec2_zero, p6 - p4, p5 - p4});
		}

		fd1.shape = &poly1;
		fd2.shape = &poly2;

		BodyDef bd1, bd2;
		bd1.type = BodyType::Dynamic;
		bd2.type = BodyType::Dynamic;
		bd1.position = m_offset;
		bd2.position = p4 + m_offset;

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

		Body* body1 = m_world->CreateBody(bd1);
		Body* body2 = m_world->CreateBody(bd2);

		body1->CreateFixture(fd1);
		body2->CreateFixture(fd2);

		// Using a soft distance constraint can reduce some jitter.
		// It also makes the structure seem a bit more fluid by
		// acting like a suspension system.
		m_world->CreateJoint(DistanceJointDef{body1, body2, p2 + m_offset, p5 + m_offset, float_t(10), float_t(0.5)});
		m_world->CreateJoint(DistanceJointDef{body1, body2, p3 + m_offset, p4 + m_offset, float_t(10), float_t(0.5)});
		m_world->CreateJoint(DistanceJointDef(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset, float_t(10), float_t(0.5)));
		m_world->CreateJoint(DistanceJointDef(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset, float_t(10), float_t(0.5)));
		m_world->CreateJoint(RevoluteJointDef{body2, m_chassis, p4 + m_offset});
	}

	TheoJansen()
	{
		m_offset = Vec2(0.0f, 8.0f);
		m_motorSpeed = 2.0f;
		m_motorOn = true;
		Vec2 pivot(0.0f, 0.8f);

		// Ground
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-50.0f, 0.0f), Vec2(50.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(Vec2(-50.0f, 0.0f), Vec2(-50.0f, 10.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(Vec2(50.0f, 0.0f), Vec2(50.0f, 10.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Balls
		for (int32 i = 0; i < 40; ++i)
		{
			CircleShape shape;
			shape.SetRadius(0.25);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-40.0f + 2.0f * i, 0.5f);

			Body* body = m_world->CreateBody(bd);
			body->CreateFixture(FixtureDef{&shape, 1.0f});
		}

		// Chassis
		{
			const auto shape = PolygonShape(2.5f, 1.0f);

			FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = pivot + m_offset;
			m_chassis = m_world->CreateBody(bd);
			m_chassis->CreateFixture(sd);
		}

		{
			CircleShape shape;
			shape.SetRadius(float_t(1.6));

			FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = pivot + m_offset;
			m_wheel = m_world->CreateBody(bd);
			m_wheel->CreateFixture(sd);
		}

		{
			RevoluteJointDef jd{m_wheel, m_chassis, pivot + m_offset};
			jd.collideConnected = false;
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 400.0f;
			jd.enableMotor = m_motorOn;
			m_motorJoint = (RevoluteJoint*)m_world->CreateJoint(jd);
		}

		Vec2 wheelAnchor;
		
		wheelAnchor = pivot + Vec2(0.0f, -0.8f);

		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetLocation(), 120_deg);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetLocation(), -120_deg);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_motorJoint->SetMotorSpeed(-m_motorSpeed);
			break;

		case Key_S:
			m_motorJoint->SetMotorSpeed(0.0f);
			break;

		case Key_D:
			m_motorJoint->SetMotorSpeed(m_motorSpeed);
			break;

		case Key_M:
			m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
			break;

		default:
			break;				
		}
	}

	static Test* Create()
	{
		return new TheoJansen;
	}

	Vec2 m_offset;
	Body* m_chassis;
	Body* m_wheel;
	RevoluteJoint* m_motorJoint;
	bool m_motorOn;
	float_t m_motorSpeed;
};

} // namespace box2d

#endif // THEO_JANSEN_H
