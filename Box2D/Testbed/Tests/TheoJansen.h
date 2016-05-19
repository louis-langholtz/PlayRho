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

		b2FixtureDef fd1, fd2;
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0f;
		fd2.density = 1.0f;

		b2PolygonShape poly1, poly2;

		if (s > 0.0f)
		{
			Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p2;
			vertices[2] = p3;
			poly1.Set(vertices, 3);

			vertices[0] = Vec2_zero;
			vertices[1] = p5 - p4;
			vertices[2] = p6 - p4;
			poly2.Set(vertices, 3);
		}
		else
		{
			Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p3;
			vertices[2] = p2;
			poly1.Set(vertices, 3);

			vertices[0] = Vec2_zero;
			vertices[1] = p6 - p4;
			vertices[2] = p5 - p4;
			poly2.Set(vertices, 3);
		}

		fd1.shape = &poly1;
		fd2.shape = &poly2;

		BodyDef bd1, bd2;
		bd1.type = DynamicBody;
		bd2.type = DynamicBody;
		bd1.position = m_offset;
		bd2.position = p4 + m_offset;

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

		Body* body1 = m_world->CreateBody(&bd1);
		Body* body2 = m_world->CreateBody(&bd2);

		body1->CreateFixture(&fd1);
		body2->CreateFixture(&fd2);

		b2DistanceJointDef djd;

		// Using a soft distance constraint can reduce some jitter.
		// It also makes the structure seem a bit more fluid by
		// acting like a suspension system.
		djd.dampingRatio = 0.5f;
		djd.frequencyHz = 10.0f;

		djd.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&djd);

		b2RevoluteJointDef rjd;

		rjd.Initialize(body2, m_chassis, p4 + m_offset);
		m_world->CreateJoint(&rjd);
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
			Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(Vec2(-50.0f, 0.0f), Vec2(50.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(Vec2(-50.0f, 0.0f), Vec2(-50.0f, 10.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(Vec2(50.0f, 0.0f), Vec2(50.0f, 10.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		// Balls
		for (int32 i = 0; i < 40; ++i)
		{
			b2CircleShape shape;
			shape.SetRadius(0.25);

			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = Vec2(-40.0f + 2.0f * i, 0.5f);

			Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 1.0f);
		}

		// Chassis
		{
			b2PolygonShape shape;
			shape.SetAsBox(2.5f, 1.0f);

			b2FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = pivot + m_offset;
			m_chassis = m_world->CreateBody(&bd);
			m_chassis->CreateFixture(&sd);
		}

		{
			b2CircleShape shape;
			shape.SetRadius(float_t(1.6));

			b2FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = pivot + m_offset;
			m_wheel = m_world->CreateBody(&bd);
			m_wheel->CreateFixture(&sd);
		}

		{
			b2RevoluteJointDef jd;
			jd.Initialize(m_wheel, m_chassis, pivot + m_offset);
			jd.collideConnected = false;
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 400.0f;
			jd.enableMotor = m_motorOn;
			m_motorJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		Vec2 wheelAnchor;
		
		wheelAnchor = pivot + Vec2(0.0f, -0.8f);

		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetPosition(), 120.0f * Pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetPosition(), -120.0f * Pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);
	}

	void Step(Settings* settings)
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
		m_textLine += DRAW_STRING_NEW_LINE;

		Test::Step(settings);
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_motorJoint->SetMotorSpeed(-m_motorSpeed);
			break;

		case GLFW_KEY_S:
			m_motorJoint->SetMotorSpeed(0.0f);
			break;

		case GLFW_KEY_D:
			m_motorJoint->SetMotorSpeed(m_motorSpeed);
			break;

		case GLFW_KEY_M:
			m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
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
	b2RevoluteJoint* m_motorJoint;
	bool m_motorOn;
	float_t m_motorSpeed;
};

} // namespace box2d

#endif // THEO_JANSEN_H
