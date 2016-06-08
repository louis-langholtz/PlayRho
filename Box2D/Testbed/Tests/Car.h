/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef CAR_H
#define CAR_H

namespace box2d {

// This is a fun demo that shows off the wheel joint
class Car : public Test
{
public:
	Car()
	{		
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 50.0f;

		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(&bd);

			EdgeShape shape;

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f));
			ground->CreateFixture(fd);

			float_t hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			float_t x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
				float_t y2 = hs[i];
				shape.Set(Vec2(x, y1), Vec2(x + dx, y2));
				ground->CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			for (int32 i = 0; i < 10; ++i)
			{
				float_t y2 = hs[i];
				shape.Set(Vec2(x, y1), Vec2(x + dx, y2));
				ground->CreateFixture(fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(fd);

			x += 80.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(fd);

			x += 40.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 10.0f, 5.0f));
			ground->CreateFixture(fd);

			x += 20.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(fd);

			x += 40.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x, 20.0f));
			ground->CreateFixture(fd);
		}

		// Teeter
		{
			BodyDef bd;
			bd.position = Vec2(140.0f, 1.0f);
			bd.type = BodyType::Dynamic;
			Body* body = m_world->CreateBody(&bd);

			PolygonShape box;
			box.SetAsBox(10.0f, 0.25f);
			body->CreateFixture(FixtureDef{&box, 1.0f});

			RevoluteJointDef jd;
			jd.Initialize(ground, body, body->GetPosition());
			jd.lowerAngle = -8.0f * Pi / 180.0f;
			jd.upperAngle = 8.0f * Pi / 180.0f;
			jd.enableLimit = true;
			m_world->CreateJoint(&jd);

			body->ApplyAngularImpulse(100.0f, true);
		}

		// Bridge
		{
			int32 N = 20;
			PolygonShape shape;
			shape.SetAsBox(1.0f, 0.125f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.6f;

			RevoluteJointDef jd;

			Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(161.0f + 2.0f * i, -0.125f);
				Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(fd);

				Vec2 anchor(160.0f + 2.0f * i, -0.125f);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;
			}

			Vec2 anchor(160.0f + 2.0f * N, -0.125f);
			jd.Initialize(prevBody, ground, anchor);
			m_world->CreateJoint(&jd);
		}

		// Boxes
		{
			PolygonShape box;
			box.SetAsBox(0.5f, 0.5f);

			Body* body = nullptr;
			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = Vec2(230.0f, 0.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&box, 0.5f});

			bd.position = Vec2(230.0f, 1.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&box, 0.5f});

			bd.position = Vec2(230.0f, 2.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&box, 0.5f});

			bd.position = Vec2(230.0f, 3.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&box, 0.5f});

			bd.position = Vec2(230.0f, 4.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&box, 0.5f});
		}

		// Car
		{
			PolygonShape chassis;
			Vec2 vertices[8];
			vertices[0] = Vec2(-1.5f, -0.5f);
			vertices[1] = Vec2(1.5f, -0.5f);
			vertices[2] = Vec2(1.5f, 0.0f);
			vertices[3] = Vec2(0.0f, 0.9f);
			vertices[4] = Vec2(-1.15f, 0.9f);
			vertices[5] = Vec2(-1.5f, 0.2f);
			chassis.Set(vertices, 6);

			CircleShape circle;
			circle.SetRadius(float_t(0.4));

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 1.0f);
			m_car = m_world->CreateBody(&bd);
			m_car->CreateFixture(FixtureDef{&chassis, 1.0f});

			FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;

			bd.position = Vec2(-1.0f, 0.35f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(fd);

			bd.position = Vec2(1.0f, 0.4f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(fd);

			WheelJointDef jd;
			Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (WheelJoint*)m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_spring1->SetMotorSpeed(m_speed);
			break;

		case GLFW_KEY_S:
			m_spring1->SetMotorSpeed(0.0f);
			break;

		case GLFW_KEY_D:
			m_spring1->SetMotorSpeed(-m_speed);
			break;

		case GLFW_KEY_Q:
			m_hz = Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;

		case GLFW_KEY_E:
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;
		}
	}

	void Step(Settings* settings)
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += DRAW_STRING_NEW_LINE;

		g_camera.m_center.x = m_car->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Car;
	}

	Body* m_car;
	Body* m_wheel1;
	Body* m_wheel2;

	float_t m_hz;
	float_t m_zeta;
	float_t m_speed;
	WheelJoint* m_spring1;
	WheelJoint* m_spring2;
};

} // namespace box2d

#endif
