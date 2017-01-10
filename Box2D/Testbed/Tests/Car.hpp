/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
		m_speed = 50_rad;

		const auto ground = m_world->CreateBody();
		{
			EdgeShape shape;

			FixtureDef fd;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);

			float_t hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			auto x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (auto i = 0; i < 10; ++i)
			{
				const auto y2 = hs[i];
				shape.Set(Vec2(x, y1), Vec2(x + dx, y2));
				ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);
				y1 = y2;
				x += dx;
			}

			for (auto i = 0; i < 10; ++i)
			{
				const auto y2 = hs[i];
				shape.Set(Vec2(x, y1), Vec2(x + dx, y2));
				ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);

			x += 80.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);

			x += 40.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 10.0f, 5.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);

			x += 20.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);

			x += 40.0f;
			shape.Set(Vec2(x, 0.0f), Vec2(x, 20.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape), fd);
		}

		// Teeter
		{
			BodyDef bd;
			bd.position = Vec2(140.0f, 1.0f);
			bd.type = BodyType::Dynamic;
			const auto body = m_world->CreateBody(bd);

			const auto box = std::make_shared<PolygonShape>(10.0f, 0.25f);
			body->CreateFixture(box, FixtureDef{}.UseDensity(1));

			RevoluteJointDef jd(ground, body, body->GetLocation());
			jd.lowerAngle = -8.0_deg;
			jd.upperAngle = 8.0_deg;
			jd.enableLimit = true;
			m_world->CreateJoint(jd);

			ApplyAngularImpulse(*body, 100.0f);
		}

		// Bridge
		{
			const auto N = 20;
			const auto shape = std::make_shared<PolygonShape>(1.0f, 0.125f);

			FixtureDef fd;
			fd.density = 1.0f;
			fd.friction = 0.6f;

			auto prevBody = ground;
			for (auto i = 0; i < N; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(161.0f + 2.0f * i, -0.125f);
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(shape, fd);

				m_world->CreateJoint(RevoluteJointDef{prevBody, body,
					Vec2(160.0f + 2.0f * i, -0.125f)});

				prevBody = body;
			}

			m_world->CreateJoint(RevoluteJointDef{prevBody, ground,
				Vec2(160.0f + 2.0f * N, -0.125f)});
		}

		// Boxes
		{
			const auto box = std::make_shared<PolygonShape>(0.5f, 0.5f);

			auto body = static_cast<Body*>(nullptr);

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = Vec2(230.0f, 0.5f);
			body = m_world->CreateBody(bd);
			body->CreateFixture(box, FixtureDef{}.UseDensity(0.5f));

			bd.position = Vec2(230.0f, 1.5f);
			body = m_world->CreateBody(bd);
			body->CreateFixture(box, FixtureDef{}.UseDensity(0.5f));

			bd.position = Vec2(230.0f, 2.5f);
			body = m_world->CreateBody(bd);
			body->CreateFixture(box, FixtureDef().UseDensity(0.5f));

			bd.position = Vec2(230.0f, 3.5f);
			body = m_world->CreateBody(bd);
			body->CreateFixture(box, FixtureDef().UseDensity(0.5f));

			bd.position = Vec2(230.0f, 4.5f);
			body = m_world->CreateBody(bd);
			body->CreateFixture(box, FixtureDef().UseDensity(0.5f));
		}

		// Car
		{
			auto chassis = std::make_shared<PolygonShape>();
			chassis->Set({
				Vec2(-1.5f, -0.5f),
				Vec2(1.5f, -0.5f),
				Vec2(1.5f, 0.0f),
				Vec2(0.0f, 0.9f),
				Vec2(-1.15f, 0.9f),
				Vec2(-1.5f, 0.2f)
			});

			const auto circle = std::make_shared<CircleShape>(float_t(0.4));

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 1.0f);
			m_car = m_world->CreateBody(bd);
			m_car->CreateFixture(chassis, FixtureDef().UseDensity(1));

			FixtureDef fd;
			fd.density = 1.0f;
			fd.friction = 0.9f;

			bd.position = Vec2(-1.0f, 0.35f);
			m_wheel1 = m_world->CreateBody(bd);
			m_wheel1->CreateFixture(circle, fd);

			bd.position = Vec2(1.0f, 0.4f);
			m_wheel2 = m_world->CreateBody(bd);
			m_wheel2->CreateFixture(circle, fd);

			WheelJointDef jd;
			Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetLocation(), axis);
			jd.motorSpeed = 0_rad;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (WheelJoint*)m_world->CreateJoint(jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetLocation(), axis);
			jd.motorSpeed = 0_rad;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (WheelJoint*)m_world->CreateJoint(jd);
		}
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_A:
			m_spring1->SetMotorSpeed(m_speed);
			break;

		case Key_S:
			m_spring1->SetMotorSpeed(0_rad);
			break;

		case Key_D:
			m_spring1->SetMotorSpeed(-m_speed);
			break;

		case Key_Q:
			m_hz = Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;

		case Key_E:
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;
	
		default:
			break;
		}
	}

	void PreStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.SetTranslation(Vec2{m_car->GetLocation().x, drawer.GetTranslation().y});
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += DRAW_STRING_NEW_LINE;
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
	Angle m_speed;
	WheelJoint* m_spring1;
	WheelJoint* m_spring2;
};

} // namespace box2d

#endif
