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

#ifndef BODY_TYPES_H
#define BODY_TYPES_H

namespace box2d {

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f)));

		// Define attachment
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 3.0f);
			m_attachment = m_world->CreateBody(bd);

			m_attachment->CreateFixture(std::make_shared<PolygonShape>(0.5f, 2.0f), FixtureDef{}.UseDensity(2));
		}

		// Define platform
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-4.0f, 5.0f);
			m_platform = m_world->CreateBody(bd);

			PolygonShape shape;
			SetAsBox(shape, 0.5f, 4.0f, Vec2(4.0f, 0.0f), 0.5_rad * Pi);

			FixtureDef fd{};
			fd.friction = 0.6f;
			fd.density = 2.0f;
			m_platform->CreateFixture(std::make_shared<PolygonShape>(shape), fd);

			RevoluteJointDef rjd(m_attachment, m_platform, Vec2(0.0f, 5.0f));
			rjd.maxMotorTorque = 50.0f;
			rjd.enableMotor = true;
			m_world->CreateJoint(rjd);

			PrismaticJointDef pjd(ground, m_platform, Vec2(0.0f, 5.0f), Vec2(1.0f, 0.0f));
			pjd.maxMotorForce = 1000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0f;
			pjd.upperTranslation = 10.0f;
			pjd.enableLimit = true;
			m_world->CreateJoint(pjd);

			m_speed = 3.0f;
		}

		// Create a payload
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 8.0f);
			Body* body = m_world->CreateBody(bd);

			FixtureDef fd{};
			fd.friction = 0.6f;
			fd.density = 2.0f;

			body->CreateFixture(std::make_shared<PolygonShape>(0.75f, 0.75f), fd);
		}
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_D:
			m_platform->SetType(BodyType::Dynamic);
			break;

		case Key_S:
			m_platform->SetType(BodyType::Static);
			break;

		case Key_K:
			m_platform->SetType(BodyType::Kinematic);
			m_platform->SetVelocity(Velocity{Vec2(-m_speed, 0.0f), 0_rad});
			break;
	
		default:
			break;
		}
	}

	void PreStep(const Settings& settings, Drawer& drawer) override
	{
		// Drive the kinematic body.
		if (m_platform->GetType() == BodyType::Kinematic)
		{
			const auto p = m_platform->GetLocation();
			const auto velocity = m_platform->GetVelocity();

			if ((p.x < -10.0f && velocity.linear.x < 0.0f) ||
				(p.x > 10.0f && velocity.linear.x > 0.0f))
			{
				m_platform->SetVelocity(Velocity{Vec2{-velocity.linear.x, velocity.linear.y}, velocity.angular});
			}
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
		m_textLine += DRAW_STRING_NEW_LINE;
	}
	
	static Test* Create()
	{
		return new BodyTypes;
	}

	Body* m_attachment;
	Body* m_platform;
	float_t m_speed;
};

} // namespace box2d

#endif
