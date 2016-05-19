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

#ifndef BODY_TYPES_H
#define BODY_TYPES_H

namespace box2d {

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(&bd);

			EdgeShape shape;
			shape.Set(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f));

			FixtureDef fd;
			fd.shape = &shape;

			ground->CreateFixture(&fd);
		}

		// Define attachment
		{
			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = Vec2(0.0f, 3.0f);
			m_attachment = m_world->CreateBody(&bd);

			PolygonShape shape;
			shape.SetAsBox(0.5f, 2.0f);
			m_attachment->CreateFixture(&shape, 2.0f);
		}

		// Define platform
		{
			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = Vec2(-4.0f, 5.0f);
			m_platform = m_world->CreateBody(&bd);

			PolygonShape shape;
			shape.SetAsBox(0.5f, 4.0f, Vec2(4.0f, 0.0f), 0.5f * Pi);

			FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;
			m_platform->CreateFixture(&fd);

			RevoluteJointDef rjd;
			rjd.Initialize(m_attachment, m_platform, Vec2(0.0f, 5.0f));
			rjd.maxMotorTorque = 50.0f;
			rjd.enableMotor = true;
			m_world->CreateJoint(&rjd);

			b2PrismaticJointDef pjd;
			pjd.Initialize(ground, m_platform, Vec2(0.0f, 5.0f), Vec2(1.0f, 0.0f));

			pjd.maxMotorForce = 1000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0f;
			pjd.upperTranslation = 10.0f;
			pjd.enableLimit = true;

			m_world->CreateJoint(&pjd);

			m_speed = 3.0f;
		}

		// Create a payload
		{
			BodyDef bd;
			bd.type = DynamicBody;
			bd.position = Vec2(0.0f, 8.0f);
			Body* body = m_world->CreateBody(&bd);

			PolygonShape shape;
			shape.SetAsBox(0.75f, 0.75f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;

			body->CreateFixture(&fd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_D:
			m_platform->SetType(DynamicBody);
			break;

		case GLFW_KEY_S:
			m_platform->SetType(StaticBody);
			break;

		case GLFW_KEY_K:
			m_platform->SetType(KinematicBody);
			m_platform->SetLinearVelocity(Vec2(-m_speed, 0.0f));
			m_platform->SetAngularVelocity(0.0f);
			break;
		}
	}

	void Step(Settings* settings)
	{
		// Drive the kinematic body.
		if (m_platform->GetType() == KinematicBody)
		{
			Vec2 p = m_platform->GetTransform().p;
			Vec2 v = m_platform->GetLinearVelocity();

			if ((p.x < -10.0f && v.x < 0.0f) ||
				(p.x > 10.0f && v.x > 0.0f))
			{
				v.x = -v.x;
				m_platform->SetLinearVelocity(v);
			}
		}

		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
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
