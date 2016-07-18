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

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

namespace box2d {

class ApplyForce : public Test
{
public:
	ApplyForce()
	{
		m_world->SetGravity(Vec2(float_t{0}, float_t{0}));

		const float_t k_restitution = 0.4f;

		Body* ground;
		{
			BodyDef bd;
			bd.position = Vec2(float_t{0}, 20.0f);
			ground = m_world->CreateBody(bd);

			EdgeShape shape;

			FixtureDef sd;
			sd.shape = &shape;
			sd.density = float_t{0};
			sd.restitution = k_restitution;

			// Left vertical
			shape.Set(Vec2(-20.0f, -20.0f), Vec2(-20.0f, 20.0f));
			ground->CreateFixture(sd);

			// Right vertical
			shape.Set(Vec2(20.0f, -20.0f), Vec2(20.0f, 20.0f));
			ground->CreateFixture(sd);

			// Top horizontal
			shape.Set(Vec2(-20.0f, 20.0f), Vec2(20.0f, 20.0f));
			ground->CreateFixture(sd);

			// Bottom horizontal
			shape.Set(Vec2(-20.0f, -20.0f), Vec2(20.0f, -20.0f));
			ground->CreateFixture(sd);
		}

		{
			Transformation xf1;
			xf1.q = Rot(0.3524f * Pi);
			xf1.p = GetXAxis(xf1.q);

			Vec2 vertices[3];
			vertices[0] = Transform(Vec2(-1.0f, float_t{0}), xf1);
			vertices[1] = Transform(Vec2(1.0f, float_t{0}), xf1);
			vertices[2] = Transform(Vec2(float_t{0}, 0.5f), xf1);
			
			PolygonShape poly1;
			poly1.Set(vertices, 3);

			FixtureDef sd1;
			sd1.shape = &poly1;
			sd1.density = 4.0f;

			Transformation xf2;
			xf2.q = Rot(-0.3524f * Pi);
			xf2.p = -GetXAxis(xf2.q);

			vertices[0] = Transform(Vec2(-1.0f, float_t{0}), xf2);
			vertices[1] = Transform(Vec2(1.0f, float_t{0}), xf2);
			vertices[2] = Transform(Vec2(float_t{0}, 0.5f), xf2);
			
			PolygonShape poly2;
			poly2.Set(vertices, 3);

			FixtureDef sd2;
			sd2.shape = &poly2;
			sd2.density = 2.0f;

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.angularDamping = 2.0f;
			bd.linearDamping = 0.5f;

			bd.position = Vec2(float_t{0}, 2.0);
			bd.angle = Pi;
			bd.allowSleep = false;
			m_body = m_world->CreateBody(bd);
			m_body->CreateFixture(sd1);
			m_body->CreateFixture(sd2);
		}

		{
			PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			for (int i = 0; i < 10; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;

				bd.position = Vec2(float_t{0}, 5.0f + 1.54f * i);
				Body* body = m_world->CreateBody(bd);

				body->CreateFixture(fd);

				float_t gravity = 10.0f;
				float_t I = body->GetInertia();
				float_t mass = body->GetMass();

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				float_t radius = Sqrt(2.0f * I / mass);

				FrictionJointDef jd;
				jd.localAnchorA = Vec2_zero;
				jd.localAnchorB = Vec2_zero;
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				m_world->CreateJoint(jd);
			}
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_W:
			{
				Vec2 f = m_body->GetWorldVector(Vec2(float_t{0}, -200.0f));
				Vec2 p = m_body->GetWorldPoint(Vec2(float_t{0}, 2.0f));
				m_body->ApplyForce(f, p, true);
			}
			break;

		case GLFW_KEY_A:
			{
				m_body->ApplyTorque(50.0f, true);
			}
			break;

		case GLFW_KEY_D:
			{
				m_body->ApplyTorque(-50.0f, true);
			}
			break;
		}
	}

	static Test* Create()
	{
		return new ApplyForce;
	}

	Body* m_body;
};

} // namespace box2d

#endif
