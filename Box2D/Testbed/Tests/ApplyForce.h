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

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

namespace box2d {

class ApplyForce : public Test
{
public:
	ApplyForce()
	{
		m_world->SetGravity(b2Vec2(float_t{0}, float_t{0}));

		const float_t k_restitution = 0.4f;

		b2Body* ground;
		{
			BodyDef bd;
			bd.position = b2Vec2(float_t{0}, 20.0f);
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef sd;
			sd.shape = &shape;
			sd.density = float_t{0};
			sd.restitution = k_restitution;

			// Left vertical
			shape.Set(b2Vec2(-20.0f, -20.0f), b2Vec2(-20.0f, 20.0f));
			ground->CreateFixture(&sd);

			// Right vertical
			shape.Set(b2Vec2(20.0f, -20.0f), b2Vec2(20.0f, 20.0f));
			ground->CreateFixture(&sd);

			// Top horizontal
			shape.Set(b2Vec2(-20.0f, 20.0f), b2Vec2(20.0f, 20.0f));
			ground->CreateFixture(&sd);

			// Bottom horizontal
			shape.Set(b2Vec2(-20.0f, -20.0f), b2Vec2(20.0f, -20.0f));
			ground->CreateFixture(&sd);
		}

		{
			b2Transform xf1;
			xf1.q = b2Rot(0.3524f * Pi);
			xf1.p = xf1.q.GetXAxis();

			b2Vec2 vertices[3];
			vertices[0] = b2Mul(xf1, b2Vec2(-1.0f, float_t{0}));
			vertices[1] = b2Mul(xf1, b2Vec2(1.0f, float_t{0}));
			vertices[2] = b2Mul(xf1, b2Vec2(float_t{0}, 0.5f));
			
			b2PolygonShape poly1;
			poly1.Set(vertices, 3);

			b2FixtureDef sd1;
			sd1.shape = &poly1;
			sd1.density = 4.0f;

			b2Transform xf2;
			xf2.q = b2Rot(-0.3524f * Pi);
			xf2.p = -xf2.q.GetXAxis();

			vertices[0] = b2Mul(xf2, b2Vec2(-1.0f, float_t{0}));
			vertices[1] = b2Mul(xf2, b2Vec2(1.0f, float_t{0}));
			vertices[2] = b2Mul(xf2, b2Vec2(float_t{0}, 0.5f));
			
			b2PolygonShape poly2;
			poly2.Set(vertices, 3);

			b2FixtureDef sd2;
			sd2.shape = &poly2;
			sd2.density = 2.0f;

			BodyDef bd;
			bd.type = DynamicBody;
			bd.angularDamping = 2.0f;
			bd.linearDamping = 0.5f;

			bd.position = b2Vec2(float_t{0}, 2.0);
			bd.angle = Pi;
			bd.allowSleep = false;
			m_body = m_world->CreateBody(&bd);
			m_body->CreateFixture(&sd1);
			m_body->CreateFixture(&sd2);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			for (int i = 0; i < 10; ++i)
			{
				BodyDef bd;
				bd.type = DynamicBody;

				bd.position = b2Vec2(float_t{0}, 5.0f + 1.54f * i);
				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&fd);

				float_t gravity = 10.0f;
				float_t I = body->GetInertia();
				float_t mass = body->GetMass();

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				float_t radius = b2Sqrt(2.0f * I / mass);

				b2FrictionJointDef jd;
				jd.localAnchorA = b2Vec2_zero;
				jd.localAnchorB = b2Vec2_zero;
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				m_world->CreateJoint(&jd);
			}
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_W:
			{
				b2Vec2 f = m_body->GetWorldVector(b2Vec2(float_t{0}, -200.0f));
				b2Vec2 p = m_body->GetWorldPoint(b2Vec2(float_t{0}, 2.0f));
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

	b2Body* m_body;
};

} // namespace box2d

#endif
