/*
* Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
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

#ifndef BREAKABLE_TEST_H
#define BREAKABLE_TEST_H

namespace box2d {

// This is used to test sensor shapes.
class Breakable : public Test
{
public:

	enum
	{
		e_count = 7
	};

	Breakable()
	{
		// Ground body
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Breakable dynamic body
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 40.0f);
			bd.angle = 0.25f * Pi;
			m_body1 = m_world->CreateBody(bd);

			m_shape1.SetAsBox(0.5f, 0.5f, Vec2(-0.5f, 0.0f), 0.0f);
			m_piece1 = m_body1->CreateFixture(FixtureDef{&m_shape1, 1.0f});

			m_shape2.SetAsBox(0.5f, 0.5f, Vec2(0.5f, 0.0f), 0.0f);
			m_piece2 = m_body1->CreateFixture(FixtureDef{&m_shape2, 1.0f});
		}

		m_break = false;
		m_broke = false;
	}

	void PostSolve(Contact& contact, const ContactImpulse& impulse, ContactListener::iteration_type solved) override
	{
		if (m_broke)
		{
			// The body already broke.
			return;
		}

		// Should the body break?
		auto maxImpulse = float_t(0);
		{
			const auto count = impulse.GetCount();
			for (auto i = decltype(count){0}; i < count; ++i)
			{
				maxImpulse = Max(maxImpulse, impulse.GetEntryNormal(i));
			}
		}

		if (maxImpulse > 40)
		{
			// Flag the body for breaking.
			m_break = true;
		}
	}

	void Break()
	{
		// Create two bodies from one.
		Body* body1 = m_piece1->GetBody();
		Vec2 center = body1->GetWorldCenter();

		body1->DestroyFixture(m_piece2);
		m_piece2 = nullptr;

		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.position = body1->GetPosition();
		bd.angle = body1->GetAngle();

		Body* body2 = m_world->CreateBody(bd);
		m_piece2 = body2->CreateFixture(FixtureDef{&m_shape2, 1.0f});

		// Compute consistent velocities for new bodies based on
		// cached velocity.
		Vec2 center1 = body1->GetWorldCenter();
		Vec2 center2 = body2->GetWorldCenter();
		
		Vec2 velocity1 = m_velocity + GetReversePerpendicular(center1 - center) * m_angularVelocity;
		Vec2 velocity2 = m_velocity + GetReversePerpendicular(center2 - center) * m_angularVelocity;

		body1->SetVelocity(Velocity{velocity1, m_angularVelocity});
		body2->SetVelocity(Velocity{velocity2, m_angularVelocity});
	}

	void Step(Settings* settings) override
	{
		if (m_break)
		{
			Break();
			m_broke = true;
			m_break = false;
		}

		// Cache velocities to improve movement on breakage.
		if (m_broke == false)
		{
			const auto velocity = m_body1->GetVelocity();
			m_velocity = velocity.v;
			m_angularVelocity = velocity.w;
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Breakable;
	}

	Body* m_body1;
	Vec2 m_velocity;
	float_t m_angularVelocity;
	PolygonShape m_shape1;
	PolygonShape m_shape2;
	Fixture* m_piece1;
	Fixture* m_piece2;

	bool m_broke;
	bool m_break;
};

} // namespace box2d

#endif
