/*
* Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
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
		m_shape1->SetDensity(RealNum{1} * KilogramPerSquareMeter);
		m_shape2->SetDensity(RealNum{1} * KilogramPerSquareMeter);

		// Ground body
		{
			const auto ground = m_world->CreateBody();
			ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f)));
		}

		// Breakable dynamic body
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 40.0f);
			bd.angle = 0.25f * Pi * Radian;
			m_body1 = m_world->CreateBody(bd);

			SetAsBox(*m_shape1, 0.5f, 0.5f, Vec2(-0.5f, 0.0f), 0.0f * Radian);
			m_piece1 = m_body1->CreateFixture(m_shape1);

			SetAsBox(*m_shape2, 0.5f, 0.5f, Vec2(0.5f, 0.0f), 0.0f * Radian);
			m_piece2 = m_body1->CreateFixture(m_shape2);
		}

		m_break = false;
		m_broke = false;
	}

	void PostSolve(Contact&, const ContactImpulsesList& impulse, ContactListener::iteration_type) override
	{
		if (m_broke)
		{
			// The body already broke.
			return;
		}

		// Should the body break?
		auto maxImpulse = RealNum(0);
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
		const auto body1 = m_piece1->GetBody();
		const auto center = body1->GetWorldCenter();

		body1->DestroyFixture(m_piece2);
		m_piece2 = nullptr;

		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.position = body1->GetLocation();
		bd.angle = body1->GetAngle();

		const auto body2 = m_world->CreateBody(bd);
		m_piece2 = body2->CreateFixture(m_shape2);

		// Compute consistent velocities for new bodies based on
		// cached velocity.
		const auto center1 = body1->GetWorldCenter();
		const auto center2 = body2->GetWorldCenter();
		
		const auto velocity1 = m_velocity + GetRevPerpendicular(center1 - center) * RealNum{m_angularVelocity / RadianPerSecond} * MeterPerSecond;
		const auto velocity2 = m_velocity + GetRevPerpendicular(center2 - center) * RealNum{m_angularVelocity / RadianPerSecond} * MeterPerSecond;

		body1->SetVelocity(Velocity{velocity1, m_angularVelocity});
		body2->SetVelocity(Velocity{velocity2, m_angularVelocity});
	}

	void PreStep(const Settings&, Drawer&) override
	{
		if (m_break)
		{
			Break();
			m_broke = true;
			m_break = false;
		}

		// Cache velocities to improve movement on breakage.
		if (!m_broke)
		{
			const auto velocity = m_body1->GetVelocity();
			m_velocity = velocity.linear;
			m_angularVelocity = velocity.angular;
		}
	}

	static Test* Create()
	{
		return new Breakable;
	}

	Body* m_body1;
	Vector2D<LinearVelocity> m_velocity;
	AngularVelocity m_angularVelocity;
	std::shared_ptr<PolygonShape> m_shape1 = std::make_shared<PolygonShape>();
	std::shared_ptr<PolygonShape> m_shape2 = std::make_shared<PolygonShape>();
	Fixture* m_piece1;
	Fixture* m_piece2;

	bool m_broke;
	bool m_break;
};

} // namespace box2d

#endif
