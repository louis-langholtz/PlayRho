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

#ifndef ONE_SIDED_PLATFORM_H
#define ONE_SIDED_PLATFORM_H

namespace box2d {

class OneSidedPlatform : public Test
{
public:

	enum State
	{
		e_unknown,
		e_above,
		e_below
	};

	OneSidedPlatform()
	{
		// Ground
		{
			const auto ground = m_world->CreateBody();
			ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f)));
		}

		// Platform
		{
			BodyDef bd;
			bd.position = Vec2(0.0f, 10.0f);
			const auto body = m_world->CreateBody(bd);
			m_platform = body->CreateFixture(std::make_shared<PolygonShape>(3.0f, 0.5f));
			m_bottom = 10.0f - 0.5f;
			m_top = 10.0f + 0.5f;
		}

		// Actor
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(0.0f, 12.0f);
			const auto body = m_world->CreateBody(bd);
			auto conf = CircleShape::Conf{};
			conf.vertexRadius = m_radius;
			conf.density = RealNum{20} * KilogramPerSquareMeter;
			m_character = body->CreateFixture(std::make_shared<CircleShape>(conf));
			body->SetVelocity(Velocity{Vec2(0.0f, -50.0f), Angle{0}});
		}
	}

	void PreSolve(Contact& contact, const Manifold& oldManifold) override
	{
		Test::PreSolve(contact, oldManifold);

		const auto fixtureA = contact.GetFixtureA();
		const auto fixtureB = contact.GetFixtureB();

		if (fixtureA != m_platform && fixtureA != m_character)
		{
			return;
		}

		if (fixtureB != m_platform && fixtureB != m_character)
		{
			return;
		}

#if 1
		const auto position = m_character->GetBody()->GetLocation();

		if (position.y < m_top + m_radius - m_platform->GetShape()->GetVertexRadius())
		{
			contact.UnsetEnabled();
		}
#else
        const auto v = m_character->GetBody()->GetLinearVelocity();
        if (v.y > 0.0f)
		{
            contact.UnsetEnabled();
        }
#endif
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
		m_textLine += DRAW_STRING_NEW_LINE;

        const auto v = GetLinearVelocity(*(m_character->GetBody()));
        drawer.DrawString(5, m_textLine, "Character Linear Velocity: %f", v.y);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new OneSidedPlatform;
	}

	RealNum m_radius = 0.5f;
	RealNum m_top;
	RealNum m_bottom;
	State m_state = e_unknown;
	Fixture* m_platform;
	Fixture* m_character;
};

} // namespace box2d

#endif
