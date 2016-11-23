/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef CONVEYOR_BELT_H
#define CONVEYOR_BELT_H

namespace box2d {

class ConveyorBelt : public Test
{
public:

	ConveyorBelt()
	{
		// Ground
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Platform
		{
			BodyDef bd;
			bd.position = Vec2(-5.0f, 5.0f);
			Body* body = m_world->CreateBody(bd);

			const auto shape = PolygonShape(10.0f, 0.5f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.8f;
			m_platform = body->CreateFixture(fd);
		}

		// Boxes
		for (int32 i = 0; i < 5; ++i)
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-10.0f + 2.0f * i, 7.0f);
			Body* body = m_world->CreateBody(bd);

			const auto shape = PolygonShape(0.5f, 0.5f);
			body->CreateFixture(FixtureDef{&shape, 20.0f});
		}
	}

	void PreSolve(Contact& contact, const Manifold& oldManifold) override
	{
		Test::PreSolve(contact, oldManifold);

		Fixture* fixtureA = contact.GetFixtureA();
		Fixture* fixtureB = contact.GetFixtureB();

		if (fixtureA == m_platform)
		{
			contact.SetTangentSpeed(5.0f);
		}

		if (fixtureB == m_platform)
		{
			contact.SetTangentSpeed(-5.0f);
		}
	}

	static Test* Create()
	{
		return new ConveyorBelt;
	}

	Fixture* m_platform;
};

} // namespace box2d

#endif
