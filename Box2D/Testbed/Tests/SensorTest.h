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

#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

namespace box2d {

// This is used to test sensor shapes.
class SensorTest : public Test
{
public:

	enum
	{
		e_count = 7
	};

	SensorTest()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(&bd);

			{
				EdgeShape shape;
				shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
				ground->CreateFixture(FixtureDef{&shape, 0.0f});
			}

#if 0
			{
				FixtureDef sd;
				sd.SetAsBox(10.0f, 2.0f, Vec2(0.0f, 20.0f), 0.0f);
				sd.isSensor = true;
				m_sensor = ground->CreateFixture(sd);
			}
#else
			{
				CircleShape shape(5.0f, Vec2(0.0f, 10.0f));

				FixtureDef fd;
				fd.shape = &shape;
				fd.isSensor = true;
				m_sensor = ground->CreateFixture(fd);
			}
#endif
		}

		{
			CircleShape shape;
			shape.SetRadius(1.0);

			for (int32 i = 0; i < e_count; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-10.0f + 3.0f * i, 20.0f);
				bd.userData = m_touching + i;

				m_touching[i] = false;
				m_bodies[i] = m_world->CreateBody(&bd);

				m_bodies[i]->CreateFixture(FixtureDef{&shape, 1.0f});
			}
		}
	}

	// Implement contact listener.
	void BeginContact(Contact* contact)
	{
		Fixture* fixtureA = contact->GetFixtureA();
		Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}
	}

	// Implement contact listener.
	void EndContact(Contact* contact)
	{
		Fixture* fixtureA = contact->GetFixtureA();
		Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int32 i = 0; i < e_count; ++i)
		{
			if (m_touching[i] == false)
			{
				continue;
			}

			Body* body = m_bodies[i];
			Body* ground = m_sensor->GetBody();

			CircleShape* circle = (CircleShape*)m_sensor->GetShape();
			Vec2 center = ground->GetWorldPoint(circle->GetPosition());

			Vec2 position = body->GetPosition();

			Vec2 d = center - position;
			if (LengthSquared(d) < Square(FLT_EPSILON))
			{
				continue;
			}

			d.Normalize();
			Vec2 F = 100.0f * d;
			body->ApplyForce(F, position, false);
		}
	}

	static Test* Create()
	{
		return new SensorTest;
	}

	Fixture* m_sensor;
	Body* m_bodies[e_count];
	bool m_touching[e_count];
};

} // namespace box2d

#endif
