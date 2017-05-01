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
            const auto ground = m_world->CreateBody();
            ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));

#if 0
            {
                FixtureDef sd;
                sd.SetAsBox(10.0f * Meter, 2.0f * Meter, Vec2(0.0f, 20.0f) * Meter, 0.0f);
                sd.isSensor = true;
                m_sensor = ground->CreateFixture(sd);
            }
#else
            {
                FixtureDef fd;
                fd.isSensor = true;
                auto conf = CircleShape::Conf{};
                conf.vertexRadius = RealNum{5.0f} * Meter;
                conf.location = Vec2(0.0f, 10.0f) * Meter;
                m_sensor = ground->CreateFixture(std::make_shared<CircleShape>(conf), fd);
            }
#endif
        }

        const auto shape = std::make_shared<CircleShape>(RealNum{1} * Meter);
        shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
        for (auto i = 0; i < e_count; ++i)
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-10.0f + 3.0f * i, 20.0f) * Meter;
            bd.userData = m_touching + i;

            m_touching[i] = false;
            m_bodies[i] = m_world->CreateBody(bd);

            m_bodies[i]->CreateFixture(shape);
        }
    }

    // Implement contact listener.
    void BeginContact(Contact& contact) override
    {
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();

        if (fixtureA == m_sensor)
        {
            const auto userData = fixtureB->GetBody()->GetUserData();
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = true;
            }
        }

        if (fixtureB == m_sensor)
        {
            const auto userData = fixtureA->GetBody()->GetUserData();
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = true;
            }
        }
    }

    // Implement contact listener.
    void EndContact(Contact& contact) override
    {
        const auto fixtureA = contact.GetFixtureA();
        const auto fixtureB = contact.GetFixtureB();

        if (fixtureA == m_sensor)
        {
            const auto userData = fixtureB->GetBody()->GetUserData();
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = false;
            }
        }

        if (fixtureB == m_sensor)
        {
            const auto userData = fixtureA->GetBody()->GetUserData();
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = false;
            }
        }
    }

    void PostStep(const Settings&, Drawer&) override
    {
        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (auto i = 0; i < e_count; ++i)
        {
            if (!m_touching[i])
            {
                continue;
            }

            const auto body = m_bodies[i];
            const auto ground = m_sensor->GetBody();

            const auto circle = static_cast<const CircleShape*>(m_sensor->GetShape());
            const auto center = GetWorldPoint(*ground, circle->GetLocation());

            const auto position = body->GetLocation();

            const auto d = center - position;
            if (almost_zero(GetLengthSquared(d) / SquareMeter))
            {
                continue;
            }

            const auto F = Force2D{RealNum{100.0f} * GetUnitVector(d) * Newton};
            ApplyForce(*body, F, position);
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
