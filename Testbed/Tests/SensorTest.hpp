/*
* Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_SENSOR_TEST_HPP
#define PLAYRHO_SENSOR_TEST_HPP

#include "../Framework/Test.hpp"

namespace testbed {

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
            const auto ground = CreateBody(m_world);
            CreateFixture(m_world, ground, Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});

#if 0
            {
                auto sd = FixtureConf{};
                sd.SetAsBox(10_m, 2_m, Vec2(0.0f, 20.0f) * 1_m, 0.0f);
                sd.isSensor = true;
                m_sensor = CreateFixture(m_world, ground, sd);
            }
#else
            {
                auto conf = DiskShapeConf{};
                conf.vertexRadius = 5_m;
                conf.location = Vec2(0.0f, 10.0f) * 1_m;
                m_sensor = CreateFixture(m_world, ground, Shape(conf), FixtureConf{}.UseIsSensor(true));
            }
#endif
        }

        const auto shape = Shape{DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m)};
        for (auto i = 0; i < e_count; ++i)
        {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = m_gravity;
            bd.location = Vec2(-10.0f + 3.0f * i, 20.0f) * 1_m;
            bd.userData = m_touching + i;
            m_touching[i] = false;
            m_bodies[i] = CreateBody(m_world, bd);
            CreateFixture(m_world, m_bodies[i], shape);
        }
    }

    // Implement contact listener.
    void BeginContact(ContactID contact) override
    {
        const auto fixtureA = GetFixtureA(m_world, contact);
        const auto fixtureB = GetFixtureB(m_world, contact);

        if (fixtureA == m_sensor)
        {
            const auto userData = GetUserData(m_world, GetBody(m_world, fixtureB));
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = true;
            }
        }

        if (fixtureB == m_sensor)
        {
            const auto userData = GetUserData(m_world, GetBody(m_world, fixtureA));
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = true;
            }
        }
    }

    // Implement contact listener.
    void EndContact(ContactID contact) override
    {
        const auto fixtureA = GetFixtureA(m_world, contact);
        const auto fixtureB = GetFixtureB(m_world, contact);

        if (fixtureA == m_sensor)
        {
            const auto userData = GetUserData(m_world, GetBody(m_world, fixtureB));
            if (userData)
            {
                bool* touching = (bool*)userData;
                *touching = false;
            }
        }

        if (fixtureB == m_sensor)
        {
            const auto userData = GetUserData(m_world, GetBody(m_world, fixtureA));
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
            const auto ground = GetBody(m_world, m_sensor);
            const auto circle = TypeCast<DiskShapeConf>(GetShape(m_world, m_sensor));
            const auto center = GetWorldPoint(m_world, ground, circle.GetLocation());
            const auto position = GetLocation(m_world, body);
            const auto d = center - position;
            if (AlmostZero(GetMagnitudeSquared(d) / SquareMeter))
            {
                continue;
            }

            const auto F = Force2{GetUnitVector(d) * 100_N};
            playrho::d2::ApplyForce(m_world, body, F, position);
        }
    }

    FixtureID m_sensor;
    BodyID m_bodies[e_count];
    bool m_touching[e_count];
};

} // namespace testbed

#endif
