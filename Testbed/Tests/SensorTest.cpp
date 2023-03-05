/*
 * Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "../Framework/Test.hpp"

#include <vector>

namespace testbed {

// This is used to test sensor shapes.
class SensorTest : public Test
{
public:
    static inline const auto registered = RegisterTest("Sensor Test", MakeUniqueTest<SensorTest>);
    enum { e_count = 7 };

    SensorTest()
    {
        SetBeginContactListener(GetWorld(), [this](ContactID id) { BeginContact(id); });
        SetEndContactListener(GetWorld(), [this](ContactID id) { EndContact(id); });

        {
            m_ground = CreateBody(GetWorld());
            Attach(GetWorld(), m_ground,
                   Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});
            auto conf = DiskShapeConf{};
            conf.vertexRadius = 5_m;
            conf.location = Vec2(0.0f, 10.0f) * 1_m;
            conf.isSensor = true;
            m_sensor = CreateShape(GetWorld(), Shape(conf));
            Attach(GetWorld(), m_ground, m_sensor);
        }

        const auto shape =
            CreateShape(GetWorld(), DiskShapeConf{}.UseDensity(1_kgpm2).UseRadius(1_m));
        for (auto i = 0; i < e_count; ++i) {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(-10.0f + 3.0f * i, 20.0f) * 1_m;
            m_bodies[i] = CreateBody(GetWorld(), bd);
            m_touching.resize(m_bodies[i].get() + 1);
            m_touching[m_bodies[i].get()] = false;
            Attach(GetWorld(), m_bodies[i], shape);
        }
    }

    // Implement contact listener.
    void BeginContact(ContactID contactId)
    {
        const auto& contact = GetContact(GetWorld(), contactId);
        if (GetShapeA(contact) == m_sensor) {
            m_touching[GetBodyB(contact).get()] = true;
        }
        if (GetShapeB(contact) == m_sensor) {
            m_touching[GetBodyA(contact).get()] = true;
        }
    }

    // Implement contact listener.
    void EndContact(ContactID contactId)
    {
        const auto& contact = GetContact(GetWorld(), contactId);
        if (GetShapeA(contact) == m_sensor) {
            m_touching[GetBodyB(contact).get()] = false;
        }
        if (GetShapeB(contact) == m_sensor) {
            m_touching[GetBodyA(contact).get()] = false;
        }
    }

    void PostStep(const Settings&, Drawer&) override
    {
        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (auto i = 0; i < e_count; ++i) {
            if (!m_touching[m_bodies[i].get()]) {
                continue;
            }
            const auto body = m_bodies[i];
            const auto circle = TypeCast<DiskShapeConf>(GetShape(GetWorld(), m_sensor));
            const auto center = GetWorldPoint(GetWorld(), m_ground, circle.GetLocation());
            const auto position = GetLocation(GetWorld(), body);
            const auto d = center - position;
            if (AlmostZero(GetMagnitudeSquared(d) / SquareMeter)) {
                continue;
            }
            const auto F = Force2{GetUnitVector(d) * 100_N};
            playrho::d2::ApplyForce(GetWorld(), body, F, position);
        }
    }

    ShapeID m_sensor = InvalidShapeID;
    BodyID m_ground = InvalidBodyID;
    BodyID m_bodies[e_count];
    std::vector<bool> m_touching;
};

} // namespace testbed
