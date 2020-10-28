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

#ifndef PLAYRHO_ONE_SIDED_PLATFORM_HPP
#define PLAYRHO_ONE_SIDED_PLATFORM_HPP

#include "../Framework/Test.hpp"

namespace testbed {

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
        CreateFixture(GetWorld(), CreateBody(GetWorld()), Shape{EdgeShapeConf{Vec2(-20.0f, 0.0f) * 1_m, Vec2(20.0f, 0.0f) * 1_m}});

        // Platform
        {
            BodyConf bd;
            bd.location = Vec2(0.0f, 10.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            m_platform = CreateFixture(GetWorld(), body, Shape{PolygonShapeConf{}.SetAsBox(3_m, 0.5_m)});
            m_bottom = Real(10.0f - 0.5f) * 1_m;
            m_top = Real(10.0f + 0.5f) * 1_m;
        }

        // Actor
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.location = Vec2(0.0f, 12.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            auto conf = DiskShapeConf{};
            conf.vertexRadius = m_radius;
            conf.density = 20_kgpm2;
            m_character = CreateFixture(GetWorld(), body, Shape(conf));
            SetVelocity(GetWorld(), body, Velocity{Vec2(0.0f, -50.0f) * 1_mps, 0_rpm});
        }
    }

    void PreSolve(ContactID contact, const Manifold& oldManifold) override
    {
        Test::PreSolve(contact, oldManifold);

        const auto fixtureA = GetFixtureA(GetWorld(), contact);
        const auto fixtureB = GetFixtureB(GetWorld(), contact);

        if (fixtureA != m_platform && fixtureA != m_character)
        {
            return;
        }

        if (fixtureB != m_platform && fixtureB != m_character)
        {
            return;
        }

#if 1
        const auto position = GetLocation(GetWorld(), GetBody(GetWorld(), m_character));
        if (GetY(position) < m_top + m_radius - GetVertexRadius(GetShape(GetWorld(), m_platform), 0))
        {
            UnsetEnabled(GetWorld(), contact);
        }
#else
        const auto v = m_character->GetBody()->GetLinearVelocity();
        if (v.y > 0.0f)
        {
            contact.UnsetEnabled();
        }
#endif
    }

    void PostStep(const Settings&, Drawer&) override
    {
        const auto v = GetLinearVelocity(GetWorld(), GetBody(GetWorld(), m_character));
        std::stringstream stream;
        stream << "Character linear velocity: ";
        stream << static_cast<double>(Real{GetY(v) / 1_mps});
        stream << " m/s.";
        SetStatus(stream.str());
    }

    Length m_radius = 0.5_m;
    Length m_top;
    Length m_bottom;
    State m_state = e_unknown;
    FixtureID m_platform;
    FixtureID m_character;
};

} // namespace testbed

#endif
