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

namespace playrho {

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
            ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter));
        }

        // Platform
        {
            BodyDef bd;
            bd.location = Vec2(0.0f, 10.0f) * Meter;
            const auto body = m_world->CreateBody(bd);
            m_platform = body->CreateFixture(std::make_shared<PolygonShape>(Real{3.0f} * Meter, Real{0.5f} * Meter));
            m_bottom = Real(10.0f - 0.5f) * Meter;
            m_top = Real(10.0f + 0.5f) * Meter;
        }

        // Actor
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(0.0f, 12.0f) * Meter;
            const auto body = m_world->CreateBody(bd);
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = m_radius;
            conf.density = Real{20} * KilogramPerSquareMeter;
            m_character = body->CreateFixture(std::make_shared<DiskShape>(conf));
            body->SetVelocity(Velocity{Vec2(0.0f, -50.0f) * MeterPerSecond, AngularVelocity{0}});
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

        if (GetY(position) < m_top + m_radius - m_platform->GetShape()->GetVertexRadius())
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
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Press: (c) create a shape, (d) destroy a shape.");
        m_textLine += DRAW_STRING_NEW_LINE;

        const auto v = GetLinearVelocity(*(m_character->GetBody()));
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Character Linear Velocity: %f",
                          static_cast<double>(Real{GetY(v) / MeterPerSecond}));
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    Length m_radius = Real{0.5f} * Meter;
    Length m_top;
    Length m_bottom;
    State m_state = e_unknown;
    Fixture* m_platform;
    Fixture* m_character;
};

} // namespace playrho

#endif
