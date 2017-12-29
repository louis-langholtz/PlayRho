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

#ifndef PLAYRHO_BREAKABLE_TEST_HPP
#define PLAYRHO_BREAKABLE_TEST_HPP

#include "../Framework/Test.hpp"

namespace testbed {

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
        m_world.CreateBody()->CreateFixture(Shape(EdgeShapeConf{}.Set(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m)));

        // Breakable dynamic body
        {
            auto conf = PolygonShapeConf{}.UseDensity(1_kgpm2);

            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(0.0f, 40.0f) * 1_m;
            bd.angle = Pi * 0.25_rad;
            m_body1 = m_world.CreateBody(bd);

            conf.SetAsBox(0.5_m, 0.5_m, Vec2(-0.5f, 0.0f) * 1_m, 0_rad);
            m_shape1 = conf;
            m_piece1 = m_body1->CreateFixture(m_shape1);

            conf.SetAsBox(0.5_m, 0.5_m, Vec2(0.5f, 0.0f) * 1_m, 0_rad);
            m_shape2 = conf;
            m_piece2 = m_body1->CreateFixture(m_shape2);
        }

        m_break = false;
        m_broke = false;
    }

    void PostSolve(Contact&, const ContactImpulsesList& impulse,
                   ContactListener::iteration_type) override
    {
        if (m_broke)
        {
            // The body already broke.
            return;
        }

        // Should the body break?
        auto maxImpulse = 0_Ns;
        {
            const auto count = impulse.GetCount();
            for (auto i = decltype(count){0}; i < count; ++i)
            {
                maxImpulse = std::max(maxImpulse, impulse.GetEntryNormal(i));
            }
        }

        if (maxImpulse > 40_Ns)
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

        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.location = body1->GetLocation();
        bd.angle = body1->GetAngle();

        const auto body2 = m_world.CreateBody(bd);
        m_piece2 = body2->CreateFixture(m_shape2);

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        const auto center1 = body1->GetWorldCenter();
        const auto center2 = body2->GetWorldCenter();
        
        const auto velocity1 = m_velocity + GetRevPerpendicular(center1 - center) * m_angularVelocity / 1_rad;
        const auto velocity2 = m_velocity + GetRevPerpendicular(center2 - center) * m_angularVelocity / 1_rad;

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

    Body* m_body1;
    LinearVelocity2 m_velocity;
    AngularVelocity m_angularVelocity;
    PolygonShapeConf m_shape1;
    PolygonShapeConf m_shape2;
    Fixture* m_piece1;
    Fixture* m_piece2;

    bool m_broke;
    bool m_break;
};

} // namespace testbed

#endif
