/*
 * Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

    static PolygonShapeConf GetShapeConf1() noexcept
    {
        return PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m, Length2{-0.5_m, 0_m}, 0_rad);
    }
    
    static PolygonShapeConf GetShapeConf2() noexcept
    {
        return PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m, Length2{+0.5_m, 0_m}, 0_rad);
    }
    
    Breakable(): m_shape1{GetShapeConf1()}, m_shape2{GetShapeConf2()}
    {
        // Ground body
        m_world.CreateFixture(m_world.CreateBody(), Shape{GetGroundEdgeConf()});

        // Breakable dynamic body
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = m_gravity;
            bd.location = Length2{0_m, 40_m};
            bd.angle = Pi * 0.25_rad;
            m_body1 = m_world.CreateBody(bd);
            m_piece1 = m_world.CreateFixture(m_body1, m_shape1);
            m_piece2 = m_world.CreateFixture(m_body1, m_shape2);
        }

        m_break = false;
        m_broke = false;
    }

    void PostSolve(ContactID, const ContactImpulsesList& impulse, unsigned) override
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
        const auto body1 = GetBody(m_world, m_piece1);
        const auto center = GetWorldCenter(m_world, body1);

        m_world.Destroy(m_piece2);
        m_piece2 = InvalidFixtureID;

        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.linearAcceleration = m_gravity;
        bd.location = GetLocation(m_world, body1);
        bd.angle = GetAngle(m_world, body1);

        const auto body2 = m_world.CreateBody(bd);
        m_piece2 = m_world.CreateFixture(body2, m_shape2);

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        const auto center1 = GetWorldCenter(m_world, body1);
        const auto center2 = GetWorldCenter(m_world, body2);
        
        const auto velocity1 = m_velocity + GetRevPerpendicular(center1 - center) * m_angularVelocity / 1_rad;
        const auto velocity2 = m_velocity + GetRevPerpendicular(center2 - center) * m_angularVelocity / 1_rad;

        SetVelocity(m_world, body1, Velocity{velocity1, m_angularVelocity});
        SetVelocity(m_world, body2, Velocity{velocity2, m_angularVelocity});
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
            const auto velocity = GetVelocity(m_world, m_body1);
            m_velocity = velocity.linear;
            m_angularVelocity = velocity.angular;
        }
    }

    BodyID m_body1;
    LinearVelocity2 m_velocity;
    AngularVelocity m_angularVelocity;
    Shape m_shape1;
    Shape m_shape2;
    FixtureID m_piece1;
    FixtureID m_piece2;

    bool m_broke;
    bool m_break;
};

} // namespace testbed

#endif
