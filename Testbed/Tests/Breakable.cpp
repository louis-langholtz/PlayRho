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

namespace testbed {

// This is used to test sensor shapes.
class Breakable : public Test
{
public:
    static inline const auto registered = RegisterTest("Breakable", MakeUniqueTest<Breakable>);

    enum { e_count = 7 };

    static PolygonShapeConf GetShapeConf1() noexcept
    {
        return PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m, Length2{-0.5_m, 0_m},
                                                               0_rad);
    }

    static PolygonShapeConf GetShapeConf2() noexcept
    {
        return PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m, Length2{+0.5_m, 0_m},
                                                               0_rad);
    }

    Breakable()
    {
        m_shape1 = CreateShape(GetWorld(), GetShapeConf1());
        m_shape2 = CreateShape(GetWorld(), GetShapeConf2());
        SetPostSolveContactListener(GetWorld(),
                                    [this](ContactID id, const ContactImpulsesList& impulses,
                                           unsigned count) { PostSolve(id, impulses, count); });

        // Ground body
        Attach(GetWorld(), CreateBody(GetWorld()), CreateShape(GetWorld(), GetGroundEdgeConf()));

        // Breakable dynamic body
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.Use(Position{Length2{0_m, 40_m}, Pi * 0.25_rad});
            m_body1 = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), m_body1, m_shape1);
            Attach(GetWorld(), m_body1, m_shape2);
        }
    }

    void PostSolve(ContactID, const ContactImpulsesList& impulse, unsigned)
    {
        if (m_broke) {
            // The body already broke.
            return;
        }

        // Should the body break?
        auto maxImpulse = 0_Ns;
        {
            const auto count = impulse.GetCount();
            for (auto i = decltype(count){0}; i < count; ++i) {
                maxImpulse = std::max(maxImpulse, impulse.GetEntryNormal(i));
            }
        }

        if (maxImpulse > 40_Ns) {
            // Flag the body for breaking.
            m_break = true;
        }
    }

    void Break()
    {
        // Create two bodies from one.
        const auto center = GetWorldCenter(GetWorld(), m_body1);

        Detach(GetWorld(), m_body1, m_shape2);

        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.linearAcceleration = GetGravity();
        bd.UseLocation(GetLocation(GetWorld(), m_body1));
        bd.UseAngle(GetAngle(GetWorld(), m_body1));

        const auto body2 = CreateBody(GetWorld(), bd);
        Attach(GetWorld(), body2, m_shape2);

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        const auto center1 = GetWorldCenter(GetWorld(), m_body1);
        const auto center2 = GetWorldCenter(GetWorld(), body2);

        const auto velocity1 =
            m_velocity + GetRevPerpendicular(center1 - center) * m_angularVelocity / 1_rad;
        const auto velocity2 =
            m_velocity + GetRevPerpendicular(center2 - center) * m_angularVelocity / 1_rad;

        SetVelocity(GetWorld(), m_body1, Velocity{velocity1, m_angularVelocity});
        SetVelocity(GetWorld(), body2, Velocity{velocity2, m_angularVelocity});
    }

    void PreStep(const Settings&, Drawer&) override
    {
        if (m_break) {
            Break();
            m_broke = true;
            m_break = false;
        }

        // Cache velocities to improve movement on breakage.
        if (!m_broke) {
            const auto velocity = GetVelocity(GetWorld(), m_body1);
            m_velocity = velocity.linear;
            m_angularVelocity = velocity.angular;
        }
    }

    BodyID m_body1;
    LinearVelocity2 m_velocity;
    AngularVelocity m_angularVelocity;
    ShapeID m_shape1 = InvalidShapeID;
    ShapeID m_shape2 = InvalidShapeID;
    bool m_broke = false;
    bool m_break = false;
};

} // namespace testbed
