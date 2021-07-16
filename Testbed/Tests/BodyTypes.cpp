/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_BODY_TYPES_HPP
#define PLAYRHO_BODY_TYPES_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class BodyTypes : public Test
{
public:
    static inline const auto registered = RegisterTest("Body Types", MakeUniqueTest<BodyTypes>);

    BodyTypes()
    {
        const auto ground = CreateBody(GetWorld());
        Attach(GetWorld(), ground,
               CreateShape(GetWorld(), EdgeShapeConf{}.Set(Vec2(-20, 0) * 1_m, Vec2(20, 0) * 1_m)));

        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Dynamic",
                       [&](KeyActionMods) { SetType(GetWorld(), m_platform, BodyType::Dynamic); });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Static",
                       [&](KeyActionMods) { SetType(GetWorld(), m_platform, BodyType::Static); });
        RegisterForKey(GLFW_KEY_K, GLFW_PRESS, 0, "Kinematic", [&](KeyActionMods) {
            SetType(GetWorld(), m_platform, BodyType::Kinematic);
            SetVelocity(GetWorld(), m_platform, Velocity{Vec2(-m_speed, 0) * 1_mps, 0_rpm});
        });

        // Define attachment
        {
            const auto bd = BodyConf{}
                                .UseType(BodyType::Dynamic)
                                .UseLocation(Vec2(0, 3) * 1_m)
                                .UseLinearAcceleration(GetGravity());
            m_attachment = CreateBody(GetWorld(), bd);
            const auto conf = PolygonShapeConf{}.UseDensity(2_kgpm2).SetAsBox(0.5_m, 2_m);
            Attach(GetWorld(), m_attachment, CreateShape(GetWorld(), conf));
        }

        // Define platform
        {
            const auto bd = BodyConf{}
                                .UseType(BodyType::Dynamic)
                                .UseLocation(Vec2(-4, 5) * 1_m)
                                .UseLinearAcceleration(GetGravity());
            m_platform = CreateBody(GetWorld(), bd);

            const auto conf =
                PolygonShapeConf{}.UseFriction(Real(0.6)).UseDensity(2_kgpm2).SetAsBox(
                    0.5_m, 4_m, Vec2(4, 0) * 1_m, Pi * 0.5_rad);
            Attach(GetWorld(), m_platform, CreateShape(GetWorld(), conf));

            auto rjd = GetRevoluteJointConf(GetWorld(), m_attachment, m_platform, Vec2(0, 5) * 1_m);
            rjd.maxMotorTorque = 50_Nm;
            rjd.enableMotor = true;
            CreateJoint(GetWorld(), rjd);

            PrismaticJointConf pjd = GetPrismaticJointConf(GetWorld(), ground, m_platform,
                                                           Vec2(0, 5) * 1_m, UnitVec::GetRight());
            pjd.maxMotorForce = 1000_N;
            pjd.enableMotor = true;
            pjd.lowerTranslation = -10_m;
            pjd.upperTranslation = 10_m;
            pjd.enableLimit = true;
            CreateJoint(GetWorld(), pjd);

            m_speed = 3.0f;
        }

        // Create a payload
        {
            const auto bd = BodyConf{}
                                .UseType(BodyType::Dynamic)
                                .UseLocation(Vec2(0, 8) * 1_m)
                                .UseLinearAcceleration(GetGravity());
            const auto body = CreateBody(GetWorld(), bd);

            const auto conf =
                PolygonShapeConf{}.UseFriction(Real(0.6)).UseDensity(2_kgpm2).SetAsBox(0.75_m,
                                                                                       0.75_m);
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        // Drive the kinematic body.
        if (GetType(GetWorld(), m_platform) == BodyType::Kinematic) {
            const auto p = GetLocation(GetWorld(), m_platform);
            const auto velocity = GetVelocity(GetWorld(), m_platform);

            if ((GetX(p) < -10_m && GetX(velocity.linear) < 0_mps) ||
                (GetX(p) > +10_m && GetX(velocity.linear) > 0_mps)) {
                SetVelocity(GetWorld(), m_platform,
                            Velocity{LinearVelocity2{-GetX(velocity.linear), GetY(velocity.linear)},
                                     velocity.angular});
            }
        }
    }

    BodyID m_attachment;
    BodyID m_platform;
    Real m_speed;
};

} // namespace testbed

#endif
