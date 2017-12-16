/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_BODY_TYPES_HPP
#define PLAYRHO_BODY_TYPES_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class BodyTypes : public Test
{
public:
    BodyTypes()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape(EdgeShapeConf{}.Set(Vec2(-20, 0) * 1_m, Vec2(20, 0) * 1_m)));

        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Dynamic", [&](KeyActionMods) {
            m_platform->SetType(BodyType::Dynamic);
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Static", [&](KeyActionMods) {
            m_platform->SetType(BodyType::Static);
        });
        RegisterForKey(GLFW_KEY_K, GLFW_PRESS, 0, "Kinematic", [&](KeyActionMods) {
            m_platform->SetType(BodyType::Kinematic);
            m_platform->SetVelocity(Velocity2D{Vec2(-m_speed, 0) * 1_mps, 0_rpm});
        });

        // Define attachment
        {
            const auto bd = BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0, 3) * 1_m);
            m_attachment = m_world.CreateBody(bd);
            const auto conf = PolygonShapeConf{}.UseDensity(2_kgpm2).SetAsBox(0.5_m, 2_m);
            m_attachment->CreateFixture(Shape(conf));
        }

        // Define platform
        {
            const auto bd = BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(-4, 5) * 1_m);
            m_platform = m_world.CreateBody(bd);

            const auto conf = PolygonShapeConf{}.UseFriction(Real(0.6f)).UseDensity(2_kgpm2)
                .SetAsBox(0.5_m, 4_m, Vec2(4, 0) * 1_m, Pi * 0.5_rad);
            m_platform->CreateFixture(Shape{conf});

            RevoluteJointDef rjd(m_attachment, m_platform, Vec2(0, 5) * 1_m);
            rjd.maxMotorTorque = 50_Nm;
            rjd.enableMotor = true;
            m_world.CreateJoint(rjd);

            PrismaticJointDef pjd(ground, m_platform, Vec2(0, 5) * 1_m, UnitVec2::GetRight());
            pjd.maxMotorForce = 1000_N;
            pjd.enableMotor = true;
            pjd.lowerTranslation = -10_m;
            pjd.upperTranslation = 10_m;
            pjd.enableLimit = true;
            m_world.CreateJoint(pjd);

            m_speed = 3.0f;
        }

        // Create a payload
        {
            const auto bd = BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0, 8) * 1_m);
            const auto body = m_world.CreateBody(bd);

            const auto conf = PolygonShapeConf{}.UseFriction(Real(0.6f)).UseDensity(2_kgpm2).SetAsBox(0.75_m, 0.75_m);
            body->CreateFixture(Shape(conf));
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {        
        // Drive the kinematic body.
        if (m_platform->GetType() == BodyType::Kinematic)
        {
            const auto p = m_platform->GetLocation();
            const auto velocity = m_platform->GetVelocity();

            if ((GetX(p) < -10_m && GetX(velocity.linear) < 0_mps) ||
                (GetX(p) > +10_m && GetX(velocity.linear) > 0_mps))
            {
                m_platform->SetVelocity(Velocity2D{
                    LinearVelocity2{-GetX(velocity.linear), GetY(velocity.linear)},
                    velocity.angular
                });
            }
        }
    }

    Body* m_attachment;
    Body* m_platform;
    Real m_speed;
};

} // namespace testbed

#endif
