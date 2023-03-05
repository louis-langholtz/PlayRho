/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_PINBALL_HPP
#define PLAYRHO_PINBALL_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
class Pinball : public Test
{
public:
    Pinball()
    {
        // Ground body
        const auto ground = CreateBody(GetWorld());
        {
            auto conf = ChainShapeConf{};
            conf.Add(Vec2(0.0f, -2.0f) * 1_m);
            conf.Add(Vec2(8.0f, 6.0f) * 1_m);
            conf.Add(Vec2(8.0f, 20.0f) * 1_m);
            conf.Add(Vec2(-8.0f, 20.0f) * 1_m);
            conf.Add(Vec2(-8.0f, 6.0f) * 1_m);
            conf.Add(conf.GetVertex(0)); // to loop back around completely.
            conf.UseDensity(0_kgpm2);
            CreateFixture(GetWorld(), ground, Shape(conf));
        }

        // Flippers
        {
            const auto p1 = Vec2(-2.0f, 0.0f) * 1_m;
            const auto p2 = Vec2(+2.0f, 0.0f) * 1_m;

            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();

            bd.location = p1;
            const auto leftFlipper = CreateBody(GetWorld(), bd);

            bd.location = p2;
            const auto rightFlipper = CreateBody(GetWorld(), bd);

            const auto box = Shape(PolygonShapeConf{}.SetAsBox(1.75_m, 0.1_m).UseDensity(1_kgpm2));
            CreateFixture(GetWorld(), leftFlipper, box);
            CreateFixture(GetWorld(), rightFlipper, box);

            RevoluteJointConf jd;
            jd.bodyA = ground;
            jd.localAnchorB = Length2{};
            jd.enableMotor = true;
            jd.maxMotorTorque = 1000_Nm;
            jd.enableLimit = true;

            jd.motorSpeed = 0_rpm;
            jd.localAnchorA = p1;
            jd.bodyB = leftFlipper;
            jd.lowerAngle = -30_deg;
            jd.upperAngle = 5_deg;
            m_leftJoint = CreateJoint(GetWorld(), jd);

            jd.motorSpeed = 0_rpm;
            jd.localAnchorA = p2;
            jd.bodyB = rightFlipper;
            jd.lowerAngle = -5_deg;
            jd.upperAngle = 30_deg;
            m_rightJoint = CreateJoint(GetWorld(), jd);
        }

        // Disk character
        {
            BodyConf bd;
            bd.location = Vec2(1.0f, 15.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.bullet = true;

            m_ball = CreateBody(GetWorld(), bd);

            auto conf = DiskShapeConf{};
            conf.density = 1_kgpm2;
            conf.vertexRadius = 0.2_m;
            CreateFixture(GetWorld(), m_ball, Shape(conf));
        }
        
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "To control the flippers", [&](KeyActionMods) {
            m_button = true;
        });
        RegisterForKey(GLFW_KEY_A, GLFW_RELEASE, 0, "To control the flippers", [&](KeyActionMods) {
            m_button = false;
        });
    }

    void PreStep(const Settings&, Drawer&) override
    {
        if (m_button)
        {
            SetMotorSpeed(GetWorld(), m_leftJoint, 20_rad / 1_s);
            SetMotorSpeed(GetWorld(), m_rightJoint, -20_rad / 1_s);
        }
        else
        {
            SetMotorSpeed(GetWorld(), m_leftJoint, -10_rad / 1_s);
            SetMotorSpeed(GetWorld(), m_rightJoint, 10_rad / 1_s);
        }
    }

    JointID m_leftJoint; // RevoluteJoint
    JointID m_rightJoint; // RevoluteJoint
    BodyID m_ball;
    bool m_button = false;
};

} // namespace testbed

#endif
