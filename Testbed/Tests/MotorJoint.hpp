/*
 * Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_TESTS_MOTOR_JOINT_HPP
#define PLAYRHO_TESTS_MOTOR_JOINT_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJointTest : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description =
            "A motor joint forces two bodies to have a given linear and/or angular"
            " offset(s) from each other.";
        return conf;
    }
    
    MotorJointTest(): Test(GetTestConf())
    {
        const auto ground = CreateBody(m_world);
        CreateFixture(m_world, ground, Shape{EdgeShapeConf{Vec2(-20.0f, 0.0f) * 1_m, Vec2(20.0f, 0.0f) * 1_m}});

        // Define motorized body
        const auto body = CreateBody(m_world, BodyConf{}
                                             .UseType(BodyType::Dynamic)
                                             .UseLocation(Vec2(0.0f, 8.0f) * 1_m)
                                             .UseLinearAcceleration(m_gravity));
        CreateFixture(m_world, body, Shape{
            PolygonShapeConf{}.SetAsBox(2_m, 0.5_m).UseFriction(Real(0.6)).UseDensity(2_kgpm2)
        });
        auto mjd = GetMotorJointConf(m_world, ground, body);
        mjd.maxForce = 1000_N;
        mjd.maxTorque = 1000_Nm;
        m_joint = m_world.CreateJoint(mjd);
        
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Pause Motor", [&](KeyActionMods) {
            m_go = !m_go;
        });
    }

    void PreStep(const Settings& settings, Drawer& drawer) override
    {
        m_status = m_go? "Motor going.": "Motor paused.";

        if (m_go && settings.dt > 0)
        {
            m_time += settings.dt;
        }

        const auto linearOffset = Vec2{6 * sin(2 * m_time), 8 + 4 * sin(m_time)} * 1_m;

        SetLinearOffset(m_world, m_joint, linearOffset);
        SetAngularOffset(m_world, m_joint, 4_rad * m_time);

        drawer.DrawPoint(linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
    }

    JointID m_joint; // motor joint
    Real m_time = 0;
    bool m_go = true;
};

} // namespace testbed

#endif /* PLAYRHO_TESTS_MOTOR_JOINT_HPP */
