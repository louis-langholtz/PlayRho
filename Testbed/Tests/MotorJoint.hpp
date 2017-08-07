/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef MOTOR_JOINT_H
#define MOTOR_JOINT_H

#include "../Framework/Test.hpp"

namespace playrho {

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJointTest : public Test
{
public:
    MotorJointTest()
    {
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter));

        // Define motorized body
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.position = Vec2(0.0f, 8.0f) * Meter;
        const auto body = m_world->CreateBody(bd);

        auto conf = PolygonShape::Conf{};
        conf.friction = 0.6f;
        conf.density = Real{2} * KilogramPerSquareMeter;
        body->CreateFixture(std::make_shared<PolygonShape>(Real{2.0f} * Meter, Real{0.5f} * Meter, conf));

        auto mjd = MotorJointDef{ground, body};
        mjd.maxForce = Real{1000.0f} * Newton;
        mjd.maxTorque = Real{1000.0f} * NewtonMeter;
        m_joint = (MotorJoint*)m_world->CreateJoint(mjd);
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_S:
            m_go = !m_go;
            break;
        default:
            break;
        }
    }

    void PreStep(const Settings& settings, Drawer& drawer) override
    {
        if (m_go && settings.dt > 0)
        {
            m_time += settings.dt;
        }

        const auto linearOffset = Vec2{
            Real{6} * std::sin(Real{2} * m_time),
            Real{8} + Real{4} * std::sin(Real{1} * m_time)
        } * Meter;

        m_joint->SetLinearOffset(linearOffset);
        m_joint->SetAngularOffset(Real{4} * Radian * m_time);

        drawer.DrawPoint(linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "Keys: (s) pause");
        m_textLine += 15;
    }

    MotorJoint* m_joint;
    Real m_time = 0;
    bool m_go = false;
};

} // namespace playrho

#endif
