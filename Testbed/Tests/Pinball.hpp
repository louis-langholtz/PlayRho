/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_PINBALL_HPP
#define PLAYRHO_PINBALL_HPP

#include "../Framework/Test.hpp"

namespace playrho {

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
class Pinball : public Test
{
public:
    Pinball()
    {
        // Ground body
        const auto ground = m_world.CreateBody();
        {
            auto conf = ChainShape::Conf{};
            conf.vertices.push_back(Vec2(0.0f, -2.0f) * Meter);
            conf.vertices.push_back(Vec2(8.0f, 6.0f) * Meter);
            conf.vertices.push_back(Vec2(8.0f, 20.0f) * Meter);
            conf.vertices.push_back(Vec2(-8.0f, 20.0f) * Meter);
            conf.vertices.push_back(Vec2(-8.0f, 6.0f) * Meter);
            conf.vertices.push_back(conf.vertices[0]); // to loop back around completely.
            conf.UseDensity(Real(0) * KilogramPerSquareMeter);
            ground->CreateFixture(std::make_shared<ChainShape>(conf));
        }

        // Flippers
        {
            const auto p1 = Vec2(-2.0f, 0.0f) * Meter;
            const auto p2 = Vec2(+2.0f, 0.0f) * Meter;

            BodyDef bd;
            bd.type = BodyType::Dynamic;

            bd.location = p1;
            const auto leftFlipper = m_world.CreateBody(bd);

            bd.location = p2;
            const auto rightFlipper = m_world.CreateBody(bd);

            const auto box = std::make_shared<PolygonShape>(Real{1.75f} * Meter, Real{0.1f} * Meter);
            box->SetDensity(Real{1} * KilogramPerSquareMeter);

            leftFlipper->CreateFixture(box);
            rightFlipper->CreateFixture(box);

            RevoluteJointDef jd;
            jd.bodyA = ground;
            jd.localAnchorB = Vec2_zero * Meter;
            jd.enableMotor = true;
            jd.maxMotorTorque = Real{1000.0f} * NewtonMeter;
            jd.enableLimit = true;

            jd.motorSpeed = AngularVelocity{0};
            jd.localAnchorA = p1;
            jd.bodyB = leftFlipper;
            jd.lowerAngle = Real{-30.0f} * Degree;
            jd.upperAngle = Real{5.0f} * Degree;
            m_leftJoint = static_cast<RevoluteJoint*>(m_world.CreateJoint(jd));

            jd.motorSpeed = AngularVelocity{0};
            jd.localAnchorA = p2;
            jd.bodyB = rightFlipper;
            jd.lowerAngle = Real{-5.0f} * Degree;
            jd.upperAngle = Real{30.0f} * Degree;
            m_rightJoint = static_cast<RevoluteJoint*>(m_world.CreateJoint(jd));
        }

        // Disk character
        {
            BodyDef bd;
            bd.location = Vec2(1.0f, 15.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.bullet = true;

            m_ball = m_world.CreateBody(bd);

            auto conf = DiskShape::Conf{};
            conf.density = Real{1} * KilogramPerSquareMeter;
            conf.vertexRadius = Real{0.2f} * Meter;
            m_ball->CreateFixture(std::make_shared<DiskShape>(conf));
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
            m_leftJoint->SetMotorSpeed(Real{20.0f} * RadianPerSecond);
            m_rightJoint->SetMotorSpeed(Real{-20.0f} * RadianPerSecond);
        }
        else
        {
            m_leftJoint->SetMotorSpeed(Real{-10.0f} * RadianPerSecond);
            m_rightJoint->SetMotorSpeed(Real{10.0f} * RadianPerSecond);
        }
    }

    RevoluteJoint* m_leftJoint;
    RevoluteJoint* m_rightJoint;
    Body* m_ball;
    bool m_button = false;
};

} // namespace playrho

#endif
