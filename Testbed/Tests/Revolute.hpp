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

#ifndef PLAYRHO_REVOLUTE_HPP
#define PLAYRHO_REVOLUTE_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class Revolute : public Test
{
public:
    Revolute()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * 1_m,
                                                          Vec2( 40.0f, 0.0f) * 1_m));

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;

            bd.location = Vec2(-10.0f, 20.0f) * 1_m;
            const auto body = m_world.CreateBody(bd);
            auto circleConf = DiskShape::Conf{};
            circleConf.vertexRadius = 0.5_m;
            circleConf.density = 5_kgpm2;
            body->CreateFixture(std::make_shared<DiskShape>(circleConf));

            const auto w = 100.0f;
            body->SetVelocity(Velocity{
                Vec2(-8.0f * w, 0.0f) * 1_mps, w * 1_rad / 1_s
            });
            
            RevoluteJointDef rjd(ground, body, Vec2(-10.0f, 12.0f) * 1_m);
            rjd.motorSpeed = Pi * 1_rad / 1_s;
            rjd.maxMotorTorque = 10000_Nm;
            rjd.enableMotor = false;
            rjd.lowerAngle = -0.25_rad * Pi;
            rjd.upperAngle = 0.5_rad * Pi;
            rjd.enableLimit = true;
            rjd.collideConnected = true;

            m_joint = (RevoluteJoint*)m_world.CreateJoint(rjd);
        }

        {
            BodyDef circle_bd;
            circle_bd.type = BodyType::Dynamic;
            circle_bd.location = Vec2(5.0f, 30.0f) * 1_m;

            FixtureDef fd;
            fd.filter.maskBits = 1;

            m_ball = m_world.CreateBody(circle_bd);
            auto circleConf = DiskShape::Conf{};
            circleConf.vertexRadius = 3_m;
            circleConf.density = 5_kgpm2;
            m_ball->CreateFixture(std::make_shared<DiskShape>(circleConf), fd);

            auto polygon_shape = PolygonShape::Conf{};
            polygon_shape.SetAsBox(10_m, 0.2_m, Vec2(-10.0f, 0.0f) * 1_m, 0_rad);
            polygon_shape.SetDensity(2_kgpm2);

            BodyDef polygon_bd;
            polygon_bd.location = Vec2(20.0f, 10.0f) * 1_m;
            polygon_bd.type = BodyType::Dynamic;
            polygon_bd.bullet = true;
            const auto polygon_body = m_world.CreateBody(polygon_bd);
            polygon_body->CreateFixture(std::make_shared<PolygonShape>(polygon_shape));

            RevoluteJointDef rjd(ground, polygon_body, Vec2(20.0f, 10.0f) * 1_m);
            rjd.lowerAngle = -0.25_rad * Pi;
            rjd.upperAngle = 0_rad * Pi;
            rjd.enableLimit = true;
            m_world.CreateJoint(rjd);
        }

        // Tests mass computation of a small object far from the origin
        {
            const auto polyShape = PolygonShape::Conf{}.Set({
                Vec2(17.63f, 36.31f) * 1_m,
                Vec2(17.52f, 36.69f) * 1_m,
                Vec2(17.19f, 36.36f) * 1_m
            }).SetDensity(1_kgpm2);
        
            const auto body = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic));
            body->CreateFixture(std::make_shared<PolygonShape>(polyShape));
        }
        
        RegisterForKey(GLFW_KEY_L, GLFW_PRESS, 0, "Limits", [&](KeyActionMods) {
            m_joint->EnableLimit(!m_joint->IsLimitEnabled());
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Motor", [&](KeyActionMods) {
            m_joint->EnableMotor(!m_joint->IsMotorEnabled());
        });
    }

    Body* m_ball;
    RevoluteJoint* m_joint;
};

} // namespace playrho

#endif
