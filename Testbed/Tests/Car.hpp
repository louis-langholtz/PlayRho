/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_CAR_HPP
#define  PLAYRHO_CAR_HPP

#include "../Framework/Test.hpp"
#include <sstream>

namespace playrho {

// This is a fun demo that shows off the wheel joint
class Car : public Test
{
public:
    Car()
    {        
        m_hz = 4_Hz;
        m_zeta = 0.7f;
        m_speed = 50_rad / 1_s;

        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Move Left.", [&](KeyActionMods) {
            m_spring1->SetMotorSpeed(m_speed);
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Brake.", [&](KeyActionMods) {
            m_spring1->SetMotorSpeed(0_rpm);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Move Right.", [&](KeyActionMods) {
            m_spring1->SetMotorSpeed(-m_speed);
        });
        RegisterForKey(GLFW_KEY_Q, GLFW_PRESS, 0, "Decrease Frequency.", [&](KeyActionMods) {
            m_hz = std::max(0_Hz, m_hz - 1_Hz);
            m_spring1->SetSpringFrequency(m_hz);
            m_spring2->SetSpringFrequency(m_hz);
        });
        RegisterForKey(GLFW_KEY_E, GLFW_PRESS, 0, "Increase Frequency.", [&](KeyActionMods) {
            m_hz += 1_Hz;
            m_spring1->SetSpringFrequency(m_hz);
            m_spring2->SetSpringFrequency(m_hz);
        });
        
        const auto ground = m_world.CreateBody();
        {
            auto conf = EdgeShape::Conf{};
            conf.UseDensity(0_kgpm2).UseFriction(Real(0.6f));

            conf.Set(Vec2(-20.0f, 0.0f) * 1_m, Vec2(20.0f, 0.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            Real hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

            auto x = Real{20.0f};
            auto y1 = Real{0.0f};
            const auto dx = Real{5.0f};

            for (auto i = 0; i < 10; ++i)
            {
                const auto y2 = hs[i];
                conf.Set(Vec2(x, y1) * 1_m, Vec2(x + dx, y2) * 1_m);
                ground->CreateFixture(Shape(conf));
                y1 = y2;
                x += dx;
            }

            for (auto i = 0; i < 10; ++i)
            {
                const auto y2 = hs[i];
                conf.Set(Vec2(x, y1) * 1_m, Vec2(x + dx, y2) * 1_m);
                ground->CreateFixture(Shape(conf));
                y1 = y2;
                x += dx;
            }

            conf.Set(Vec2(x, 0.0f) * 1_m, Vec2(x + 40.0f, 0.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            x += 80.0f;
            conf.Set(Vec2(x, 0.0f) * 1_m, Vec2(x + 40.0f, 0.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            x += 40.0f;
            conf.Set(Vec2(x, 0.0f) * 1_m, Vec2(x + 10.0f, 5.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            x += 20.0f;
            conf.Set(Vec2(x, 0.0f) * 1_m, Vec2(x + 40.0f, 0.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            x += 40.0f;
            conf.Set(Vec2(x, 0.0f) * 1_m, Vec2(x, 20.0f) * 1_m);
            ground->CreateFixture(Shape(conf));
        }

        // Teeter
        {
            BodyDef bd;
            bd.location = Vec2(140.0f, 1.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            const auto body = m_world.CreateBody(bd);
            body->CreateFixture(Shape{PolygonShape::Conf{}.SetDensity(1_kgpm2).SetAsBox(10_m, 0.25_m)});

            RevoluteJointDef jd(ground, body, body->GetLocation());
            jd.lowerAngle = -8_deg;
            jd.upperAngle = +8_deg;
            jd.enableLimit = true;
            m_world.CreateJoint(jd);

            // AngularMomentum is L^2 M T^-1 QP^-1.
            ApplyAngularImpulse(*body, 100_m2 * 1_kg / (1_s * 1_rad));
        }

        // Bridge
        {
            const auto N = 20;
            const auto shape = Shape{
                PolygonShape::Conf{}.SetDensity(1_kgpm2).SetFriction(Real(0.6f)).SetAsBox(1_m, 0.125_m)
            };
            auto prevBody = ground;
            for (auto i = 0; i < N; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(161.0f + 2.0f * i, -0.125f) * 1_m;
                const auto body = m_world.CreateBody(bd);
                body->CreateFixture(shape);

                m_world.CreateJoint(RevoluteJointDef{prevBody, body,
                    Vec2(160.0f + 2.0f * i, -0.125f) * 1_m});

                prevBody = body;
            }
            m_world.CreateJoint(RevoluteJointDef{prevBody, ground,
                Vec2(160.0f + 2.0f * N, -0.125f) * 1_m});
        }

        // Boxes
        {
            const auto box = Shape{PolygonShape::Conf{}.SetDensity(0.5_kgpm2).SetAsBox(0.5_m, 0.5_m)};
            auto body = static_cast<Body*>(nullptr);

            BodyDef bd;
            bd.type = BodyType::Dynamic;

            bd.location = Vec2(230.0f, 0.5f) * 1_m;
            body = m_world.CreateBody(bd);
            body->CreateFixture(box);

            bd.location = Vec2(230.0f, 1.5f) * 1_m;
            body = m_world.CreateBody(bd);
            body->CreateFixture(box);

            bd.location = Vec2(230.0f, 2.5f) * 1_m;
            body = m_world.CreateBody(bd);
            body->CreateFixture(box);

            bd.location = Vec2(230.0f, 3.5f) * 1_m;
            body = m_world.CreateBody(bd);
            body->CreateFixture(box);

            bd.location = Vec2(230.0f, 4.5f) * 1_m;
            body = m_world.CreateBody(bd);
            body->CreateFixture(box);
        }

        // Car
        {
            const auto chassis = Shape(PolygonShape::Conf{}
                .UseDensity(1_kgpm2).UseVertices({
                    Vec2(-1.5f, -0.5f) * 1_m,
                    Vec2(1.5f, -0.5f) * 1_m,
                    Vec2(1.5f, 0.0f) * 1_m,
                    Vec2(0.0f, 0.9f) * 1_m,
                    Vec2(-1.15f, 0.9f) * 1_m,
                    Vec2(-1.5f, 0.2f) * 1_m
                }));

            const auto circle = Shape{
                DiskShapeConf{}.SetRadius(0.4_m).UseDensity(1_kgpm2).UseFriction(Real(0.9f))
            };
            
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(0.0f, 1.0f) * 1_m;
            m_car = m_world.CreateBody(bd);
            m_car->CreateFixture(chassis);

            bd.location = Vec2(-1.0f, 0.35f) * 1_m;
            m_wheel1 = m_world.CreateBody(bd);
            m_wheel1->CreateFixture(circle);

            bd.location = Vec2(1.0f, 0.4f) * 1_m;
            m_wheel2 = m_world.CreateBody(bd);
            m_wheel2->CreateFixture(circle);

            const auto axis = UnitVec2::GetTop();

            {
                WheelJointDef jd(m_car, m_wheel1, m_wheel1->GetLocation(), axis);
                jd.motorSpeed = 0_rpm;
                jd.maxMotorTorque = 20_Nm;
                jd.enableMotor = true;
                jd.frequency = m_hz;
                jd.dampingRatio = m_zeta;
                m_spring1 = static_cast<WheelJoint*>(m_world.CreateJoint(jd));
            }
            {
                WheelJointDef jd(m_car, m_wheel2, m_wheel2->GetLocation(), axis);
                jd.motorSpeed = 0_rpm;
                jd.maxMotorTorque = 10_Nm;
                jd.enableMotor = false;
                jd.frequency = m_hz;
                jd.dampingRatio = m_zeta;
                m_spring2 = static_cast<WheelJoint*>(m_world.CreateJoint(jd));
            }
        }
    }

    void PreStep(const Settings&, Drawer& drawer) override
    {
        drawer.SetTranslation(Length2{GetX(m_car->GetLocation()), GetY(drawer.GetTranslation())});
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Frequency = " << static_cast<double>(Real{m_hz / 1_Hz}) << " hz, ";
        stream << "damping ratio = " << m_zeta;
        m_status = stream.str();
    }

    Body* m_car;
    Body* m_wheel1;
    Body* m_wheel2;

    Frequency m_hz ;
    Real m_zeta;
    AngularVelocity m_speed;
    WheelJoint* m_spring1;
    WheelJoint* m_spring2;
};

} // namespace playrho

#endif
