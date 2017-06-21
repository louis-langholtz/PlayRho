/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef CAR_H
#define CAR_H

#include "../Framework/Test.hpp"

namespace box2d {

// This is a fun demo that shows off the wheel joint
class Car : public Test
{
public:
    Car()
    {        
        m_hz = RealNum{4} * Hertz;
        m_zeta = 0.7f;
        m_speed = RealNum{50} * RadianPerSecond;

        const auto ground = m_world->CreateBody();
        {
            EdgeShape shape;

            shape.Set(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter);
            shape.SetDensity(RealNum{0} * KilogramPerSquareMeter);
            shape.SetFriction(0.6f);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            RealNum hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

            auto x = RealNum{20.0f};
            auto y1 = RealNum{0.0f};
            const auto dx = RealNum{5.0f};

            for (auto i = 0; i < 10; ++i)
            {
                const auto y2 = hs[i];
                shape.Set(Vec2(x, y1) * Meter, Vec2(x + dx, y2) * Meter);
                ground->CreateFixture(std::make_shared<EdgeShape>(shape));
                y1 = y2;
                x += dx;
            }

            for (auto i = 0; i < 10; ++i)
            {
                const auto y2 = hs[i];
                shape.Set(Vec2(x, y1) * Meter, Vec2(x + dx, y2) * Meter);
                ground->CreateFixture(std::make_shared<EdgeShape>(shape));
                y1 = y2;
                x += dx;
            }

            shape.Set(Vec2(x, 0.0f) * Meter, Vec2(x + 40.0f, 0.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            x += 80.0f;
            shape.Set(Vec2(x, 0.0f) * Meter, Vec2(x + 40.0f, 0.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            x += 40.0f;
            shape.Set(Vec2(x, 0.0f) * Meter, Vec2(x + 10.0f, 5.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            x += 20.0f;
            shape.Set(Vec2(x, 0.0f) * Meter, Vec2(x + 40.0f, 0.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            x += 40.0f;
            shape.Set(Vec2(x, 0.0f) * Meter, Vec2(x, 20.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        // Teeter
        {
            BodyDef bd;
            bd.position = Vec2(140.0f, 1.0f) * Meter;
            bd.type = BodyType::Dynamic;
            const auto body = m_world->CreateBody(bd);

            const auto box = std::make_shared<PolygonShape>(RealNum{10.0f} * Meter, RealNum{0.25f} * Meter);
            box->SetDensity(RealNum{1} * KilogramPerSquareMeter);
            body->CreateFixture(box);

            RevoluteJointDef jd(ground, body, body->GetLocation());
            jd.lowerAngle = RealNum{-8.0f} * Degree;
            jd.upperAngle = RealNum{+8.0f} * Degree;
            jd.enableLimit = true;
            m_world->CreateJoint(jd);

            // AngularMomentum is L^2 M T^-1 QP^-1.
            ApplyAngularImpulse(*body, RealNum{100} * SquareMeter * Kilogram / (Second * Radian));
        }

        // Bridge
        {
            const auto N = 20;
            const auto shape = std::make_shared<PolygonShape>(RealNum{1.0f} * Meter, RealNum{0.125f} * Meter);
            shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
            shape->SetFriction(0.6f);

            auto prevBody = ground;
            for (auto i = 0; i < N; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(161.0f + 2.0f * i, -0.125f) * Meter;
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(shape);

                m_world->CreateJoint(RevoluteJointDef{prevBody, body,
                    Vec2(160.0f + 2.0f * i, -0.125f) * Meter});

                prevBody = body;
            }

            m_world->CreateJoint(RevoluteJointDef{prevBody, ground,
                Vec2(160.0f + 2.0f * N, -0.125f) * Meter});
        }

        // Boxes
        {
            const auto box = std::make_shared<PolygonShape>(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter);
            box->SetDensity(RealNum{0.5f} * KilogramPerSquareMeter);

            auto body = static_cast<Body*>(nullptr);

            BodyDef bd;
            bd.type = BodyType::Dynamic;

            bd.position = Vec2(230.0f, 0.5f) * Meter;
            body = m_world->CreateBody(bd);
            body->CreateFixture(box);

            bd.position = Vec2(230.0f, 1.5f) * Meter;
            body = m_world->CreateBody(bd);
            body->CreateFixture(box);

            bd.position = Vec2(230.0f, 2.5f) * Meter;
            body = m_world->CreateBody(bd);
            body->CreateFixture(box);

            bd.position = Vec2(230.0f, 3.5f) * Meter;
            body = m_world->CreateBody(bd);
            body->CreateFixture(box);

            bd.position = Vec2(230.0f, 4.5f) * Meter;
            body = m_world->CreateBody(bd);
            body->CreateFixture(box);
        }

        // Car
        {
            auto chassis = std::make_shared<PolygonShape>();
            chassis->Set({
                Vec2(-1.5f, -0.5f) * Meter,
                Vec2(1.5f, -0.5f) * Meter,
                Vec2(1.5f, 0.0f) * Meter,
                Vec2(0.0f, 0.9f) * Meter,
                Vec2(-1.15f, 0.9f) * Meter,
                Vec2(-1.5f, 0.2f) * Meter
            });
            chassis->SetDensity(RealNum{1} * KilogramPerSquareMeter);

            const auto circle = std::make_shared<DiskShape>(RealNum(0.4) * Meter);
            circle->SetDensity(RealNum{1} * KilogramPerSquareMeter);
            circle->SetFriction(0.9f);

            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(0.0f, 1.0f) * Meter;
            m_car = m_world->CreateBody(bd);
            m_car->CreateFixture(chassis);

            bd.position = Vec2(-1.0f, 0.35f) * Meter;
            m_wheel1 = m_world->CreateBody(bd);
            m_wheel1->CreateFixture(circle);

            bd.position = Vec2(1.0f, 0.4f) * Meter;
            m_wheel2 = m_world->CreateBody(bd);
            m_wheel2->CreateFixture(circle);

            const auto axis = UnitVec2::GetTop();

            {
                WheelJointDef jd(m_car, m_wheel1, m_wheel1->GetLocation(), axis);
                jd.motorSpeed = AngularVelocity{0};
                jd.maxMotorTorque = RealNum{20} * NewtonMeter;
                jd.enableMotor = true;
                jd.frequencyHz = m_hz;
                jd.dampingRatio = m_zeta;
                m_spring1 = static_cast<WheelJoint*>(m_world->CreateJoint(jd));
            }
            {
                WheelJointDef jd(m_car, m_wheel2, m_wheel2->GetLocation(), axis);
                jd.motorSpeed = AngularVelocity{0};
                jd.maxMotorTorque = RealNum{10} * NewtonMeter;
                jd.enableMotor = false;
                jd.frequencyHz = m_hz;
                jd.dampingRatio = m_zeta;
                m_spring2 = static_cast<WheelJoint*>(m_world->CreateJoint(jd));
            }
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_A:
            m_spring1->SetMotorSpeed(m_speed);
            break;

        case Key_S:
            m_spring1->SetMotorSpeed(AngularVelocity{0});
            break;

        case Key_D:
            m_spring1->SetMotorSpeed(-m_speed);
            break;

        case Key_Q:
            m_hz = Max(RealNum(0) * Hertz, m_hz - RealNum{1} * Hertz);
            m_spring1->SetSpringFrequencyHz(m_hz);
            m_spring2->SetSpringFrequencyHz(m_hz);
            break;

        case Key_E:
            m_hz += RealNum{1} * Hertz;
            m_spring1->SetSpringFrequencyHz(m_hz);
            m_spring2->SetSpringFrequencyHz(m_hz);
            break;
    
        default:
            break;
        }
    }

    void PreStep(const Settings&, Drawer& drawer) override
    {
        drawer.SetTranslation(Length2D{m_car->GetLocation().x, drawer.GetTranslation().y});
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine,
                          "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g",
                          static_cast<double>(RealNum{m_hz / Hertz}), m_zeta);
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    Body* m_car;
    Body* m_wheel1;
    Body* m_wheel2;

    Frequency m_hz ;
    RealNum m_zeta;
    AngularVelocity m_speed;
    WheelJoint* m_spring1;
    WheelJoint* m_spring2;
};

} // namespace box2d

#endif
