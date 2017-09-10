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

namespace playrho {

class BodyTypes : public Test
{
public:
    BodyTypes()
    {
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter));

        // Define attachment
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(0.0f, 3.0f) * Meter;
            m_attachment = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{2} * KilogramPerSquareMeter;
            m_attachment->CreateFixture(std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{2.0f} * Meter, conf));
        }

        // Define platform
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-4.0f, 5.0f) * Meter;
            m_platform = m_world->CreateBody(bd);

            auto conf = PolygonShape::Conf{};
            conf.friction = 0.6f;
            conf.density = Real{2} * KilogramPerSquareMeter;
            PolygonShape shape{conf};
            SetAsBox(shape, Real{0.5f} * Meter, Real{4.0f} * Meter, Vec2(4.0f, 0.0f) * Meter, Real{0.5f} * Pi * Radian);

            m_platform->CreateFixture(std::make_shared<PolygonShape>(shape));

            RevoluteJointDef rjd(m_attachment, m_platform, Vec2(0.0f, 5.0f) * Meter);
            rjd.maxMotorTorque = Torque{Real{50.0f} * NewtonMeter};
            rjd.enableMotor = true;
            m_world->CreateJoint(rjd);

            PrismaticJointDef pjd(ground, m_platform, Vec2(0.0f, 5.0f) * Meter, UnitVec2::GetRight());
            pjd.maxMotorForce = Real{1000.0f} * Newton;
            pjd.enableMotor = true;
            pjd.lowerTranslation = Real{-10.0f} * Meter;
            pjd.upperTranslation = Real{10.0f} * Meter;
            pjd.enableLimit = true;
            m_world->CreateJoint(pjd);

            m_speed = 3.0f;
        }

        // Create a payload
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(0.0f, 8.0f) * Meter;
            Body* body = m_world->CreateBody(bd);

            auto conf = PolygonShape::Conf{};
            conf.friction = 0.6f;
            conf.density = Real{2} * KilogramPerSquareMeter;

            body->CreateFixture(std::make_shared<PolygonShape>(Real{0.75f} * Meter, Real{0.75f} * Meter, conf));
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_D:
            m_platform->SetType(BodyType::Dynamic);
            break;

        case Key_S:
            m_platform->SetType(BodyType::Static);
            break;

        case Key_K:
            m_platform->SetType(BodyType::Kinematic);
            m_platform->SetVelocity(Velocity{Vec2(-m_speed, 0.0f) * MeterPerSecond, AngularVelocity{0}});
            break;
    
        default:
            break;
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        // Drive the kinematic body.
        if (m_platform->GetType() == BodyType::Kinematic)
        {
            const auto p = m_platform->GetLocation();
            const auto velocity = m_platform->GetVelocity();

            if ((GetX(p) < Real{-10.0f} * Meter && GetX(velocity.linear) < Real{0.0f} * MeterPerSecond) ||
                (GetX(p) > Real{10.0f} * Meter && GetX(velocity.linear) > Real{0.0f} * MeterPerSecond))
            {
                m_platform->SetVelocity(Velocity{
                    {-GetX(velocity.linear), GetY(velocity.linear)}, velocity.angular
                });
            }
        }
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    Body* m_attachment;
    Body* m_platform;
    Real m_speed;
};

} // namespace playrho

#endif
