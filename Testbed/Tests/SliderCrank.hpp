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

#ifndef SLIDER_CRANK_H
#define SLIDER_CRANK_H

#include "../Framework/Test.hpp"

namespace playrho {

// A motor driven slider crank with joint friction.
class SliderCrank : public Test
{
public:
    SliderCrank()
    {
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));

        {
            auto prevBody = ground;

            // Define crank.
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(0.0f, 7.0f) * Meter;
                const auto body = m_world->CreateBody(bd);
                auto shapeConf = PolygonShape::Conf{};
                shapeConf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{2.0f} * Meter, shapeConf));

                RevoluteJointDef rjd{prevBody, body, Vec2(0.0f, 5.0f) * Meter};
                rjd.motorSpeed = 1.0f * Pi * RadianPerSecond;
                rjd.maxMotorTorque = Real{10000.0f} * NewtonMeter;
                rjd.enableMotor = true;
                m_joint1 = (RevoluteJoint*)m_world->CreateJoint(rjd);

                prevBody = body;
            }

            // Define follower.
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(0.0f, 13.0f) * Meter;
                const auto body = m_world->CreateBody(bd);
                auto shapeConf = PolygonShape::Conf{};
                shapeConf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{4.0f} * Meter, shapeConf));

                RevoluteJointDef rjd{prevBody, body, Vec2(0.0f, 9.0f) * Meter};
                rjd.enableMotor = false;
                m_world->CreateJoint(rjd);

                prevBody = body;
            }

            // Define piston
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.fixedRotation = true;
                bd.position = Vec2(0.0f, 17.0f) * Meter;
                const auto body = m_world->CreateBody(bd);
                auto shapeConf = PolygonShape::Conf{};
                shapeConf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{1.5f} * Meter, Real{1.5f} * Meter, shapeConf));

                m_world->CreateJoint(RevoluteJointDef{prevBody, body, Vec2(0.0f, 17.0f) * Meter});

                PrismaticJointDef pjd(ground, body, Vec2(0.0f, 17.0f) * Meter, UnitVec2::GetTop());

                pjd.maxMotorForce = Real{1000.0f} * Newton;
                pjd.enableMotor = true;
                
                m_joint2 = static_cast<PrismaticJoint*>(m_world->CreateJoint(pjd));
            }

            // Create a payload
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(0.0f, 23.0f) * Meter;
                const auto body = m_world->CreateBody(bd);
                auto shapeConf = PolygonShape::Conf{};
                shapeConf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{1.5f} * Meter, Real{1.5f} * Meter, shapeConf));
            }
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_F:
            m_joint2->EnableMotor(!m_joint2->IsMotorEnabled());
            m_joint2->GetBodyB()->SetAwake();
            break;

        case Key_M:
            m_joint1->EnableMotor(!m_joint1->IsMotorEnabled());
            m_joint1->GetBodyB()->SetAwake();
            break;
        
        default:
            break;                
        }
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
        m_textLine += DRAW_STRING_NEW_LINE;
        const auto torque = m_joint1->GetMotorTorque(Real{settings.hz} * Hertz);
        drawer.DrawString(5, m_textLine, "Motor Torque = %5.0f",
                          static_cast<double>(Real{torque / NewtonMeter}));
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    RevoluteJoint* m_joint1;
    PrismaticJoint* m_joint2;
};

} // namespace playrho

#endif
