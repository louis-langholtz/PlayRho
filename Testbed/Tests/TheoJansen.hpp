/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

// Inspired by a contribution by roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

#ifndef THEO_JANSEN_H
#define THEO_JANSEN_H

#include "../Framework/Test.hpp"

namespace box2d {

class TheoJansen : public Test
{
public:

    void CreateLeg(Real s, const Length2D wheelAnchor)
    {
        const auto p1 = Vec2(5.4f * s, -6.1f) * Meter;
        const auto p2 = Vec2(7.2f * s, -1.2f) * Meter;
        const auto p3 = Vec2(4.3f * s, -1.9f) * Meter;
        const auto p4 = Vec2(3.1f * s, 0.8f) * Meter;
        const auto p5 = Vec2(6.0f * s, 1.5f) * Meter;
        const auto p6 = Vec2(2.5f * s, 3.7f) * Meter;

        PolygonShape poly1, poly2;
        if (s > 0.0f)
        {
            poly1.Set({p1, p2, p3});
            poly2.Set({Vec2_zero * Meter, p5 - p4, p6 - p4});
        }
        else
        {
            poly1.Set({p1, p3, p2});
            poly2.Set({Vec2_zero * Meter, p6 - p4, p5 - p4});
        }
        poly1.SetDensity(Real{1} * KilogramPerSquareMeter);
        poly2.SetDensity(Real{1} * KilogramPerSquareMeter);

        FixtureDef fd1, fd2;
        fd1.filter.groupIndex = -1;
        fd2.filter.groupIndex = -1;
        
        BodyDef bd1, bd2;
        bd1.type = BodyType::Dynamic;
        bd2.type = BodyType::Dynamic;
        bd1.position = m_offset;
        bd2.position = p4 + m_offset;

        bd1.angularDamping = Real(10) * Hertz;
        bd2.angularDamping = Real(10) * Hertz;

        const auto body1 = m_world->CreateBody(bd1);
        const auto body2 = m_world->CreateBody(bd2);

        body1->CreateFixture(std::make_shared<PolygonShape>(poly1), fd1);
        body2->CreateFixture(std::make_shared<PolygonShape>(poly2), fd2);

        // Using a soft distance constraint can reduce some jitter.
        // It also makes the structure seem a bit more fluid by
        // acting like a suspension system.
        m_world->CreateJoint(DistanceJointDef{
            body1, body2, p2 + m_offset, p5 + m_offset, Real(10) * Hertz, Real(0.5)
        });
        m_world->CreateJoint(DistanceJointDef{
            body1, body2, p3 + m_offset, p4 + m_offset, Real(10) * Hertz, Real(0.5)
        });
        m_world->CreateJoint(DistanceJointDef{
            body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset, Real(10) * Hertz, Real(0.5)
        });
        m_world->CreateJoint(DistanceJointDef{
            body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset, Real(10) * Hertz, Real(0.5)
        });
        m_world->CreateJoint(RevoluteJointDef{body2, m_chassis, p4 + m_offset});
    }

    TheoJansen()
    {
        m_offset = Vec2(0.0f, 8.0f) * Meter;
        m_motorSpeed = Real{2.0f} * RadianPerSecond;
        m_motorOn = true;
        const auto pivot = Vec2(0.0f, 0.8f) * Meter;

        // Ground
        {
            BodyDef bd;
            const auto ground = m_world->CreateBody(bd);

            EdgeShape shape;
            shape.Set(Vec2(-50.0f, 0.0f) * Meter, Vec2(50.0f, 0.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(Vec2(-50.0f, 0.0f) * Meter, Vec2(-50.0f, 10.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(Vec2(50.0f, 0.0f) * Meter, Vec2(50.0f, 10.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        // Balls
        auto circleConf = DiskShape::Conf{};
        circleConf.vertexRadius = Real{0.25f} * Meter;
        circleConf.density = Real{1} * KilogramPerSquareMeter;
        const auto circle = std::make_shared<DiskShape>(circleConf);
        for (auto i = 0; i < 40; ++i)
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-40.0f + 2.0f * i, 0.5f) * Meter;

            const auto body = m_world->CreateBody(bd);
            body->CreateFixture(circle);
        }

        // Chassis
        {
            FixtureDef sd;
            sd.filter.groupIndex = -1;
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = pivot + m_offset;
            m_chassis = m_world->CreateBody(bd);
            auto polygonConf = PolygonShape::Conf{};
            polygonConf.density = Real{1} * KilogramPerSquareMeter;
            m_chassis->CreateFixture(std::make_shared<PolygonShape>(Real{2.5f} * Meter, Real{1.0f} * Meter, polygonConf), sd);
        }

        {
            FixtureDef sd;
            sd.filter.groupIndex = -1;
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = pivot + m_offset;
            m_wheel = m_world->CreateBody(bd);
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = Real{1.6f} * Meter;
            conf.density = Real{1} * KilogramPerSquareMeter;
            m_wheel->CreateFixture(std::make_shared<DiskShape>(conf), sd);
        }

        {
            RevoluteJointDef jd{m_wheel, m_chassis, pivot + m_offset};
            jd.collideConnected = false;
            jd.motorSpeed = m_motorSpeed;
            jd.maxMotorTorque = Real{400.0f} * NewtonMeter;
            jd.enableMotor = m_motorOn;
            m_motorJoint = (RevoluteJoint*)m_world->CreateJoint(jd);
        }

        const auto wheelAnchor = pivot + Vec2(0.0f, -0.8f) * Meter;

        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel->SetTransform(m_wheel->GetLocation(), Real{120.0f} * Degree);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel->SetTransform(m_wheel->GetLocation(), Real{-120.0f} * Degree);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_A:
            m_motorJoint->SetMotorSpeed(-m_motorSpeed);
            break;

        case Key_S:
            m_motorJoint->SetMotorSpeed(Real{0.0f} * RadianPerSecond);
            break;

        case Key_D:
            m_motorJoint->SetMotorSpeed(m_motorSpeed);
            break;

        case Key_M:
            m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
            break;

        default:
            break;                
        }
    }

    Length2D m_offset;
    Body* m_chassis;
    Body* m_wheel;
    RevoluteJoint* m_motorJoint;
    bool m_motorOn;
    AngularVelocity m_motorSpeed;
};

} // namespace box2d

#endif // THEO_JANSEN_H
