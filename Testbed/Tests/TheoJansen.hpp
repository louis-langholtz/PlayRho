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

// Inspired by a contribution by roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

#ifndef THEO_JANSEN_HPP
#define THEO_JANSEN_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class TheoJansen : public Test
{
public:

    void CreateLeg(Real s, const Length2 wheelAnchor)
    {
        const auto p1 = Vec2(5.4f * s, -6.1f) * 1_m;
        const auto p2 = Vec2(7.2f * s, -1.2f) * 1_m;
        const auto p3 = Vec2(4.3f * s, -1.9f) * 1_m;
        const auto p4 = Vec2(3.1f * s, 0.8f) * 1_m;
        const auto p5 = Vec2(6.0f * s, 1.5f) * 1_m;
        const auto p6 = Vec2(2.5f * s, 3.7f) * 1_m;

        auto poly1 = PolygonShapeConf{};
        auto poly2 = PolygonShapeConf{};
        if (s > 0.0f)
        {
            poly1.Set({p1, p2, p3});
            poly2.Set({Length2{}, p5 - p4, p6 - p4});
        }
        else
        {
            poly1.Set({p1, p3, p2});
            poly2.Set({Length2{}, p6 - p4, p5 - p4});
        }
        poly1.UseDensity(1_kgpm2);
        poly2.UseDensity(1_kgpm2);

        FixtureConf fd1, fd2;
        fd1.filter.groupIndex = -1;
        fd2.filter.groupIndex = -1;
        
        BodyConf bd1, bd2;
        bd1.type = BodyType::Dynamic;
        bd2.type = BodyType::Dynamic;
        bd1.location = m_offset;
        bd2.location = p4 + m_offset;

        bd1.angularDamping = 10_Hz;
        bd2.angularDamping = 10_Hz;

        const auto body1 = m_world.CreateBody(bd1);
        const auto body2 = m_world.CreateBody(bd2);

        body1->CreateFixture(Shape(poly1), fd1);
        body2->CreateFixture(Shape(poly2), fd2);

        // Using a soft distance constraint can reduce some jitter.
        // It also makes the structure seem a bit more fluid by
        // acting like a suspension system.

        m_world.CreateJoint(DistanceJointConf{body1, body2, p2 + m_offset, p5 + m_offset}
                             .UseFrequency(10_Hz).UseDampingRatio(Real(0.5)));
        m_world.CreateJoint(DistanceJointConf{body1, body2, p3 + m_offset, p4 + m_offset}
                             .UseFrequency(10_Hz).UseDampingRatio(Real(0.5)));
        m_world.CreateJoint(DistanceJointConf{body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset}
                             .UseFrequency(10_Hz).UseDampingRatio(Real(0.5)));
        m_world.CreateJoint(DistanceJointConf{body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset}
                             .UseFrequency(10_Hz).UseDampingRatio(Real(0.5)));
        m_world.CreateJoint(RevoluteJointConf{body2, m_chassis, p4 + m_offset});
    }

    TheoJansen()
    {
        m_offset = Vec2(0.0f, 8.0f) * 1_m;
        m_motorSpeed = 2_rad / 1_s;
        m_motorOn = true;
        const auto pivot = Vec2(0.0f, 0.8f) * 1_m;

        // Ground
        {
            BodyConf bd;
            const auto ground = m_world.CreateBody(bd);

            auto conf = EdgeShapeConf{};
 
            conf.Set(Vec2(-50.0f, 0.0f) * 1_m, Vec2(50.0f, 0.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            conf.Set(Vec2(-50.0f, 0.0f) * 1_m, Vec2(-50.0f, 10.0f) * 1_m);
            ground->CreateFixture(Shape(conf));

            conf.Set(Vec2(50.0f, 0.0f) * 1_m, Vec2(50.0f, 10.0f) * 1_m);
            ground->CreateFixture(Shape(conf));
        }

        // Balls
        auto circleConf = DiskShapeConf{};
        circleConf.vertexRadius = 0.25_m;
        circleConf.density = 1_kgpm2;
        const auto circle = Shape(circleConf);
        for (auto i = 0; i < 40; ++i)
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-40.0f + 2.0f * i, 0.5f) * 1_m;

            const auto body = m_world.CreateBody(bd);
            body->CreateFixture(circle);
        }

        // Chassis
        {
            FixtureConf sd;
            sd.filter.groupIndex = -1;
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = pivot + m_offset;
            m_chassis = m_world.CreateBody(bd);
            m_chassis->CreateFixture(PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(2.5_m, 1_m), sd);
        }

        {
            FixtureConf sd;
            sd.filter.groupIndex = -1;
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = pivot + m_offset;
            m_wheel = m_world.CreateBody(bd);
            auto conf = DiskShapeConf{};
            conf.vertexRadius = 1.6_m;
            conf.density = 1_kgpm2;
            m_wheel->CreateFixture(Shape(conf), sd);
        }

        {
            RevoluteJointConf jd{m_wheel, m_chassis, pivot + m_offset};
            jd.collideConnected = false;
            jd.motorSpeed = m_motorSpeed;
            jd.maxMotorTorque = 400_Nm;
            jd.enableMotor = m_motorOn;
            m_motorJoint = (RevoluteJoint*)m_world.CreateJoint(jd);
        }

        const auto wheelAnchor = pivot + Vec2(0.0f, -0.8f) * 1_m;

        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel->SetTransform(m_wheel->GetLocation(), 120_deg);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel->SetTransform(m_wheel->GetLocation(), -120_deg);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);
        
        SetAccelerations(m_world, m_gravity);

        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Left", [&](KeyActionMods) {
            m_motorJoint->SetMotorSpeed(-m_motorSpeed);
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Brake", [&](KeyActionMods) {
            m_motorJoint->SetMotorSpeed(0_rad / 1_s);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Right", [&](KeyActionMods) {
            m_motorJoint->SetMotorSpeed(m_motorSpeed);
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Toggle Motor", [&](KeyActionMods) {
            m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
        });
    }

    Length2 m_offset;
    Body* m_chassis;
    Body* m_wheel;
    RevoluteJoint* m_motorJoint;
    bool m_motorOn;
    AngularVelocity m_motorSpeed;
};

} // namespace testbed

#endif // THEO_JANSEN_HPP
