/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "../Framework/Test.hpp"

namespace testbed {

class TheoJansen : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Theo Jansen's Walker", MakeUniqueTest<TheoJansen>);

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
        if (s > 0.0f) {
            poly1.Set({p1, p2, p3});
            poly2.Set({Length2{}, p5 - p4, p6 - p4});
        }
        else {
            poly1.Set({p1, p3, p2});
            poly2.Set({Length2{}, p6 - p4, p5 - p4});
        }
        Filter filter;
        filter.groupIndex = -1;
        poly1.UseDensity(1_kgpm2);
        poly1.UseFilter(filter);
        poly2.UseDensity(1_kgpm2);
        poly2.UseFilter(filter);

        BodyConf bd1, bd2;
        bd1.type = BodyType::Dynamic;
        bd2.type = BodyType::Dynamic;
        bd1.location = m_offset;
        bd2.location = p4 + m_offset;

        bd1.angularDamping = 10_Hz;
        bd2.angularDamping = 10_Hz;

        const auto body1 = CreateBody(GetWorld(), bd1);
        const auto body2 = CreateBody(GetWorld(), bd2);

        Attach(GetWorld(), body1, CreateShape(GetWorld(), poly1));
        Attach(GetWorld(), body2, CreateShape(GetWorld(), poly2));

        // Using a soft distance constraint can reduce some jitter.
        // It also makes the structure seem a bit more fluid by
        // acting like a suspension system.

        CreateJoint(GetWorld(),
                    GetDistanceJointConf(GetWorld(), body1, body2, p2 + m_offset, p5 + m_offset)
                        .UseFrequency(10_Hz)
                        .UseDampingRatio(Real(0.5)));
        CreateJoint(GetWorld(),
                    GetDistanceJointConf(GetWorld(), body1, body2, p3 + m_offset, p4 + m_offset)
                        .UseFrequency(10_Hz)
                        .UseDampingRatio(Real(0.5)));
        CreateJoint(GetWorld(), GetDistanceJointConf(GetWorld(), body1, m_wheel, p3 + m_offset,
                                                     wheelAnchor + m_offset)
                                    .UseFrequency(10_Hz)
                                    .UseDampingRatio(Real(0.5)));
        CreateJoint(GetWorld(), GetDistanceJointConf(GetWorld(), body2, m_wheel, p6 + m_offset,
                                                     wheelAnchor + m_offset)
                                    .UseFrequency(10_Hz)
                                    .UseDampingRatio(Real(0.5)));
        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), body2, m_chassis, p4 + m_offset));
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
            const auto ground = CreateBody(GetWorld(), bd);

            auto conf = EdgeShapeConf{};

            conf.Set(Vec2(-50.0f, 0.0f) * 1_m, Vec2(50.0f, 0.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));

            conf.Set(Vec2(-50.0f, 0.0f) * 1_m, Vec2(-50.0f, 10.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));

            conf.Set(Vec2(50.0f, 0.0f) * 1_m, Vec2(50.0f, 10.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
        }

        // Balls
        auto circleConf = DiskShapeConf{};
        circleConf.vertexRadius = 0.25_m;
        circleConf.density = 1_kgpm2;
        const auto circle = CreateShape(GetWorld(), circleConf);
        for (auto i = 0; i < 40; ++i) {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-40.0f + 2.0f * i, 0.5f) * 1_m;

            const auto body = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), body, circle);
        }

        // Chassis
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = pivot + m_offset;
            m_chassis = CreateBody(GetWorld(), bd);
            auto conf = PolygonShapeConf{};
            conf.density = 1_kgpm2;
            conf.filter.groupIndex = -1;
            conf.SetAsBox(2.5_m, 1_m);
            Attach(GetWorld(), m_chassis, CreateShape(GetWorld(), conf));
        }

        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = pivot + m_offset;
            m_wheel = CreateBody(GetWorld(), bd);
            auto conf = DiskShapeConf{};
            conf.vertexRadius = 1.6_m;
            conf.density = 1_kgpm2;
            conf.filter.groupIndex = -1;
            Attach(GetWorld(), m_wheel, CreateShape(GetWorld(), conf));
        }

        {
            auto jd = GetRevoluteJointConf(GetWorld(), m_wheel, m_chassis, pivot + m_offset);
            jd.collideConnected = false;
            jd.motorSpeed = m_motorSpeed;
            jd.maxMotorTorque = 400_Nm;
            jd.enableMotor = m_motorOn;
            m_motorJoint = CreateJoint(GetWorld(), jd);
        }

        const auto wheelAnchor = pivot + Vec2(0.0f, -0.8f) * 1_m;

        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        SetTransform(GetWorld(), m_wheel, GetLocation(GetWorld(), m_wheel), 120_deg);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        SetTransform(GetWorld(), m_wheel, GetLocation(GetWorld(), m_wheel), -120_deg);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        SetAccelerations(GetWorld(), GetGravity());

        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Left", [&](KeyActionMods) {
            SetMotorSpeed(GetWorld(), m_motorJoint, -m_motorSpeed);
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Brake", [&](KeyActionMods) {
            SetMotorSpeed(GetWorld(), m_motorJoint, 0_rad / 1_s);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Right", [&](KeyActionMods) {
            SetMotorSpeed(GetWorld(), m_motorJoint, m_motorSpeed);
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Toggle Motor", [&](KeyActionMods) {
            EnableMotor(GetWorld(), m_motorJoint, !IsMotorEnabled(GetWorld(), m_motorJoint));
        });
    }

    Length2 m_offset;
    BodyID m_chassis;
    BodyID m_wheel;
    JointID m_motorJoint; // RevoluteJoint
    bool m_motorOn;
    AngularVelocity m_motorSpeed;
};

} // namespace testbed
