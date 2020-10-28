/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_GEARS_HPP
#define  PLAYRHO_GEARS_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class Gears : public Test
{
public:
    Gears()
    {
        const auto ground = CreateBody(GetWorld());
        CreateFixture(GetWorld(), ground, Shape{EdgeShapeConf{
            Vec2(50.0f, 0.0f) * 1_m, Vec2(-50.0f, 0.0f) * 1_m}});

        const auto circle1 = DiskShapeConf{}.UseRadius(1_m).UseDensity(5_kgpm2);
        const auto circle2 = DiskShapeConf{}.UseRadius(2_m).UseDensity(5_kgpm2);
        const auto box = Shape{PolygonShapeConf{}.SetAsBox(0.5_m, 5_m).UseDensity(5_kgpm2)};
    
        {
            auto bd1 = BodyConf{};
            bd1.type = BodyType::Static;
            bd1.location = Vec2(10.0f, 9.0f) * 1_m;
            const auto body1 = CreateBody(GetWorld(), bd1);

            auto bd2 = BodyConf{};
            bd2.type = BodyType::Dynamic;
            bd2.location = Vec2(10.0f, 8.0f) * 1_m;
            const auto body2 = CreateBody(GetWorld(), bd2);
            CreateFixture(GetWorld(), body2, box);

            auto bd3 = BodyConf{};
            bd3.type = BodyType::Dynamic;
            bd3.location = Vec2(10.0f, 6.0f) * 1_m;
            const auto body3 = CreateBody(GetWorld(), bd3);
            CreateFixture(GetWorld(), body3, Shape{circle2});

            auto joint1 = CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), body2, body1, bd1.location));
            auto joint2 = CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), body2, body3, bd3.location));

            auto jd4 = GetGearJointConf(GetWorld(), joint1, joint2);
            jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
            CreateJoint(GetWorld(), jd4);
        }

        {
            auto bd1 = BodyConf{};
            bd1.type = BodyType::Dynamic;
            bd1.location = Vec2(-3.0f, 12.0f) * 1_m;
            const auto body1 = CreateBody(GetWorld(), bd1);
            CreateFixture(GetWorld(), body1, Shape{circle1});

            auto jd1 = RevoluteJointConf{};
            jd1.bodyA = ground;
            jd1.bodyB = body1;
            jd1.localAnchorA = GetLocalPoint(GetWorld(), ground, bd1.location);
            jd1.localAnchorB = GetLocalPoint(GetWorld(), body1, bd1.location);
            jd1.referenceAngle = GetAngle(GetWorld(), body1) - GetAngle(GetWorld(), ground);
            m_joint1 = CreateJoint(GetWorld(), jd1);

            auto bd2 = BodyConf{};
            bd2.type = BodyType::Dynamic;
            bd2.location = Vec2(0.0f, 12.0f) * 1_m;
            const auto body2 = CreateBody(GetWorld(), bd2);
            CreateFixture(GetWorld(), body2, Shape{circle2});

            auto jd2 = GetRevoluteJointConf(GetWorld(), ground, body2, bd2.location);
            m_joint2 = CreateJoint(GetWorld(), jd2);

            auto bd3 = BodyConf{};
            bd3.type = BodyType::Dynamic;
            bd3.location = Vec2(2.5f, 12.0f) * 1_m;
            const auto body3 = CreateBody(GetWorld(), bd3);
            CreateFixture(GetWorld(), body3, box);

            auto jd3 = GetPrismaticJointConf(GetWorld(), ground, body3, bd3.location,
                                             UnitVec::GetTop());
            jd3.lowerTranslation = -5_m;
            jd3.upperTranslation = 5_m;
            jd3.enableLimit = true;

            m_joint3 = CreateJoint(GetWorld(), jd3);

            auto jd4 = GetGearJointConf(GetWorld(), m_joint1, m_joint2);
            jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
            m_joint4 = CreateJoint(GetWorld(), jd4);

            auto jd5 = GetGearJointConf(GetWorld(), m_joint2, m_joint3);
            jd5.ratio = -1.0f / (circle2.GetRadius() / 1_m);
            m_joint5 = CreateJoint(GetWorld(), jd5);
        }
        
        SetAccelerations(GetWorld(), GetGravity());
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        {
            const auto ratio = GetRatio(GetWorld(), m_joint4);
            const auto angle = GetAngle(GetWorld(), m_joint1) + ratio * GetAngle(GetWorld(), m_joint2);
            stream << "Theta1 + " << static_cast<double>(ratio);
            stream << " * theta2 = " << static_cast<double>(Real{angle / 1_rad});
            stream << " rad.\n";
        }
        {
            const auto ratio = GetRatio(GetWorld(), m_joint5);
            const auto value = ratio * GetJointTranslation(GetWorld(), m_joint3);
            stream << "Theta2 + " << static_cast<double>(ratio);
            stream << " * theta2 = " << static_cast<double>(Real{value / 1_m});
            stream << " m.";
        }
        SetStatus(stream.str());
    }

    JointID m_joint1; // Revolute joint.
    JointID m_joint2; // Revolute joint.
    JointID m_joint3; // Prismatic joint.
    JointID m_joint4; // Gear joint.
    JointID m_joint5; // Gear joint.
};

} // namespace testbed

#endif
