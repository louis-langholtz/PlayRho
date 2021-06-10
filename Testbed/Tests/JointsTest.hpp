/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef JointsTest_hpp
#define JointsTest_hpp

#include "../Framework/Test.hpp"

namespace testbed {

class JointsTest: public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Demonstrates all of the different Joint types "
            "offering a comparative overview of them all at once.";
        return conf;
    }
    
    using DiskConf = DiskShapeConf;
    using PolyConf = PolygonShapeConf;

    JointsTest(): Test(GetTestConf())
    {
        m_rectShape = CreateShape(GetWorld(),
                                  PolyConf{}.UseDensity(1_kgpm2).SetAsBox(RectHWidth, RectHHeight));
        m_diskShape = CreateShape(GetWorld(), DiskConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
        m_squareShape = CreateShape(GetWorld(),
                                    PolyConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m));
        m_smallDiskShape = CreateShape(GetWorld(),
                                       DiskConf{}.UseRadius(0.5_m).UseDensity(1_kgpm2).UseRestitution(0));

        // Eleven joint types. Arrange demos in a 4 column by 3 row layout.
         const auto columnStart = -1.5f * ColumnSize;

        // Row one...
        auto centerY = Length{20_m + RowSize};
        {
            auto centerX = columnStart;
            SetupDistanceJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupFrictionJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupGearJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupMotorJoint(Length2{centerX, centerY});
        }
        
        // Row two...
        centerY -= RowSize;
        {
            auto centerX = columnStart;
            SetupTargetJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupPrismaticJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupPulleyJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupRevoluteJoint(Length2{centerX, centerY});
        }

        // Row three...
        centerY -= RowSize;
        {
            auto centerX = columnStart;
            SetupRopeJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupWeldJoint(Length2{centerX, centerY});
            centerX += ColumnSize;
            SetupWheelJoint(Length2{centerX, centerY});
        }
    }
    
private:
    void PostStep(const Settings&, Drawer& drawer) override
    {
        const auto startLoc = Length2{-1.5f * ColumnSize, 21_m + 0.5f * RowSize};
        {
            auto loc = startLoc - Length2{0_m, 0 * RowSize};
            drawer.DrawString(loc, Drawer::Center, "DistanceJoint (fixed length)");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "FrictionJoint (dampened point & angle)");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "GearJoint");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "MotorJoint");
        }
        {
            auto loc = startLoc - Length2{0_m, 1 * RowSize};
            drawer.DrawString(loc, Drawer::Center, "TargetJoint");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "PrismaticJoint (fixed line)");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "PulleyJoint");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "RevoluteJoint (fixed point)");
        }
        {
            auto loc = startLoc - Length2{0_m, 2 * RowSize};
            drawer.DrawString(loc, Drawer::Center, "RopeJoint (fixed max length)");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "WeldJoint (fixed point & angle)");
            GetX(loc) += ColumnSize;
            drawer.DrawString(loc, Drawer::Center, "WheelJoint");
            // empty
        }
        {
            // For the target joints...
            const auto mouseWorld = GetMouseWorld();
            if (m_lftTargetJoint != InvalidJointID)
            {
                SetTarget(GetWorld(), m_lftTargetJoint, mouseWorld);
            }
            if (m_rgtTargetJoint != InvalidJointID)
            {
                SetTarget(GetWorld(), m_rgtTargetJoint, mouseWorld);
            }
        }
    }

    BodyID SetupContainer(Length2 center)
    {
        const auto conf = GetChainShapeConf(Length2{ColumnSize, RowSize});
        const auto b = CreateBody(GetWorld());
        Attach(GetWorld(), b, Shape{conf});
        SetLocation(GetWorld(), b, center);
        return b;
    }

    void SetupRevoluteJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(center - offset));
        Attach(GetWorld(), fb, m_rectShape);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offset));
        Attach(GetWorld(), mb, m_rectShape);
        auto jd = GetRevoluteJointConf(GetWorld(), fb, mb, center);
        m_revoluteJoint = CreateJoint(GetWorld(), jd);
        SetupContainer(center);
    }

    void SetupPrismaticJoint(Length2 center)
    {
        const auto offs = Length2{3.5_m, 3.5_m};
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(center));
        Attach(GetWorld(), fb, m_diskShape);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offs));
        Attach(GetWorld(), mb, m_squareShape);
        auto jd = GetPrismaticJointConf(GetWorld(), fb, mb, center, UnitVec::GetTopRight())
            .UseEnableLimit(true)
            .UseLowerLength(-9_m)
            .UseUpperLength(+0_m);
        m_prismaticJoint = CreateJoint(GetWorld(), jd);
        SetupContainer(center);
    }
    
    void SetupDistanceJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(center));
        Attach(GetWorld(), fb, m_diskShape);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offset));
        Attach(GetWorld(), mb, m_squareShape);
        auto jd = GetDistanceJointConf(GetWorld(), fb, mb, center, center + offset).UseLength(2_m);
        m_distanceJoint = CreateJoint(GetWorld(), jd);
        SetupContainer(center);
    }
    
    void SetupPulleyJoint(Length2 center)
    {
        const auto cbody = SetupContainer(center);
        const auto left  = Length2{-2_m, +2.5_m};
        const auto right = Length2{+2_m, +2.5_m};
        {
            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto conf = DiskConf{}.UseRadius(0.7_m);
            Attach(GetWorld(), cbody, Shape{DiskConf(conf).UseLocation(left)});
            Attach(GetWorld(), cbody, Shape{DiskConf(conf).UseLocation(right)});
        }
        {
            const auto shape = CreateShape(GetWorld(),
                                           PolyConf{}.UseDensity(5_kgpm2).SetAsBox(0.5_m, 0.5_m));
            const auto ganchor1 = center + left;
            const auto ganchor2 = center + right;
            const auto anchor1 = ganchor1 - Length2{0_m, 1.5_m};
            const auto anchor2 = ganchor2 - Length2{0_m, 5_m};

            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto body1 = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(anchor1));
            Attach(GetWorld(), body1, shape);
            const auto body2 = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(anchor2));
            Attach(GetWorld(), body2, shape);
            
            const auto pulleyConf = GetPulleyJointConf(GetWorld(), body1, body2, ganchor1, ganchor2,
                                                       anchor1, anchor2).UseRatio(1.3f);
            m_pulleyJoint = CreateJoint(GetWorld(), pulleyConf);
        }
    }
    
    void SetupGearJoint(Length2 center)
    {
        const auto containerBody = SetupContainer(center);

        const auto sr = GetVertexRadius(GetShape(GetWorld(), m_smallDiskShape), 0);
        const auto nr = GetVertexRadius(GetShape(GetWorld(), m_diskShape), 0);
        const auto tr = sr + nr;
        const auto bd1 = BodyConf(DynamicBD).UseLocation(center - Length2{tr, 0_m});
        const auto body1 = CreateBody(GetWorld(), bd1);
        Attach(GetWorld(), body1, m_smallDiskShape);
        
        auto jd1 = RevoluteJointConf{};
        jd1.bodyA = containerBody;
        jd1.bodyB = body1;
        jd1.localAnchorA = GetLocalPoint(GetWorld(), containerBody, bd1.location);
        jd1.localAnchorB = GetLocalPoint(GetWorld(), body1, bd1.location);
        jd1.referenceAngle = GetAngle(GetWorld(), body1) - GetAngle(GetWorld(), containerBody);
        const auto joint1 = CreateJoint(GetWorld(), jd1);
        
        const auto bd2 = BodyConf(DynamicBD).UseLocation(center);
        const auto body2 = CreateBody(GetWorld(), bd2);
        Attach(GetWorld(), body2, m_diskShape);
        
        const auto jd2 = GetRevoluteJointConf(GetWorld(), containerBody, body2, bd2.location);
        const auto joint2 = CreateJoint(GetWorld(), jd2);
        
        auto bd3 = BodyConf(DynamicBD)
            .UseLocation(center + Length2{nr + RectHHeight, RectHWidth})
            .UseAngle(Pi * 1_rad / 2);
        const auto body3 = CreateBody(GetWorld(), bd3);
        Attach(GetWorld(), body3, m_rectShape);
        
        auto jd3 = GetPrismaticJointConf(GetWorld(), containerBody, body3, bd3.location,
                                         UnitVec::GetTop());
        jd3.upperTranslation = +0_m;
        jd3.lowerTranslation = -3.6_m;
        jd3.enableLimit = true;
        
        const auto joint3 = CreateJoint(GetWorld(), jd3);
        
        auto jd4 = GetGearJointConf(GetWorld(), joint1, joint2);
        jd4.ratio = GetVertexRadius(GetShape(GetWorld(), m_diskShape), 0) / GetVertexRadius(GetShape(GetWorld(), m_smallDiskShape), 0);
        m_gearJoint0 = CreateJoint(GetWorld(), jd4);
        
        auto jd5 = GetGearJointConf(GetWorld(), joint2, joint3);
        jd5.ratio = -1.0f / (GetVertexRadius(GetShape(GetWorld(), m_diskShape), 0) / 1_m);
        m_gearJoint1 = CreateJoint(GetWorld(), jd5);
    }

    void SetupWheelJoint(Length2 center)
    {
        SetupContainer(center);
        
        // Vertices from car Testbed code.
        const auto carVerts = std::vector<Length2>({
            Vec2(-1.5f, -0.5f) * 1_m,
            Vec2(1.5f, -0.5f) * 1_m,
            Vec2(1.5f, 0.0f) * 1_m,
            Vec2(0.0f, 0.9f) * 1_m,
            Vec2(-1.15f, 0.9f) * 1_m,
            Vec2(-1.5f, 0.2f) * 1_m
        });
        const auto circle = CreateShape(GetWorld(),
            DiskShapeConf{}.UseDensity(1_kgpm2).UseFriction(Real(0.9)).UseRadius(0.4_m));
        
        const auto carLocation = center - Vec2(3.3f, 1.0f) * 1_m;
        const auto car = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(carLocation));
        Attach(GetWorld(), car, Shape{
            PolygonShapeConf{}.UseDensity(1_kgpm2).Set(carVerts)
        });
        
        const auto backWheel  = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(carLocation + Vec2(-1.0f, -0.65f) * 1_m));
        Attach(GetWorld(), backWheel, circle);
        
        const auto frontWheel = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(carLocation + Vec2(+1.0f, -0.65f) * 1_m));
        Attach(GetWorld(), frontWheel, circle);
        
        const auto frequency = 4_Hz;
        const auto dampingRatio = 0.7f;
        const auto motorSpeed = -2 * 1_rad / 1_s;
        const auto axis = UnitVec::GetTop();
        {
            auto jd = GetWheelJointConf(GetWorld(), car, backWheel,
                                        GetLocation(GetWorld(), backWheel), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint0 = CreateJoint(GetWorld(), jd);
        }
        {
            auto jd = GetWheelJointConf(GetWorld(), car, frontWheel,
                                        GetLocation(GetWorld(), frontWheel), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint1 = CreateJoint(GetWorld(), jd);
        }
    }

    void SetupWeldJoint(Length2 center)
    {
        const auto offs = Length2{RectHWidth, 0_m};
        const auto containerBody = SetupContainer(center);
        const auto fb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center - offs));
        Attach(GetWorld(), fb, m_rectShape);
        const auto jd0 = GetWeldJointConf(GetWorld(), containerBody, fb, center - 2 * offs)
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint0 = CreateJoint(GetWorld(), jd0);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offs));
        Attach(GetWorld(), mb, m_rectShape);
        const auto jd1 = GetWeldJointConf(GetWorld(), fb, mb, center)
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint1 = CreateJoint(GetWorld(), jd1);
    }
    
    void SetupFrictionJoint(Length2 center)
    {
        const auto fb = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(center));
        Attach(GetWorld(), fb, m_diskShape);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offset));
        Attach(GetWorld(), mb, m_squareShape);
        auto jd = GetFrictionJointConf(GetWorld(), fb, mb, center)
            .UseMaxForce(20_N).UseMaxTorque(12_Nm);
        m_frictionJoint = CreateJoint(GetWorld(), jd);
        SetupContainer(center);
    }
    
    void SetupRopeJoint(Length2 center)
    {
        const auto fb = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(center));
        Attach(GetWorld(), fb, m_diskShape);
        const auto mb = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center + offset));
        Attach(GetWorld(), mb, m_squareShape);
        auto jd = RopeJointConf{fb, mb};
        jd.localAnchorA = Length2{};
        jd.localAnchorB = Length2{};
        jd.maxLength = 3_m;
        m_ropeJoint = CreateJoint(GetWorld(), jd);
        SetupContainer(center);
    }
    
    void SetupTargetJoint(Length2 ctr)
    {
        SetupContainer(ctr);

        const auto lftOffs = Length2{-2_m, 0.8_m};
        const auto rgtOffs = Length2{+2_m, 0.8_m};

        {
            const auto lftEye = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(ctr + lftOffs));
            const auto rgtEye = CreateBody(GetWorld(), BodyConf(StaticBD).UseLocation(ctr + rgtOffs));
            auto cconf = ChainShapeConf{};
            cconf.restitution = 0;
            cconf.friction = 0;
            cconf.Set(GetCircleVertices(1.8_m, 24, 0_deg, 1));
            const auto eyeEnc = CreateShape(GetWorld(), cconf);
            Attach(GetWorld(), lftEye, eyeEnc);
            Attach(GetWorld(), rgtEye, eyeEnc);
        }

        const auto lftPup = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(ctr + lftOffs));
        const auto rgtPup = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(ctr + rgtOffs));
        Attach(GetWorld(), lftPup, m_smallDiskShape);
        Attach(GetWorld(), rgtPup, m_smallDiskShape);
        
        // Remove gravity on dynamic bodies...
        SetAcceleration(GetWorld(), lftPup, LinearAcceleration2{}, AngularAcceleration{});
        SetAcceleration(GetWorld(), rgtPup, LinearAcceleration2{}, AngularAcceleration{});
        m_lftTargetJoint = CreateJoint(GetWorld(), TargetJointConf{lftPup}
                                               .UseMaxForce(200_N)
                                               .UseFrequency(2_Hz)
                                               .UseTarget(GetLocation(GetWorld(), lftPup)));
        m_rgtTargetJoint = CreateJoint(GetWorld(), TargetJointConf{rgtPup}
                                               .UseMaxForce(200_N)
                                               .UseFrequency(2_Hz)
                                               .UseTarget(GetLocation(GetWorld(), rgtPup)));
    }

    void SetupMotorJoint(Length2 center)
    {
        m_motorJointCenter = center;
        const auto containerBody = SetupContainer(center);
        const auto movingBody = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(center));
        Attach(GetWorld(), movingBody, m_rectShape);

        auto jd = GetMotorJointConf(GetWorld(), containerBody, movingBody)
            .UseMaxForce(1000_N).UseMaxTorque(1000_Nm);
        m_motorJoint = CreateJoint(GetWorld(), jd);
    }

    void PreStep(const Settings& settings, Drawer& drawer) override
    {
        if (settings.dt > 0)
        {
            // For any joints that need accumulated time...
            m_time += settings.dt;
        }

        {
            // For the motor joint...
            const auto linearOffset = Length2{
                static_cast<Real>(2.6 * std::sin(2 * m_time)) * 1_m,
                static_cast<Real>(2.0 * std::sin(1 * m_time)) * 1_m
            };
            const auto angularOffset = static_cast<Real>(4 * m_time) * 1_rad;
            SetLinearOffset(GetWorld(), m_motorJoint, linearOffset);
            SetAngularOffset(GetWorld(), m_motorJoint, angularOffset);
            drawer.DrawPoint(m_motorJointCenter + linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
        }
    }
    
    const Length RowSize = +10_m;
    const Length ColumnSize = +10_m;
    const BodyConf StaticBD = BodyConf{}.UseType(BodyType::Static);
    const BodyConf DynamicBD = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(GetGravity());
    const Length RectHHeight = 0.25_m;
    const Length RectHWidth = 2_m;
    ShapeID m_diskShape = InvalidShapeID;
    ShapeID m_smallDiskShape = InvalidShapeID;
    ShapeID m_squareShape = InvalidShapeID;
    ShapeID m_rectShape = InvalidShapeID;
    const Length2 offset = Length2{+2_m, 0_m};
    
    double m_time = 0;
    Length2 m_motorJointCenter = Length2{};
    JointID m_lftTargetJoint = InvalidJointID;
    JointID m_rgtTargetJoint = InvalidJointID;
    JointID m_motorJoint = InvalidJointID;
    JointID m_wheelJoint0 = InvalidJointID;
    JointID m_wheelJoint1 = InvalidJointID;
    JointID m_gearJoint0 = InvalidJointID;
    JointID m_gearJoint1 = InvalidJointID;
    JointID m_pulleyJoint = InvalidJointID;
    JointID m_revoluteJoint = InvalidJointID;
    JointID m_prismaticJoint = InvalidJointID;
    JointID m_distanceJoint = InvalidJointID;
    JointID m_weldJoint = InvalidJointID;
    JointID m_weldJoint0 = InvalidJointID;
    JointID m_weldJoint1 = InvalidJointID;
    JointID m_frictionJoint = InvalidJointID;
    JointID m_ropeJoint = InvalidJointID;
};

} // namespace testbed

#endif // JointsTest_hpp

