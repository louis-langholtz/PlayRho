/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
        conf.description = "Demonstrates all of the different Joint subclasses "
            "offering a comparative overview of them all at once.";
        return conf;
    }
    
    using DiskConf = DiskShapeConf;
    using PolyConf = PolygonShapeConf;

    JointsTest(): Test(GetTestConf())
    {
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
                SetTarget(m_world, m_lftTargetJoint, mouseWorld);
            }
            if (m_rgtTargetJoint != InvalidJointID)
            {
                SetTarget(m_world, m_rgtTargetJoint, mouseWorld);
            }
        }
    }

    BodyID SetupContainer(Length2 center)
    {
        const auto conf = GetChainShapeConf(Length2{ColumnSize, RowSize});
        const auto b = CreateBody(m_world);
        CreateFixture(m_world, b, Shape{conf});
        SetLocation(m_world, b, center);
        return b;
    }

    void SetupRevoluteJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(m_world, BodyConf(StaticBD).UseLocation(center - offset));
        CreateFixture(m_world, fb, m_rectShape);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offset));
        CreateFixture(m_world, mb, m_rectShape);
        auto jd = GetRevoluteJointConf(m_world, fb, mb, center);
        m_revoluteJoint = m_world.CreateJoint(jd);
        SetupContainer(center);
    }

    void SetupPrismaticJoint(Length2 center)
    {
        const auto offs = Length2{3.5_m, 3.5_m};
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(m_world, BodyConf(StaticBD).UseLocation(center));
        CreateFixture(m_world, fb, m_diskShape);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offs));
        CreateFixture(m_world, mb, m_squareShape);
        auto jd = GetPrismaticJointConf(m_world, fb, mb, center, UnitVec::GetTopRight())
            .UseEnableLimit(true)
            .UseLowerTranslation(-9_m)
            .UseUpperTranslation(+0_m);
        m_prismaticJoint = m_world.CreateJoint(jd);
        SetupContainer(center);
    }
    
    void SetupDistanceJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = CreateBody(m_world, BodyConf(StaticBD).UseLocation(center));
        CreateFixture(m_world, fb, m_diskShape);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offset));
        CreateFixture(m_world, mb, m_squareShape);
        auto jd = GetDistanceJointConf(m_world, fb, mb, center, center + offset).UseLength(2_m);
        m_distanceJoint = m_world.CreateJoint(jd);
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
            CreateFixture(m_world, cbody, Shape{DiskConf(conf).UseLocation(left)});
            CreateFixture(m_world, cbody, Shape{DiskConf(conf).UseLocation(right)});
        }
        {
            const auto shape = Shape{PolyConf{}.UseDensity(5_kgpm2).SetAsBox(0.5_m, 0.5_m)};
            const auto ganchor1 = center + left;
            const auto ganchor2 = center + right;
            const auto anchor1 = ganchor1 - Length2{0_m, 1.5_m};
            const auto anchor2 = ganchor2 - Length2{0_m, 5_m};

            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto body1 = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(anchor1));
            CreateFixture(m_world, body1, shape);
            const auto body2 = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(anchor2));
            CreateFixture(m_world, body2, shape);
            
            const auto pulleyConf = GetPulleyJointConf(m_world, body1, body2, ganchor1, ganchor2,
                anchor1, anchor2).UseRatio(1.3f);
            m_pulleyJoint = m_world.CreateJoint(pulleyConf);
        }
    }
    
    void SetupGearJoint(Length2 center)
    {
        const auto containerBody = SetupContainer(center);

        const auto sr = GetVertexRadius(m_smallDiskShape, 0);
        const auto nr = GetVertexRadius(m_diskShape, 0);
        const auto tr = sr + nr;
        const auto bd1 = BodyConf(DynamicBD).UseLocation(center - Length2{tr, 0_m});
        const auto body1 = CreateBody(m_world, bd1);
        CreateFixture(m_world, body1, m_smallDiskShape);
        
        auto jd1 = RevoluteJointConf{};
        jd1.bodyA = containerBody;
        jd1.bodyB = body1;
        jd1.localAnchorA = GetLocalPoint(m_world, containerBody, bd1.location);
        jd1.localAnchorB = GetLocalPoint(m_world, body1, bd1.location);
        jd1.referenceAngle = GetAngle(m_world, body1) - GetAngle(m_world, containerBody);
        const auto joint1 = m_world.CreateJoint(jd1);
        
        const auto bd2 = BodyConf(DynamicBD).UseLocation(center);
        const auto body2 = CreateBody(m_world, bd2);
        CreateFixture(m_world, body2, m_diskShape);
        
        const auto jd2 = GetRevoluteJointConf(m_world, containerBody, body2, bd2.location);
        const auto joint2 = m_world.CreateJoint(jd2);
        
        auto bd3 = BodyConf(DynamicBD)
            .UseLocation(center + Length2{nr + RectHHeight, RectHWidth})
            .UseAngle(Pi * 1_rad / 2);
        const auto body3 = CreateBody(m_world, bd3);
        CreateFixture(m_world, body3, m_rectShape);
        
        auto jd3 = GetPrismaticJointConf(m_world, containerBody, body3, bd3.location,
                                         UnitVec::GetTop());
        jd3.upperTranslation = +0_m;
        jd3.lowerTranslation = -3.6_m;
        jd3.enableLimit = true;
        
        const auto joint3 = m_world.CreateJoint(jd3);
        
        auto jd4 = GetGearJointConf(m_world, joint1, joint2);
        jd4.ratio = GetVertexRadius(m_diskShape, 0) / GetVertexRadius(m_smallDiskShape, 0);
        m_gearJoint0 = m_world.CreateJoint(jd4);
        
        auto jd5 = GetGearJointConf(m_world, joint2, joint3);
        jd5.ratio = -1.0f / (GetVertexRadius(m_diskShape, 0) / 1_m);
        m_gearJoint1 = m_world.CreateJoint(jd5);
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
        const auto circle = Shape{
            DiskShapeConf{}.UseDensity(1_kgpm2).UseFriction(Real(0.9)).UseRadius(0.4_m)};
        
        const auto carLocation = center - Vec2(3.3f, 1.0f) * 1_m;
        const auto car = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(carLocation));
        CreateFixture(m_world, car, Shape{
            PolygonShapeConf{}.UseDensity(1_kgpm2).Set(carVerts)
        });
        
        const auto backWheel  = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(carLocation + Vec2(-1.0f, -0.65f) * 1_m));
        CreateFixture(m_world, backWheel, circle);
        
        const auto frontWheel = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(carLocation + Vec2(+1.0f, -0.65f) * 1_m));
        CreateFixture(m_world, frontWheel, circle);
        
        const auto frequency = 4_Hz;
        const auto dampingRatio = 0.7f;
        const auto motorSpeed = -2 * 1_rad / 1_s;
        const auto axis = UnitVec::GetTop();
        {
            auto jd = GetWheelJointConf(m_world, car, backWheel,
                                        GetLocation(m_world, backWheel), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint0 = m_world.CreateJoint(jd);
        }
        {
            auto jd = GetWheelJointConf(m_world, car, frontWheel,
                                        GetLocation(m_world, frontWheel), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint1 = m_world.CreateJoint(jd);
        }
    }

    void SetupWeldJoint(Length2 center)
    {
        const auto offs = Length2{RectHWidth, 0_m};
        const auto containerBody = SetupContainer(center);
        const auto fb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center - offs));
        CreateFixture(m_world, fb, m_rectShape);
        const auto jd0 = GetWeldJointConf(m_world, containerBody, fb, center - 2 * offs)
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint0 = m_world.CreateJoint(jd0);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offs));
        CreateFixture(m_world, mb, m_rectShape);
        const auto jd1 = GetWeldJointConf(m_world, fb, mb, center)
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint1 = m_world.CreateJoint(jd1);
    }
    
    void SetupFrictionJoint(Length2 center)
    {
        const auto fb = CreateBody(m_world, BodyConf(StaticBD).UseLocation(center));
        CreateFixture(m_world, fb, m_diskShape);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offset));
        CreateFixture(m_world, mb, m_squareShape);
        auto jd = GetFrictionJointConf(m_world, fb, mb, center)
            .UseMaxForce(20_N).UseMaxTorque(12_Nm);
        m_frictionJoint = m_world.CreateJoint(jd);
        SetupContainer(center);
    }
    
    void SetupRopeJoint(Length2 center)
    {
        const auto fb = CreateBody(m_world, BodyConf(StaticBD).UseLocation(center));
        CreateFixture(m_world, fb, m_diskShape);
        const auto mb = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center + offset));
        CreateFixture(m_world, mb, m_squareShape);
        auto jd = RopeJointConf{fb, mb};
        jd.localAnchorA = Length2{};
        jd.localAnchorB = Length2{};
        jd.maxLength = 3_m;
        m_ropeJoint = m_world.CreateJoint(jd);
        SetupContainer(center);
    }
    
    void SetupTargetJoint(Length2 ctr)
    {
        SetupContainer(ctr);

        const auto lftOffs = Length2{-2_m, 0.8_m};
        const auto rgtOffs = Length2{+2_m, 0.8_m};

        {
            const auto lftEye = CreateBody(m_world, BodyConf(StaticBD).UseLocation(ctr + lftOffs));
            const auto rgtEye = CreateBody(m_world, BodyConf(StaticBD).UseLocation(ctr + rgtOffs));
            auto cconf = ChainShapeConf{};
            cconf.restitution = 0;
            cconf.friction = 0;
            cconf.Set(GetCircleVertices(1.8_m, 24, 0_deg, 1));
            const auto eyeEnc = Shape(cconf);
            CreateFixture(m_world, lftEye, eyeEnc);
            CreateFixture(m_world, rgtEye, eyeEnc);
        }

        const auto lftPup = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(ctr + lftOffs));
        const auto rgtPup = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(ctr + rgtOffs));
        CreateFixture(m_world, lftPup, m_smallDiskShape);
        CreateFixture(m_world, rgtPup, m_smallDiskShape);
        
        // Remove gravity on dynamic bodies...
        SetAcceleration(m_world, lftPup, LinearAcceleration2{}, AngularAcceleration{});
        SetAcceleration(m_world, rgtPup, LinearAcceleration2{}, AngularAcceleration{});
        m_lftTargetJoint = m_world.CreateJoint(TargetJointConf{lftPup}
                                               .UseMaxForce(200_N)
                                               .UseFrequency(2_Hz)
                                               .UseTarget(GetLocation(m_world, lftPup)));
        m_rgtTargetJoint = m_world.CreateJoint(TargetJointConf{rgtPup}
                                               .UseMaxForce(200_N)
                                               .UseFrequency(2_Hz)
                                               .UseTarget(GetLocation(m_world, rgtPup)));
    }

    void SetupMotorJoint(Length2 center)
    {
        m_motorJointCenter = center;
        const auto containerBody = SetupContainer(center);
        const auto movingBody = CreateBody(m_world, BodyConf(DynamicBD).UseLocation(center));
        CreateFixture(m_world, movingBody, m_rectShape);

        auto jd = GetMotorJointConf(m_world, containerBody, movingBody)
            .UseMaxForce(1000_N).UseMaxTorque(1000_Nm);
        m_motorJoint = m_world.CreateJoint(jd);
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
            SetLinearOffset(m_world, m_motorJoint, linearOffset);
            SetAngularOffset(m_world, m_motorJoint, angularOffset);
            drawer.DrawPoint(m_motorJointCenter + linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
        }
    }
    
    const Length RowSize = +10_m;
    const Length ColumnSize = +10_m;
    const BodyConf StaticBD = BodyConf{}.UseType(BodyType::Static);
    const BodyConf DynamicBD = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(m_gravity);
    const Length RectHHeight = 0.25_m;
    const Length RectHWidth = 2_m;
    Shape m_diskShape{DiskConf{}.UseRadius(1_m).UseDensity(1_kgpm2)};
    Shape m_smallDiskShape{DiskConf{}.UseRadius(0.5_m).UseDensity(1_kgpm2).UseRestitution(0)};
    Shape m_squareShape{PolyConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m)};
    Shape m_rectShape{PolyConf{}.UseDensity(1_kgpm2).SetAsBox(RectHWidth, RectHHeight)};
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

