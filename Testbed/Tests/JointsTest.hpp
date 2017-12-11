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

namespace playrho {

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
            SetupMouseJoint(Length2{centerX, centerY});
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
            drawer.DrawString(loc, Drawer::Center, "MouseJoint");
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
            // For the mouse joints...
            const auto mouseWorld = GetMouseWorld();
            if (m_lftMouseJoint)
            {
                m_lftMouseJoint->SetTarget(mouseWorld);
            }
            if (m_rgtMouseJoint)
            {
                m_rgtMouseJoint->SetTarget(mouseWorld);
            }
        }
    }

    Body* SetupContainer(Length2 center)
    {
        const auto b = CreateRectangularEnclosingBody(m_world, Length2{ColumnSize, RowSize},
                                                      ShapeDef{});
        SetLocation(*b, center);
        return b;
    }

    void SetupRevoluteJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = m_world.CreateBody(BodyDef(StaticBD).UseLocation(center - offset));
        fb->CreateFixture(m_rectShape);
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offset));
        mb->CreateFixture(m_rectShape);
        auto jd = RevoluteJointDef{fb, mb, center};
        m_revoluteJoint = static_cast<RevoluteJoint*>(m_world.CreateJoint(jd));
        SetupContainer(center);
    }

    void SetupPrismaticJoint(Length2 center)
    {
        const auto offs = Length2{3.5_m, 3.5_m};
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = m_world.CreateBody(BodyDef(StaticBD).UseLocation(center));
        fb->CreateFixture(m_diskShape);
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offs));
        mb->CreateFixture(m_squareShape);
        auto jd = PrismaticJointDef{fb, mb, center, UnitVec2::GetTopRight()}
            .UseEnableLimit(true)
            .UseLowerTranslation(-9_m)
            .UseUpperTranslation(+0_m);
        m_prismaticJoint = static_cast<PrismaticJoint*>(m_world.CreateJoint(jd));
        SetupContainer(center);
    }
    
    void SetupDistanceJoint(Length2 center)
    {
        // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
        const auto fb = m_world.CreateBody(BodyDef(StaticBD).UseLocation(center));
        fb->CreateFixture(m_diskShape);
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offset));
        mb->CreateFixture(m_squareShape);
        auto jd = DistanceJointDef{fb, mb, center, center + offset}.UseLength(2_m);
        m_distanceJoint = static_cast<DistanceJoint*>(m_world.CreateJoint(jd));
        SetupContainer(center);
    }
    
    void SetupPulleyJoint(Length2 center)
    {
        const auto cbody = SetupContainer(center);
        const auto left  = Length2{-2_m, +2.5_m};
        const auto right = Length2{+2_m, +2.5_m};
        {
            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto conf = DiskConf{}.UseVertexRadius(0.7_m);
            cbody->CreateFixture(DiskConf(conf).UseLocation(left));
            cbody->CreateFixture(DiskConf(conf).UseLocation(right));
        }
        {
            const auto shape = Shape{PolyConf{}.UseDensity(5_kgpm2).SetAsBox(0.5_m, 0.5_m)};
            const auto ganchor1 = center + left;
            const auto ganchor2 = center + right;
            const auto anchor1 = ganchor1 - Length2{0_m, 1.5_m};
            const auto anchor2 = ganchor2 - Length2{0_m, 5_m};

            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto body1 = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(anchor1));
            body1->CreateFixture(shape);
            const auto body2 = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(anchor2));
            body2->CreateFixture(shape);
            
            const auto pulleyDef = PulleyJointDef{body1, body2, ganchor1, ganchor2,
                anchor1, anchor2}.UseRatio(1.3f);
            m_pulleyJoint = static_cast<PulleyJoint*>(m_world.CreateJoint(pulleyDef));
        }
    }
    
    void SetupGearJoint(Length2 center)
    {
        const auto containerBody = SetupContainer(center);

        const auto sr = GetVertexRadius(m_smallDiskShape);
        const auto nr = GetVertexRadius(m_diskShape);
        const auto tr = sr + nr;
        const auto bd1 = BodyDef(DynamicBD).UseLocation(center - Length2{tr, 0_m});
        const auto body1 = m_world.CreateBody(bd1);
        body1->CreateFixture(m_smallDiskShape);
        
        auto jd1 = RevoluteJointDef{};
        jd1.bodyA = containerBody;
        jd1.bodyB = body1;
        jd1.localAnchorA = GetLocalPoint(*containerBody, bd1.location);
        jd1.localAnchorB = GetLocalPoint(*body1, bd1.location);
        jd1.referenceAngle = body1->GetAngle() - containerBody->GetAngle();
        const auto joint1 = static_cast<RevoluteJoint*>(m_world.CreateJoint(jd1));
        
        const auto bd2 = BodyDef(DynamicBD).UseLocation(center);
        const auto body2 = m_world.CreateBody(bd2);
        body2->CreateFixture(m_diskShape);
        
        const auto jd2 = RevoluteJointDef{containerBody, body2, bd2.location};
        const auto joint2 = static_cast<RevoluteJoint*>(m_world.CreateJoint(jd2));
        
        auto bd3 = BodyDef(DynamicBD)
            .UseLocation(center + Length2{nr + RectHHeight, RectHWidth})
            .UseAngle(Pi * 1_rad / 2);
        const auto body3 = m_world.CreateBody(bd3);
        body3->CreateFixture(m_rectShape);
        
        auto jd3 = PrismaticJointDef{containerBody, body3, bd3.location, UnitVec2::GetTop()};
        jd3.upperTranslation = +0_m;
        jd3.lowerTranslation = -3.6_m;
        jd3.enableLimit = true;
        
        const auto joint3 = static_cast<PrismaticJoint*>(m_world.CreateJoint(jd3));
        
        auto jd4 = GearJointDef{joint1, joint2};
        jd4.ratio = GetVertexRadius(m_diskShape) / GetVertexRadius(m_smallDiskShape);
        m_gearJoint0 = static_cast<GearJoint*>(m_world.CreateJoint(jd4));
        
        auto jd5 = GearJointDef{joint2, joint3};
        jd5.ratio = -1.0f / (GetVertexRadius(m_diskShape) / 1_m);
        m_gearJoint1 = static_cast<GearJoint*>(m_world.CreateJoint(jd5));
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
            DiskShapeConf{}.SetDensity(1_kgpm2).SetFriction(Real(0.9f)).SetRadius(0.4_m)};
        
        const auto carLocation = center - Vec2(3.3f, 1.0f) * 1_m;
        const auto car = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(carLocation));
        car->CreateFixture(PolygonShapeConf{}.SetDensity(1_kgpm2).Set(Span<const Length2>(carVerts.data(), carVerts.size())));
        
        const auto backWheel  = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(carLocation + Vec2(-1.0f, -0.65f) * 1_m));
        backWheel->CreateFixture(circle);
        
        const auto frontWheel = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(carLocation + Vec2(+1.0f, -0.65f) * 1_m));
        frontWheel->CreateFixture(circle);
        
        const auto frequency = 4_Hz;
        const auto dampingRatio = 0.7f;
        const auto motorSpeed = -2 * 1_rad / 1_s;
        const auto axis = UnitVec2::GetTop();
        {
            auto jd = WheelJointDef(car, backWheel, backWheel->GetLocation(), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint0 = static_cast<WheelJoint*>(m_world.CreateJoint(jd));
        }
        {
            auto jd = WheelJointDef(car, frontWheel, frontWheel->GetLocation(), axis);
            jd.motorSpeed = motorSpeed;
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = true;
            jd.frequency = frequency;
            jd.dampingRatio = dampingRatio;
            m_wheelJoint1 = static_cast<WheelJoint*>(m_world.CreateJoint(jd));
        }
    }

    void SetupWeldJoint(Length2 center)
    {
        const auto offs = Length2{RectHWidth, 0_m};
        const auto containerBody = SetupContainer(center);
        const auto fb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center - offs));
        fb->CreateFixture(m_rectShape);
        const auto jd0 = WeldJointDef{containerBody, fb, center - 2 * offs}
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint0 = static_cast<WeldJoint*>(m_world.CreateJoint(jd0));
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offs));
        mb->CreateFixture(m_rectShape);
        const auto jd1 = WeldJointDef{fb, mb, center}
            .UseFrequency(5_Hz).UseDampingRatio(0.7f);
        m_weldJoint1 = static_cast<WeldJoint*>(m_world.CreateJoint(jd1));
    }
    
    void SetupFrictionJoint(Length2 center)
    {
        const auto fb = m_world.CreateBody(BodyDef(StaticBD).UseLocation(center));
        fb->CreateFixture(m_diskShape);
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offset));
        mb->CreateFixture(m_squareShape);
        auto jd = FrictionJointDef{fb, mb, center}
            .UseMaxForce(20_N).UseMaxTorque(12_Nm);
        m_frictionJoint = static_cast<FrictionJoint*>(m_world.CreateJoint(jd));
        SetupContainer(center);
    }
    
    void SetupRopeJoint(Length2 center)
    {
        const auto fb = m_world.CreateBody(BodyDef(StaticBD).UseLocation(center));
        fb->CreateFixture(m_diskShape);
        const auto mb = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center + offset));
        mb->CreateFixture(m_squareShape);
        auto jd = RopeJointDef{fb, mb};
        jd.localAnchorA = Length2{};
        jd.localAnchorB = Length2{};
        jd.maxLength = 3_m;
        m_ropeJoint = static_cast<RopeJoint*>(m_world.CreateJoint(jd));
        SetupContainer(center);
    }
    
    void SetupMouseJoint(Length2 ctr)
    {
        SetupContainer(ctr);

        const auto lftOffs = Length2{-2_m, 0.8_m};
        const auto rgtOffs = Length2{+2_m, 0.8_m};

        {
            const auto lftEye = m_world.CreateBody(BodyDef(StaticBD).UseLocation(ctr + lftOffs));
            const auto rgtEye = m_world.CreateBody(BodyDef(StaticBD).UseLocation(ctr + rgtOffs));
            auto cconf = ChainShapeConf{};
            cconf.restitution = 0;
            cconf.friction = 0;
            cconf.Set(GetCircleVertices(1.8_m, 24, 0_deg, 1));
            const auto eyeEnc = Shape(cconf);
            lftEye->CreateFixture(eyeEnc);
            rgtEye->CreateFixture(eyeEnc);
        }

        const auto lftPup = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(ctr + lftOffs));
        const auto rgtPup = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(ctr + rgtOffs));
        lftPup->CreateFixture(m_smallDiskShape);
        rgtPup->CreateFixture(m_smallDiskShape);
        
        // Remove gravity on dynamic bodies...
        lftPup->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
        rgtPup->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});

        m_lftMouseJoint = static_cast<MouseJoint*>(m_world.CreateJoint(MouseJointDef{lftPup}
                                                                       .UseMaxForce(200_N)
                                                                       .UseFrequency(2_Hz)
                                                                       .UseTarget(GetLocation(*lftPup))));
        m_rgtMouseJoint = static_cast<MouseJoint*>(m_world.CreateJoint(MouseJointDef{rgtPup}
                                                                       .UseMaxForce(200_N)
                                                                       .UseFrequency(2_Hz)
                                                                       .UseTarget(GetLocation(*rgtPup))));
    }
    
    void SetupMotorJoint(Length2 center)
    {
        m_motorJointCenter = center;
        const auto containerBody = SetupContainer(center);
        const auto movingBody = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(center));
        movingBody->CreateFixture(m_rectShape);
        
        auto jd = MotorJointDef{containerBody, movingBody}
            .UseMaxForce(1000_N).UseMaxTorque(1000_Nm);
        m_motorJoint = static_cast<MotorJoint*>(m_world.CreateJoint(jd));
    }
    
    void PreStep(const Settings& settings, Drawer& drawer) override
    {
        if (settings.dt > 0)
        {
            // For any joints that need accumalated time...
            m_time += settings.dt;
        }

        {
            // For the motor joint...
            const auto linearOffset = Length2{
                static_cast<Real>(2.6 * std::sin(2 * m_time)) * 1_m,
                static_cast<Real>(2.0 * std::sin(1 * m_time)) * 1_m
            };
            const auto angularOffset = static_cast<Real>(4 * m_time) * 1_rad;
            m_motorJoint->SetLinearOffset(linearOffset);
            m_motorJoint->SetAngularOffset(angularOffset);
            drawer.DrawPoint(m_motorJointCenter + linearOffset, 4.0f, Color(0.9f, 0.9f, 0.9f));
        }
    }
    
    const Length RowSize = +10_m;
    const Length ColumnSize = +10_m;
    const BodyDef StaticBD = BodyDef{}.UseType(BodyType::Static);
    const BodyDef DynamicBD = BodyDef{}.UseType(BodyType::Dynamic);
    const Length RectHHeight = 0.25_m;
    const Length RectHWidth = 2_m;
    Shape m_diskShape{DiskConf{}.UseVertexRadius(1_m).UseDensity(1_kgpm2)};
    Shape m_smallDiskShape{DiskConf{}.UseVertexRadius(0.5_m).UseDensity(1_kgpm2).UseRestitution(Real(0))};
    Shape m_squareShape{PolyConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 0.5_m)};
    Shape m_rectShape{PolyConf{}.UseDensity(1_kgpm2).SetAsBox(RectHWidth, RectHHeight)};
    const Length2 offset = Length2{+2_m, 0_m};
    
    double m_time = 0;
    Length2 m_motorJointCenter = Length2{};
    MouseJoint* m_lftMouseJoint = nullptr;
    MouseJoint* m_rgtMouseJoint = nullptr;
    MotorJoint* m_motorJoint = nullptr;
    WheelJoint* m_wheelJoint0 = nullptr;
    WheelJoint* m_wheelJoint1 = nullptr;
    GearJoint* m_gearJoint0 = nullptr;
    GearJoint* m_gearJoint1 = nullptr;
    PulleyJoint* m_pulleyJoint = nullptr;
    RevoluteJoint* m_revoluteJoint = nullptr;
    PrismaticJoint* m_prismaticJoint = nullptr;
    DistanceJoint* m_distanceJoint = nullptr;
    WeldJoint* m_weldJoint = nullptr;
    WeldJoint* m_weldJoint0 = nullptr;
    WeldJoint* m_weldJoint1 = nullptr;
    FrictionJoint* m_frictionJoint = nullptr;
    RopeJoint* m_ropeJoint = nullptr;
};

} // namespace playrho

#endif // JointsTest_hpp

