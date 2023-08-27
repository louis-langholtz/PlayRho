/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"

#include <PlayRho/d2/GearJointConf.hpp>

#include <PlayRho/d2/DistanceJointConf.hpp>
#include <PlayRho/d2/RevoluteJointConf.hpp>
#include <PlayRho/d2/PrismaticJointConf.hpp>
#include <PlayRho/d2/Joint.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/StepConf.hpp>
#include <PlayRho/d2/BodyConstraint.hpp>
#include <PlayRho/ConstraintSolverConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(GearJointConf, DefaultConstruction)
{
    auto conf = GearJointConf{};
    EXPECT_EQ(conf.bodyA, InvalidBodyID);
    EXPECT_EQ(conf.bodyB, InvalidBodyID);
    EXPECT_EQ(conf.bodyC, InvalidBodyID);
    EXPECT_EQ(conf.bodyD, InvalidBodyID);
    EXPECT_EQ(conf.ratio, Real(1));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(conf.typeDataAC));
    EXPECT_TRUE(std::holds_alternative<std::monostate>(conf.typeDataBD));
    EXPECT_EQ(GetTypeAC(conf), GetTypeID<void>());
    EXPECT_EQ(GetTypeBD(conf), GetTypeID<void>());
    auto bodies = std::vector<BodyConstraint>{};
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    EXPECT_NO_THROW(SolveVelocity(conf, bodies, StepConf{}));
    EXPECT_NO_THROW(SolvePosition(conf, bodies, ConstraintSolverConf{}));
}

#if 0
TEST(GearJointConf, ConstructionRequiresNonNullJoints)
{
    EXPECT_THROW(GearJointConf(nullptr, nullptr), NonNull<Joint*>::checker_type::exception_type);
}

TEST(GearJoint, IsOkay)
{
    auto world = World{};
    const auto b1 = CreateBody(world);
    const auto b2 = CreateBody(world);
    const auto b3 = CreateBody(world);
    const auto b4 = CreateBody(world);
    const auto dj = CreateJoint(world, DistanceJointConf{b1, b2});
    EXPECT_FALSE(GearJoint::IsOkay(GearJointConf{dj, dj}));
    const auto rj1 = CreateJoint(world, GetRevoluteJointConf(world, b1, b2, Length2{}));
    const auto rj2 = CreateJoint(world, GetRevoluteJointConf(world, b3, b4, Length2{}));
    EXPECT_TRUE(GearJoint::IsOkay(GearJointConf{rj1, rj2}));
    EXPECT_FALSE(GearJoint::IsOkay(GearJointConf{rj1, rj1}));
}
#endif

TEST(GearJoint, CreationRevolute)
{
    auto world = World{};
    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto body2 = CreateBody(world);
    const auto body3 = CreateBody(world);
    const auto rdef0 = GetRevoluteJointConf(world, body0, body1, Length2{});
    const auto rdef1 = GetRevoluteJointConf(world, body2, body3, Length2{});
    const auto revJoint1 = CreateJoint(world, rdef0);
    const auto revJoint2 = CreateJoint(world, rdef1);
    const auto def = GetGearJointConf(world, revJoint1, revJoint2);
    ASSERT_EQ(GetTypeAC(def), GetTypeID<RevoluteJointConf>());
    ASSERT_EQ(GetTypeBD(def), GetTypeID<RevoluteJointConf>());

    const auto joint = CreateJoint(world, def);
    EXPECT_EQ(GetType(world, joint), GetTypeID<GearJointConf>());
    EXPECT_EQ(GetBodyA(world, joint), def.bodyA);
    EXPECT_EQ(GetBodyB(world, joint), def.bodyB);
    EXPECT_EQ(GetCollideConnected(world, joint), def.collideConnected);
    EXPECT_EQ(GetLinearReaction(world, joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(world, joint), AngularMomentum{0});

    EXPECT_EQ(GetLocalAnchorA(world, joint), GetLocalAnchorB(world, revJoint1));
    EXPECT_EQ(GetLocalAnchorB(world, joint), GetLocalAnchorB(world, revJoint2));
    EXPECT_EQ(GetRatio(world, joint), def.ratio);

    const auto djoint = CreateJoint(world, DistanceJointConf{body0, body1});
    EXPECT_THROW(CreateJoint(world, GetGearJointConf(world, djoint, revJoint2)), InvalidArgument);
}

TEST(GearJoint, ShiftOrigin)
{
    auto world = World{};
    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto body2 = CreateBody(world);
    const auto body3 = CreateBody(world);
    const auto rdef1 = GetRevoluteJointConf(world, body0, body1, Length2{});
    const auto rdef2 = GetRevoluteJointConf(world, body2, body3, Length2{});
    const auto revJoint1 = CreateJoint(world, rdef1);
    const auto revJoint2 = CreateJoint(world, rdef2);
    const auto def = GetGearJointConf(world, revJoint1, revJoint2);
    const auto joint = CreateJoint(world, def);
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(ShiftOrigin(world, joint, newOrigin));
}

TEST(GearJoint, SetRatio)
{
    auto world = World{};
    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto body2 = CreateBody(world);
    const auto body3 = CreateBody(world);
    auto rdef1 = GetRevoluteJointConf(world, body0, body1, Length2{});
    auto rdef2 = GetRevoluteJointConf(world, body2, body3, Length2{});
    const auto revJoint1 = CreateJoint(world, rdef1);
    const auto revJoint2 = CreateJoint(world, rdef2);
    auto def = GetGearJointConf(world, revJoint1, revJoint2);
    const auto joint = CreateJoint(world, def);
    ASSERT_EQ(GetRatio(world, joint), Real(1));
    auto conf = TypeCast<GearJointConf>(GetJoint(world, joint));
    EXPECT_NO_THROW(SetRatio(conf, Real(2)));
    EXPECT_EQ(GetRatio(conf), Real(2));
    EXPECT_NO_THROW(SetJoint(world, joint, conf));
    EXPECT_EQ(GetRatio(world, joint), Real(2));
}

TEST(GearJoint, GetGearJointConfThrows)
{
    EXPECT_THROW(GetGearJointConf(Joint{}), std::bad_cast);
}

TEST(GearJoint, GetGearJointConf)
{
    auto world = World{};
    const auto body0 = CreateBody(world);
    const auto body1 = CreateBody(world);
    const auto body2 = CreateBody(world);
    const auto body3 = CreateBody(world);
    auto rdef1 = GetRevoluteJointConf(world, body0, body1, Length2{});
    auto rdef2 = GetRevoluteJointConf(world, body2, body3, Length2{});
    const auto revJoint1 = CreateJoint(world, rdef1);
    const auto revJoint2 = CreateJoint(world, rdef2);
    const auto def = GetGearJointConf(world, revJoint1, revJoint2);
    const auto joint = CreateJoint(world, def);

    ASSERT_EQ(GetType(world, joint), GetTypeID<GearJointConf>());
    ASSERT_EQ(GetBodyA(world, joint), def.bodyA);
    ASSERT_EQ(GetBodyB(world, joint), def.bodyB);
    ASSERT_EQ(GetCollideConnected(world, joint), def.collideConnected);

    ASSERT_EQ(GetLocalAnchorA(world, joint), GetLocalAnchorB(world, revJoint1));
    ASSERT_EQ(GetLocalAnchorB(world, joint), GetLocalAnchorB(world, revJoint2));
    auto conf = TypeCast<GearJointConf>(GetJoint(world, joint));
    ASSERT_EQ(GetTypeAC(conf), GetTypeID<decltype(rdef1)>());
    ASSERT_EQ(GetTypeBD(conf), GetTypeID<decltype(rdef2)>());
    ASSERT_EQ(GetRatio(world, joint), def.ratio);

    const auto cdef = GetGearJointConf(GetJoint(world, joint));
    EXPECT_EQ(cdef.bodyA, def.bodyA);
    EXPECT_EQ(cdef.bodyB, def.bodyB);
    EXPECT_EQ(cdef.collideConnected, false);

    EXPECT_EQ(GetTypeAC(cdef), GetTypeID<decltype(rdef1)>());
    EXPECT_EQ(GetTypeBD(cdef), GetTypeID<decltype(rdef2)>());
    EXPECT_EQ(cdef.ratio, Real(1));
}

TEST(GearJoint, WithDynamicCirclesAndRevoluteJoints)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto def =
        GetGearJointConf(world, CreateJoint(world, GetRevoluteJointConf(world, b1, b2, Length2{})),
                         CreateJoint(world, GetRevoluteJointConf(world, b4, b3, Length2{})));
    const auto joint = CreateJoint(world, def);
    ASSERT_NE(joint, InvalidJointID);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(GearJoint, WithDynamicCirclesAndPrismaticJoints)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto def = GetGearJointConf(
        world,
        CreateJoint(world, GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetTop())),
        CreateJoint(world, GetPrismaticJointConf(world, b4, b3, Length2{}, UnitVec::GetTop())));
    EXPECT_EQ(GetTypeAC(def), GetTypeID<PrismaticJointConf>());
    EXPECT_EQ(GetTypeBD(def), GetTypeID<PrismaticJointConf>());
    EXPECT_EQ(GetLocalAnchorA(def), Length2(-1_m, 0_m));
    EXPECT_EQ(GetLocalAnchorB(def), Length2(-2_m, 0_m));
    const auto joint = CreateJoint(world, def);
    ASSERT_NE(joint, InvalidJointID);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), 0.0, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(GearJoint, GetAnchorAandB)
{
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    Attach(world, b1, shapeId);
    Attach(world, b2, shapeId);
    const auto def =
        GetGearJointConf(world, CreateJoint(world, GetRevoluteJointConf(world, b1, b2, Length2{})),
                         CreateJoint(world, GetRevoluteJointConf(world, b4, b3, Length2{})));
    const auto joint = CreateJoint(world, def);
    ASSERT_NE(joint, InvalidJointID);

    const auto anchorA =
        GetWorldPoint(world, GetBodyA(world, joint), GetLocalAnchorA(world, joint));
    const auto anchorB =
        GetWorldPoint(world, GetBodyB(world, joint), GetLocalAnchorB(world, joint));
    EXPECT_EQ(GetAnchorA(world, joint), anchorA);
    EXPECT_EQ(GetAnchorB(world, joint), anchorB);
}

TEST(GearJointConf, EqualsOperator)
{
    EXPECT_TRUE(GearJointConf() == GearJointConf());
    {
        auto conf = GearJointConf{};
        conf.typeDataAC = GearJointConf::PrismaticData{Length2{1.2_m, -3_m}, Length2{}, UnitVec::GetTop()};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(GearJointConf() == conf);
    }
    {
        auto conf = GearJointConf{};
        conf.typeDataAC = GearJointConf::PrismaticData{Length2{}, Length2{1.2_m, -3_m}, UnitVec::GetTop()};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(GearJointConf() == conf);
    }
    {
        auto conf = GearJointConf{};
        conf.typeDataAC = GearJointConf::RevoluteData{23_deg};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(GearJointConf() == conf);
    }
    {
        auto conf = GearJointConf{};
        conf.typeDataBD = GearJointConf::RevoluteData{19_deg};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(GearJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(GearJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(GearJointConf() != GearJointConf());
    {
        auto conf = GearJointConf{};
        conf.mass = Real(3.4);
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(GearJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(GearJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<GearJointConf>()), "d2::GearJointConf");
}

TEST(GearJointConf, InitVelocity)
{
    auto conf = GearJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    conf.bodyC = BodyID(0);
    conf.bodyD = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(GearJointConf, SolveVelocity)
{
    auto conf = GearJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    conf.bodyC = BodyID(0);
    conf.bodyD = BodyID(0);
    EXPECT_THROW(SolveVelocity(conf, bodies, StepConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
}

TEST(GearJointConf, SolvePosition)
{
    auto conf = GearJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    conf.bodyC = BodyID(0);
    conf.bodyD = BodyID(0);
    EXPECT_THROW(SolvePosition(conf, bodies, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
}
