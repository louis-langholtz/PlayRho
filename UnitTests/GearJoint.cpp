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

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(GearJointConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32)
#if defined(_WIN64)
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(136));
#else
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(124));
#endif
#else
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(136));
#endif
            break;
        case  8:
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(240));
            break;
        case 16:
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(448));
            break;
        default:
            FAIL();
            break;
    }
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

TEST(GearJoint, Creation)
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
    ASSERT_EQ(def.type1, GetTypeID<RevoluteJointConf>());
    ASSERT_EQ(def.type2, GetTypeID<RevoluteJointConf>());

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
    ASSERT_EQ(GetType1(conf), GetTypeID<decltype(rdef1)>());
    ASSERT_EQ(GetType2(conf), GetTypeID<decltype(rdef2)>());
    ASSERT_EQ(GetRatio(world, joint), def.ratio);
    
    const auto cdef = GetGearJointConf(GetJoint(world, joint));
    EXPECT_EQ(cdef.bodyA, def.bodyA);
    EXPECT_EQ(cdef.bodyB, def.bodyB);
    EXPECT_EQ(cdef.collideConnected, false);
    
    EXPECT_EQ(cdef.type1, GetTypeID<decltype(rdef1)>());
    EXPECT_EQ(cdef.type2, GetTypeID<decltype(rdef2)>());
    EXPECT_EQ(cdef.ratio, Real(1));
}

TEST(GearJoint, WithDynamicCirclesAndRevoluteJoints)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    CreateFixture(world, b1, circle);
    CreateFixture(world, b2, circle);
    const auto def = GetGearJointConf(world,
                                      CreateJoint(world, GetRevoluteJointConf(world, b1, b2, Length2{})),
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
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    CreateFixture(world, b1, circle);
    CreateFixture(world, b2, circle);
    const auto def = GetGearJointConf(world,
        CreateJoint(world, GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetTop())),
        CreateJoint(world, GetPrismaticJointConf(world, b4, b3, Length2{}, UnitVec::GetTop()))
    );
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
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    CreateFixture(world, b1, circle);
    CreateFixture(world, b2, circle);
    const auto def = GetGearJointConf(world,
        CreateJoint(world, GetRevoluteJointConf(world, b1, b2, Length2{})),
        CreateJoint(world, GetRevoluteJointConf(world, b4, b3, Length2{}))
    );
    const auto joint = CreateJoint(world, def);
    ASSERT_NE(joint, InvalidJointID);
    
    const auto anchorA = GetWorldPoint(world, GetBodyA(world, joint), GetLocalAnchorA(world, joint));
    const auto anchorB = GetWorldPoint(world, GetBodyB(world, joint), GetLocalAnchorB(world, joint));
    EXPECT_EQ(GetAnchorA(world, joint), anchorA);
    EXPECT_EQ(GetAnchorB(world, joint), anchorB);
}
