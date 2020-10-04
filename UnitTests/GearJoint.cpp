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

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/TypeJointVisitor.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(GearJointConf, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(32));
#else
            EXPECT_EQ(sizeof(GearJointConf), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(GearJointConf), std::size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(GearJointConf), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(GearJointConf, Traits)
{
    EXPECT_FALSE(IsIterable<GearJointConf>::value);

    EXPECT_FALSE(std::is_default_constructible<GearJointConf>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<GearJointConf>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<GearJointConf>::value);
    
    EXPECT_FALSE(std::is_constructible<GearJointConf>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<GearJointConf>::value);
    EXPECT_FALSE(std::is_trivially_constructible<GearJointConf>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<GearJointConf>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<GearJointConf>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<GearJointConf>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<GearJointConf>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<GearJointConf>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<GearJointConf>::value);
    
    EXPECT_TRUE(std::is_destructible<GearJointConf>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<GearJointConf>::value);
    EXPECT_TRUE(std::is_trivially_destructible<GearJointConf>::value);
}
#if 0
TEST(GearJointConf, ConstructionRequiresNonNullJoints)
{
    EXPECT_THROW(GearJointConf(nullptr, nullptr), NonNull<Joint*>::checker_type::exception_type);
}

TEST(GearJoint, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(GearJoint), std::size_t(144));
#else
            EXPECT_EQ(sizeof(GearJoint), std::size_t(184));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(GearJoint), std::size_t(288)); break;
        case 16: EXPECT_EQ(sizeof(GearJoint), std::size_t(496)); break;
        default: FAIL(); break;
    }
}

TEST(GearJoint, IsOkay)
{
    auto world = World{};
    const auto b1 = world.CreateBody();
    const auto b2 = world.CreateBody();
    const auto b3 = world.CreateBody();
    const auto b4 = world.CreateBody();
    const auto dj = world.CreateJoint(DistanceJointConf{b1, b2});
    EXPECT_FALSE(GearJoint::IsOkay(GearJointConf{dj, dj}));
    const auto rj1 = world.CreateJoint(GetRevoluteJointConf(m_world, b1, b2, Length2{}));
    const auto rj2 = world.CreateJoint(GetRevoluteJointConf(m_world, b3, b4, Length2{}));
    EXPECT_TRUE(GearJoint::IsOkay(GearJointConf{rj1, rj2}));
    EXPECT_FALSE(GearJoint::IsOkay(GearJointConf{rj1, rj1}));
}

TEST(GearJoint, Creation)
{
    auto world = World{};
    const auto body0 = world.CreateBody();
    const auto body1 = world.CreateBody();
    const auto body2 = world.CreateBody();
    const auto body3 = world.CreateBody();
    auto rdef0 = GetRevoluteJointConf(m_world, body0, body1, Length2{});
    auto rdef1 = GetRevoluteJointConf(m_world, body2, body3, Length2{});
    auto revJoint1 = RevoluteJoint{rdef0};
    auto revJoint2 = RevoluteJoint{rdef1};
    auto def = GearJointConf{&revJoint1, &revJoint2};
    auto& joint = *static_cast<GearJoint*>(world.CreateJoint(def));

    EXPECT_EQ(GetType(joint), def.type);
    EXPECT_EQ(joint.GetBodyA(), def.joint1->GetBodyB());
    EXPECT_EQ(joint.GetBodyB(), def.joint2->GetBodyB());
    EXPECT_EQ(joint.GetCollideConnected(), def.collideConnected);
    EXPECT_EQ(joint.GetUserData(), def.userData);
    EXPECT_EQ(joint.GetLinearReaction(), Momentum2{});
    EXPECT_EQ(joint.GetAngularReaction(), AngularMomentum{0});
    
    EXPECT_EQ(joint.GetLocalAnchorA(), revJoint1.GetLocalAnchorB());
    EXPECT_EQ(joint.GetLocalAnchorB(), revJoint2.GetLocalAnchorB());
    EXPECT_EQ(joint.GetJoint1(), def.joint1);
    EXPECT_EQ(joint.GetJoint2(), def.joint2);
    EXPECT_EQ(joint.GetRatio(), def.ratio);
    
    TypeJointVisitor visitor;
    joint.Accept(visitor);
    EXPECT_EQ(visitor.GetType().value(), JointType::Gear);
    
    const auto djoint = world.CreateJoint(DistanceJointConf{body0, body1});
    EXPECT_THROW(world.CreateJoint(GearJointConf{djoint, &revJoint2}), InvalidArgument);
}

TEST(GearJoint, ShiftOrigin)
{
    auto world = World{};
    auto& body0 = *world.CreateBody();
    auto& body1 = *world.CreateBody();
    auto& body2 = *world.CreateBody();
    auto& body3 = *world.CreateBody();
    auto rdef1 = GetRevoluteJointConf(m_world, &body0, &body1, Length2{});
    auto rdef2 = GetRevoluteJointConf(m_world, &body2, &body3, Length2{});
    auto revJoint1 = RevoluteJoint{rdef1};
    auto revJoint2 = RevoluteJoint{rdef2};
    auto def = GearJointConf{&revJoint1, &revJoint2};
    auto& joint = *static_cast<GearJoint*>(world.CreateJoint(def));
    const auto newOrigin = Length2{1_m, 1_m};
    EXPECT_FALSE(joint.ShiftOrigin(newOrigin));
}

TEST(GearJoint, SetRatio)
{
    auto world = World{};
    auto& body0 = *world.CreateBody();
    auto& body1 = *world.CreateBody();
    auto& body2 = *world.CreateBody();
    auto& body3 = *world.CreateBody();
    auto rdef1 = GetRevoluteJointConf(m_world, &body0, &body1, Length2{});
    auto rdef2 = GetRevoluteJointConf(m_world, &body2, &body3, Length2{});
    RevoluteJoint revJoint1{rdef1};
    RevoluteJoint revJoint2{rdef2};
    auto def = GearJointConf{&revJoint1, &revJoint2};
    auto& joint = *static_cast<GearJoint*>(world.CreateJoint(def));
    ASSERT_EQ(joint.GetRatio(), Real(1));
    joint.SetRatio(Real(2));
    EXPECT_EQ(joint.GetRatio(), Real(2));
}

TEST(GearJoint, GetGearJointConf)
{
    auto world = World{};
    auto& body0 = *world.CreateBody();
    auto& body1 = *world.CreateBody();
    auto& body2 = *world.CreateBody();
    auto& body3 = *world.CreateBody();
    auto rdef1 = GetRevoluteJointConf(m_world, &body0, &body1, Length2{});
    auto rdef2 = GetRevoluteJointConf(m_world, &body2, &body3, Length2{});
    RevoluteJoint revJoint1{rdef1};
    RevoluteJoint revJoint2{rdef2};
    GearJointConf def{&revJoint1, &revJoint2};
    auto& joint = *static_cast<GearJoint*>(world.CreateJoint(def));

    ASSERT_EQ(GetType(joint), def.type);
    ASSERT_EQ(joint.GetBodyA(), def.joint1->GetBodyB());
    ASSERT_EQ(joint.GetBodyB(), def.joint2->GetBodyB());
    ASSERT_EQ(joint.GetCollideConnected(), def.collideConnected);
    ASSERT_EQ(joint.GetUserData(), def.userData);
    
    ASSERT_EQ(joint.GetLocalAnchorA(), revJoint1.GetLocalAnchorB());
    ASSERT_EQ(joint.GetLocalAnchorB(), revJoint2.GetLocalAnchorB());
    ASSERT_EQ(joint.GetJoint1(), def.joint1);
    ASSERT_EQ(joint.GetJoint2(), def.joint2);
    ASSERT_EQ(joint.GetRatio(), def.ratio);
    
    const auto cdef = GetGearJointConf(joint);
    EXPECT_EQ(cdef.type, JointType::Gear);
    EXPECT_EQ(cdef.bodyA, def.joint1->GetBodyB());
    EXPECT_EQ(cdef.bodyB, def.joint2->GetBodyB());
    EXPECT_EQ(cdef.collideConnected, false);
    EXPECT_EQ(cdef.userData, nullptr);
    
    EXPECT_EQ(cdef.joint1, &revJoint1);
    EXPECT_EQ(cdef.joint2, &revJoint2);
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
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    world.CreateFixture(*b1, circle);
    world.CreateFixture(*b2, circle);
    GearJointConf def{
        world.CreateJoint(GetRevoluteJointConf(m_world, b1, b2, Length2{})),
        world.CreateJoint(GetRevoluteJointConf(m_world, b4, b3, Length2{}))
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = world.CreateJoint(def);
    ASSERT_NE(joint, nullptr);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
}

TEST(GearJoint, WithDynamicCirclesAndPrismaticJoints)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    world.CreateFixture(*b1, circle);
    world.CreateFixture(*b2, circle);
    GearJointConf def{
        world.CreateJoint(GetPrismaticJointConf(world, b1, b2, Length2{}, UnitVec::GetTop())),
        world.CreateJoint(GetPrismaticJointConf(world, b4, b3, Length2{}, UnitVec::GetTop()))
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = world.CreateJoint(def);
    ASSERT_NE(joint, nullptr);
    Step(world, 1_s);
    EXPECT_NEAR(double(Real{GetX(b1->GetLocation()) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(b1->GetLocation()) / Meter}), 0.0, 0.001);
    EXPECT_NEAR(double(Real{GetX(b2->GetLocation()) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(b2->GetLocation()) / Meter}), 0.0, 0.01);
    EXPECT_EQ(b1->GetAngle(), 0_deg);
    EXPECT_EQ(b2->GetAngle(), 0_deg);
}

TEST(GearJoint, GetAnchorAandB)
{
    const auto circle = Shape{DiskShapeConf{}.UseRadius(0.2_m)};
    auto world = World{};
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto p3 = Length2{+2_m, 0_m};
    const auto p4 = Length2{+3_m, 0_m};
    const auto b1 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p1));
    const auto b2 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));
    const auto b3 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p3));
    const auto b4 = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p4));
    world.CreateFixture(*b1, circle);
    world.CreateFixture(*b2, circle);
    GearJointConf def{
        world.CreateJoint(GetRevoluteJointConf(m_world, b1, b2, Length2{})),
        world.CreateJoint(GetRevoluteJointConf(m_world, b4, b3, Length2{}))
    };
    ASSERT_NE(def.joint1, nullptr);
    ASSERT_NE(def.joint2, nullptr);
    const auto joint = static_cast<GearJoint*>(world.CreateJoint(def));
    ASSERT_NE(joint, nullptr);
    
    const auto anchorA = GetWorldPoint(*(joint->GetBodyA()), joint->GetLocalAnchorA());
    const auto anchorB = GetWorldPoint(*(joint->GetBodyB()), joint->GetLocalAnchorB());

    EXPECT_EQ(joint->GetAnchorA(world), anchorA);
    EXPECT_EQ(joint->GetAnchorB(world), anchorB);
}
#endif
