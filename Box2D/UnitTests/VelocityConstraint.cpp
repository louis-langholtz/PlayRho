/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>
#include <Box2D/Dynamics/Contacts/BodyConstraint.hpp>
#include <Box2D/Collision/WorldManifold.hpp>

using namespace box2d;

TEST(VelocityConstraint, ByteSizeIs_168_or_304_or_576)
{
    switch (sizeof(RealNum))
    {
        case  4: EXPECT_EQ(sizeof(VelocityConstraint), size_t(168)); break;
        case  8: EXPECT_EQ(sizeof(VelocityConstraint), size_t(304)); break;
        case 16: EXPECT_EQ(sizeof(VelocityConstraint), size_t(576)); break;
        default: FAIL(); break;
    }
}

#if 0
TEST(VelocityConstraint, DefaultInit)
{
    VelocityConstraint vc;
    EXPECT_FALSE(IsValid(vc.GetK()));
    EXPECT_FALSE(IsValid(vc.GetNormalMass()));
    EXPECT_FALSE(IsValid(vc.GetNormal()));
    EXPECT_FALSE(IsValid(vc.GetFriction()));
    EXPECT_FALSE(IsValid(vc.GetRestitution()));
    EXPECT_FALSE(IsValid(vc.GetTangentSpeed()));
    EXPECT_FALSE(IsValid(vc.GetContactIndex()));
    
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{0});

    EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(0)));
    EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(0)));
    EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(0)));
    EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(0)));
    EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(0)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosA(0)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosB(0)));

    EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(1)));    
    EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosA(1)));
    EXPECT_FALSE(IsValid(vc.GetPointRelPosB(1)));
}

TEST(VelocityConstraint, InitializingConstructor)
{
    const auto contact_index = VelocityConstraint::index_type{3};
    const auto friction = RealNum(0.432);
    const auto restitution = RealNum(0.989);
    const auto tangent_speed = RealNum(1.876);
    
    auto bodyA = BodyConstraint{};
    auto bodyB = BodyConstraint{};
    const auto normal = UnitVec2::GetTop();

    const VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB, normal};

    EXPECT_EQ(vc.GetContactIndex(), contact_index);
    EXPECT_EQ(vc.GetFriction(), friction);
    EXPECT_EQ(vc.GetRestitution(), restitution);
    EXPECT_EQ(vc.GetTangentSpeed(), tangent_speed);
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
    EXPECT_EQ(vc.GetNormal(), normal);
}

TEST(VelocityConstraint, AddPoint)
{
    const auto contact_index = VelocityConstraint::index_type{3};
    const auto friction = RealNum(0.432);
    const auto restitution = RealNum(0.989);
    const auto tangent_speed = RealNum(1.876);

    auto bodyA = BodyConstraint{};
    auto bodyB = BodyConstraint{};
    const auto normal = UnitVec2::GetTop();

    VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB, normal};

    ASSERT_EQ(vc.GetContactIndex(), contact_index);
    ASSERT_EQ(vc.GetFriction(), friction);
    ASSERT_EQ(vc.GetRestitution(), restitution);
    ASSERT_EQ(vc.GetTangentSpeed(), tangent_speed);
    ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
    ASSERT_EQ(vc.GetNormal(), normal);

    const auto ni = RealNum(1.2);
    const auto ti = RealNum(0.3);
    
    const auto rA = Vec2{0, 0};
    const auto rB = Vec2{0, 0};
    
    vc.AddPoint(ni, ti, rA, rB, VelocityConstraint::Conf{});
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(1));
    
    vc.AddPoint(ni + 2, ti + 2, rA, rB, VelocityConstraint::Conf{});
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(2));

    EXPECT_EQ(GetNormalImpulseAtPoint(vc, 0), ni);
    EXPECT_EQ(GetTangentImpulseAtPoint(vc, 0), ti);
    EXPECT_EQ(GetNormalImpulseAtPoint(vc, 1), ni + 2);
    EXPECT_EQ(GetTangentImpulseAtPoint(vc, 1), ti + 2);
}
#endif
