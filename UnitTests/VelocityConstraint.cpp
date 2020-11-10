/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>
#include <PlayRho/Dynamics/Contacts/BodyConstraint.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(VelocityConstraint, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(VelocityConstraint), std::size_t(128));
#else
            EXPECT_EQ(sizeof(VelocityConstraint), std::size_t(136));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(VelocityConstraint), std::size_t(256)); break;
        case 16: EXPECT_EQ(sizeof(VelocityConstraint), std::size_t(496)); break;
        default: FAIL(); break;
    }
}

TEST(VelocityConstraint, DefaultInit)
{
    const auto vc = VelocityConstraint{};

    //EXPECT_FALSE(IsValid(vc.GetK()));
    //EXPECT_FALSE(IsValid(vc.GetNormalMass()));
    EXPECT_FALSE(IsValid(vc.GetNormal()));
    //EXPECT_FALSE(IsValid(vc.GetFriction()));
    //EXPECT_FALSE(IsValid(vc.GetRestitution()));
    //EXPECT_FALSE(IsValid(vc.GetTangentSpeed()));
    
    EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{0});

    EXPECT_TRUE(IsValid(vc.GetNormalImpulseAtPoint(0)));
    EXPECT_TRUE(IsValid(vc.GetTangentImpulseAtPoint(0)));
    EXPECT_TRUE(IsValid(vc.GetNormalMassAtPoint(0)));
    EXPECT_TRUE(IsValid(vc.GetTangentMassAtPoint(0)));
    EXPECT_TRUE(IsValid(vc.GetVelocityBiasAtPoint(0)));
    EXPECT_TRUE(IsValid(vc.GetPointRelPosA(0)));
    EXPECT_TRUE(IsValid(vc.GetPointRelPosB(0)));

    EXPECT_TRUE(IsValid(vc.GetNormalImpulseAtPoint(1)));
    EXPECT_TRUE(IsValid(vc.GetTangentImpulseAtPoint(1)));
    EXPECT_TRUE(IsValid(vc.GetNormalMassAtPoint(1)));
    EXPECT_TRUE(IsValid(vc.GetTangentMassAtPoint(1)));
    EXPECT_TRUE(IsValid(vc.GetVelocityBiasAtPoint(1)));
    EXPECT_TRUE(IsValid(vc.GetPointRelPosA(1)));
    EXPECT_TRUE(IsValid(vc.GetPointRelPosB(1)));
}

#if 0
TEST(VelocityConstraint, InitializingConstructor)
{
    const auto contact_index = VelocityConstraint::index_type{3};
    const auto friction = Real(0.432);
    const auto restitution = Real(0.989);
    const auto tangent_speed = Real(1.876);
    
    auto bodyA = BodyConstraint{};
    auto bodyB = BodyConstraint{};
    const auto normal = UnitVec::GetTop();

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
    const auto friction = Real(0.432);
    const auto restitution = Real(0.989);
    const auto tangent_speed = Real(1.876);

    auto bodyA = BodyConstraint{};
    auto bodyB = BodyConstraint{};
    const auto normal = UnitVec::GetTop();

    VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB, normal};

    ASSERT_EQ(vc.GetContactIndex(), contact_index);
    ASSERT_EQ(vc.GetFriction(), friction);
    ASSERT_EQ(vc.GetRestitution(), restitution);
    ASSERT_EQ(vc.GetTangentSpeed(), tangent_speed);
    ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
    ASSERT_EQ(vc.GetNormal(), normal);

    const auto ni = Real(1.2);
    const auto ti = Real(0.3);
    
    const auto rA = Length2{};
    const auto rB = Length2{};
    
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
