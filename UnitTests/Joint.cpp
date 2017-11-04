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

#include "gtest/gtest.h"
#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <type_traits>

using namespace playrho;

TEST(JointBuilder, Construction)
{
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Unknown}.type, JointType::Unknown);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Unknown}.bodyA, nullptr);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Unknown}.bodyB, nullptr);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Unknown}.collideConnected, false);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Unknown}.userData, nullptr);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Gear}.type, JointType::Gear);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Rope}.type, JointType::Rope);
}

TEST(JointBuilder, UseBodyA)
{
    const auto b = reinterpret_cast<Body*>(2);
    EXPECT_NE(JointBuilder<JointDef>{JointType::Rope}.bodyA, b);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Rope}.UseBodyA(b).bodyA, b);
}

TEST(JointBuilder, UseBodyB)
{
    const auto b = reinterpret_cast<Body*>(77);
    EXPECT_NE(JointBuilder<JointDef>{JointType::Rope}.bodyB, b);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Rope}.UseBodyB(b).bodyB, b);
}

TEST(JointBuilder, UseCollideConnected)
{
    const auto cc = true;
    EXPECT_NE(JointBuilder<JointDef>{JointType::Rope}.collideConnected, cc);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Rope}.UseCollideConnected(cc).collideConnected, cc);
}

TEST(JointBuilder, UseUserData)
{
    const auto d = reinterpret_cast<void*>(318);
    EXPECT_NE(JointBuilder<JointDef>{JointType::Rope}.userData, d);
    EXPECT_EQ(JointBuilder<JointDef>{JointType::Rope}.UseUserData(d).userData, d);
}

TEST(Joint, ByteSize)
{
    switch (sizeof(void*))
    {
        case 4: break;
        case 8: EXPECT_EQ(sizeof(Joint), std::size_t(40)); break;
        default: break;
    }
}

TEST(Joint, Traits)
{
    EXPECT_FALSE(std::is_default_constructible<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<Joint>::value);
    
    EXPECT_FALSE(std::is_constructible<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_constructible<Joint>::value);
    
    EXPECT_FALSE(std::is_copy_constructible<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<Joint>::value);
    
    EXPECT_FALSE(std::is_copy_assignable<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<Joint>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<Joint>::value);
    
    EXPECT_TRUE(std::is_destructible<Joint>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_destructible<Joint>::value);
}

TEST(Joint, StaticIsOkay)
{
    using Builder = JointBuilder<JointDef>;
    EXPECT_FALSE(Joint::IsOkay(Builder{JointType::Unknown}));
    EXPECT_FALSE(Joint::IsOkay(Builder{JointType::Friction}));
    const auto b1 = reinterpret_cast<Body*>(0x1);
    const auto b2 = reinterpret_cast<Body*>(0x2);
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyA(b1)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyA(b2)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyB(b1)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyB(b2)));
}

TEST(Joint, GetWorldIndexFreeFunction)
{
    EXPECT_EQ(GetWorldIndex(static_cast<const Joint*>(nullptr)), JointCounter(-1));
}

TEST(Joint, LimitStateToStringFF)
{
    const auto equalLimitsString = std::string(ToString(Joint::e_equalLimits));
    const auto inactiveLimitString = std::string(ToString(Joint::e_inactiveLimit));
    const auto upperLimitsString = std::string(ToString(Joint::e_atUpperLimit));
    const auto lowerLimitsString = std::string(ToString(Joint::e_atLowerLimit));
    
    EXPECT_FALSE(equalLimitsString.empty());
    EXPECT_FALSE(inactiveLimitString.empty());
    EXPECT_FALSE(upperLimitsString.empty());
    EXPECT_FALSE(lowerLimitsString.empty());
    std::set<std::string> names;
    names.insert(equalLimitsString);
    names.insert(inactiveLimitString);
    names.insert(upperLimitsString);
    names.insert(lowerLimitsString);
    EXPECT_EQ(names.size(), decltype(names.size()){4});
}
