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

#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/Joints/JointConf.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>

#include <type_traits>
#include <any>

using namespace playrho;
using namespace playrho::d2;

TEST(JointBuilder, Construction)
{
    EXPECT_EQ(JointBuilder<JointConf>{}.bodyA, InvalidBodyID);
    EXPECT_EQ(JointBuilder<JointConf>{}.bodyB, InvalidBodyID);
    EXPECT_EQ(JointBuilder<JointConf>{}.collideConnected, false);
}

TEST(JointBuilder, UseBodyA)
{
    const auto b = static_cast<BodyID>(2);
    EXPECT_NE(JointBuilder<JointConf>{}.bodyA, b);
    EXPECT_EQ(JointBuilder<JointConf>{}.UseBodyA(b).bodyA, b);
}

TEST(JointBuilder, UseBodyB)
{
    const auto b = static_cast<BodyID>(77);
    EXPECT_NE(JointBuilder<JointConf>{}.bodyB, b);
    EXPECT_EQ(JointBuilder<JointConf>{}.UseBodyB(b).bodyB, b);
}

TEST(JointBuilder, UseCollideConnected)
{
    const auto value = true;
    EXPECT_NE(JointBuilder<JointConf>{}.collideConnected, value);
    EXPECT_EQ(JointBuilder<JointConf>{}.UseCollideConnected(value).collideConnected,
              value);
}

TEST(Joint, ByteSize)
{
    switch (sizeof(void*))
    {
        case 4: break;
        case 8: EXPECT_EQ(sizeof(Joint), std::size_t(8)); break;
        default: break;
    }
}

TEST(Joint, Traits)
{
    EXPECT_FALSE(IsIterable<Joint>::value);
    EXPECT_FALSE((IsAddable<Joint>::value));
    EXPECT_FALSE((IsAddable<Joint,Joint>::value));

    EXPECT_TRUE(std::is_default_constructible<Joint>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<Joint>::value);

    EXPECT_TRUE(std::is_copy_constructible<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<Joint>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<Joint>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<Joint>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<Joint>::value);
    
    EXPECT_TRUE(std::is_destructible<Joint>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Joint>::value);
    EXPECT_FALSE(std::is_trivially_destructible<Joint>::value);
}

#if 0
TEST(Joint, StaticIsOkay)
{
    using Builder = JointBuilder<JointConf>;
    EXPECT_FALSE(Joint::IsOkay(Builder{JointType::Unknown}));
    EXPECT_FALSE(Joint::IsOkay(Builder{JointType::Friction}));
    const auto b1 = reinterpret_cast<Body*>(0x1);
    const auto b2 = reinterpret_cast<Body*>(0x2);
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyA(b1)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyA(b2)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyB(b1)));
    EXPECT_TRUE(Joint::IsOkay(Builder{JointType::Friction}.UseBodyB(b2)));
}
#endif

TEST(WorldJoint, GetWorldIndexFreeFunction)
{
    World world;
    EXPECT_EQ(GetWorldIndex(world, InvalidJointID), JointCounter(-1));
}

TEST(Joint, LimitStateToStringFF)
{
    const auto equalLimitsString = std::string(ToString(LimitState::e_equalLimits));
    const auto inactiveLimitString = std::string(ToString(LimitState::e_inactiveLimit));
    const auto upperLimitsString = std::string(ToString(LimitState::e_atUpperLimit));
    const auto lowerLimitsString = std::string(ToString(LimitState::e_atLowerLimit));
    
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

namespace {

struct JointTester {
    int number = 0;
};

bool ShiftOrigin(JointTester&, Length2) noexcept
{
    return false;
}

void InitVelocity(JointTester&, std::vector<BodyConstraint>&, const StepConf&,
                  const ConstraintSolverConf&)
{
}

bool SolveVelocity(JointTester&, std::vector<BodyConstraint>&, const StepConf&)
{
    return true;
}

bool SolvePosition(const JointTester&, std::vector<BodyConstraint>&, const ConstraintSolverConf&)
{
    return true;
}

} // namespace

TEST(Joint, TypeCast)
{
    std::any test;
    {
        const auto joint = Joint{};
        auto value = static_cast<const int*>(nullptr);
        EXPECT_NO_THROW(value = TypeCast<const int>(&joint));
        EXPECT_TRUE(value == nullptr);
    }
    {
        auto joint = Joint{};
        auto value = static_cast<int*>(nullptr);
        EXPECT_NO_THROW(value = TypeCast<int>(&joint));
        EXPECT_TRUE(value == nullptr);
    }
    {
        const auto joint = Joint{};
        EXPECT_THROW(TypeCast<int>(joint), std::bad_cast);
        EXPECT_THROW(TypeCast<const int>(joint), std::bad_cast);
    }
    {
        auto joint = Joint{};
        EXPECT_THROW(TypeCast<int>(joint), std::bad_cast);
        EXPECT_THROW(TypeCast<const int>(joint), std::bad_cast);
    }
    {
        auto number = 10;
        const auto original = JointTester{number};
        EXPECT_EQ(original.number, number);
        auto joint = Joint{original};
        EXPECT_THROW(TypeCast<int>(joint), std::bad_cast);
        auto value = JointTester{};
        EXPECT_NO_THROW(value = TypeCast<JointTester>(joint));
        EXPECT_EQ(value.number, number);
        EXPECT_NO_THROW(TypeCast<JointTester&>(joint).number = 3);
        EXPECT_EQ(TypeCast<const JointTester&>(joint).number, 3);
        EXPECT_NO_THROW(value = TypeCast<JointTester>(joint));
        EXPECT_EQ(value.number, 3);
        EXPECT_NO_THROW(TypeCast<JointTester>(&joint)->number = 4);
        EXPECT_EQ(TypeCast<const JointTester>(joint).number, 4);
    }
}
