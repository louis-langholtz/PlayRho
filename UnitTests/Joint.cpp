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

#include <playrho/d2/Joint.hpp>

#include <playrho/Templates.hpp>
#include <playrho/d2/JointConf.hpp>
#include <playrho/d2/WheelJointConf.hpp>

#include <any>
#include <stdexcept> // for std::invalid_argument
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

// Macros for testing eligibility of types for construction with or assignment to Joint instances.
#define DEFINE_GETBODYA                                                                            \
    [[maybe_unused]] BodyID GetBodyA(const JointTester&) noexcept                                  \
    {                                                                                              \
        return InvalidBodyID;                                                                      \
    }
#define DEFINE_GETBODYB                                                                            \
    [[maybe_unused]] BodyID GetBodyB(const JointTester&) noexcept                                  \
    {                                                                                              \
        return InvalidBodyID;                                                                      \
    }
#define DEFINE_GETCOLLIDECONNECTED                                                                 \
    [[maybe_unused]] bool GetCollideConnected(const JointTester&) noexcept                         \
    {                                                                                              \
        return false;                                                                              \
    }
#define DEFINE_SHIFTORIGIN                                                                         \
    [[maybe_unused]] bool ShiftOrigin(JointTester&, Length2) noexcept                              \
    {                                                                                              \
        return false;                                                                              \
    }
#define DEFINE_INITVELOCITY                                                                        \
    [[maybe_unused]] void InitVelocity(JointTester&, const Span<BodyConstraint>&,                  \
                                       const StepConf&, const ConstraintSolverConf&)               \
    {                                                                                              \
    }
#define DEFINE_SOLVEVELOCITY                                                                       \
    [[maybe_unused]] bool SolveVelocity(JointTester&, const Span<BodyConstraint>&,                 \
                                        const StepConf&)                                           \
    {                                                                                              \
        return true;                                                                               \
    }
#define DEFINE_SOLVEPOSITION                                                                       \
    [[maybe_unused]] bool SolvePosition(const JointTester&, const Span<BodyConstraint>&,           \
                                        const ConstraintSolverConf&)                               \
    {                                                                                              \
        return true;                                                                               \
    }
#define DEFINE_EQUALS                                                                              \
    [[maybe_unused]] bool operator==(const JointTester& lhs, const JointTester& rhs) noexcept      \
    {                                                                                              \
        return lhs.number == rhs.number;                                                           \
    }

namespace test {

// Namespace of type eligible for use with Joint.
namespace sans_none {
namespace {
struct JointTester {
    static int defaultConstructorCalled;
    static int copyConstructorCalled;
    static int moveConstructorCalled;
    static int copyAssignmentCalled;
    static int moveAssignmentCalled;

    static void resetClass()
    {
        defaultConstructorCalled = 0;
        copyConstructorCalled = 0;
        moveConstructorCalled = 0;
        copyAssignmentCalled = 0;
        moveAssignmentCalled = 0;
    }

    int number = 0;
    std::string data;

    JointTester()
    {
        ++defaultConstructorCalled;
    }

    JointTester(const JointTester& other): number{other.number}, data{other.data}
    {
        ++copyConstructorCalled;
    }

    JointTester(JointTester&& other): number{std::move(other.number)}, data{std::move(other.data)}
    {
        ++moveConstructorCalled;
    }

    JointTester& operator=(const JointTester& other)
    {
        number = other.number;
        data = other.data;
        ++copyAssignmentCalled;
        return *this;
    }

    JointTester& operator=(JointTester&& other)
    {
        number = std::move(other.number);
        data = std::move(other.data);
        ++moveAssignmentCalled;
        return *this;
    }
};

int JointTester::defaultConstructorCalled;
int JointTester::copyConstructorCalled;
int JointTester::moveConstructorCalled;
int JointTester::copyAssignmentCalled;
int JointTester::moveAssignmentCalled;

DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_none

// Namespace of type ineligible for use with Joint because missing GetBodyA.
namespace sans_getbodya {
namespace {
struct JointTester {
    int number = 0;
};
// DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_getbodya

// Namespace of type ineligible for use with Joint because missing GetBodyB.
namespace sans_getbodyb {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
// DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_getbodyb

// Namespace of type ineligible for use with Joint because missing GetCollideConnected.
namespace sans_getcollideconnected {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
// DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_getcollideconnected

// Namespace of type ineligible for use with Joint because missing ShiftOrigin.
namespace sans_shiftorigin {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
// DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_shiftorigin

// Namespace of type ineligible for use with Joint because missing InitVelocity.
namespace sans_initvelocity {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
// DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_initvelocity

// Namespace of type ineligible for use with Joint because missing SolveVelocity.
namespace sans_solvevelocity {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
// DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_solvevelocity

// Namespace of type ineligible for use with Joint because missing SolvePosition.
namespace sans_solveposition {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
// DEFINE_SOLVEPOSITION;
DEFINE_EQUALS;
} // namespace
} // namespace sans_solveposition

// Namespace of type ineligible for use with Joint because missing equals operator.
namespace sans_equals {
namespace {
struct JointTester {
    int number = 0;
};
DEFINE_GETBODYA;
DEFINE_GETBODYB;
DEFINE_GETCOLLIDECONNECTED;
DEFINE_SHIFTORIGIN;
DEFINE_INITVELOCITY;
DEFINE_SOLVEVELOCITY;
DEFINE_SOLVEPOSITION;
// DEFINE_EQUALS;
} // namespace
} // namespace sans_equals

// Namespace of type ineligible for use with Joint because missing all required functions.
namespace sans_all {
namespace {
struct JointTester {
    int number = 0;
};
} // namespace
} // namespace sans_all

// Compile-time test the different combos defined above...
static_assert(IsValidJointType<sans_none::JointTester>::value);
static_assert(!IsValidJointType<sans_getbodya::JointTester>::value);
static_assert(!IsValidJointType<sans_getbodyb::JointTester>::value);
static_assert(!IsValidJointType<sans_getcollideconnected::JointTester>::value);
static_assert(!IsValidJointType<sans_shiftorigin::JointTester>::value);
static_assert(!IsValidJointType<sans_initvelocity::JointTester>::value);
static_assert(!IsValidJointType<sans_solvevelocity::JointTester>::value);
static_assert(!IsValidJointType<sans_solveposition::JointTester>::value);
static_assert(!IsValidJointType<sans_equals::JointTester>::value);
static_assert(!IsValidJointType<sans_all::JointTester>::value);

} // namespace test

TEST(JointConf, Traits)
{
    EXPECT_TRUE(std::is_default_constructible_v<JointConf>);
    EXPECT_TRUE(std::is_nothrow_default_constructible_v<JointConf>);

    EXPECT_TRUE(std::is_copy_constructible_v<JointConf>);
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<JointConf>);
}

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
    EXPECT_EQ(JointBuilder<JointConf>{}.UseCollideConnected(value).collideConnected, value);
}

TEST(Joint, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(void*)) {
    case 4:
        break;
    case 8:
        EXPECT_EQ(sizeof(Joint), std::size_t(8));
        break;
    default:
        break;
    }
}

TEST(Joint, Traits)
{
    EXPECT_FALSE(IsIterable<Joint>::value);
    EXPECT_FALSE((IsAddable<Joint>::value));
    EXPECT_FALSE((IsAddable<Joint, Joint>::value));

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

    // Double parenthesis needed for proper macro expansion.
    EXPECT_TRUE((std::is_constructible<Joint, int>::value));
    EXPECT_TRUE((std::is_constructible<Joint, char*>::value));
    EXPECT_TRUE((std::is_constructible<Joint, test::sans_none::JointTester>::value));
}

TEST(Joint, DefaultConstructor)
{
    const Joint joint;
    EXPECT_TRUE(joint == joint);
    EXPECT_FALSE(joint != joint);
    EXPECT_EQ(GetBodyA(joint), InvalidBodyID);
    EXPECT_EQ(GetBodyB(joint), InvalidBodyID);
    EXPECT_FALSE(GetCollideConnected(joint));
    EXPECT_FALSE(joint.has_value());
}

TEST(Joint, CopyAssignment)
{
    const auto bodyA = BodyID{4};
    const auto bodyB = BodyID{11};

    auto dst = Joint{};
    ASSERT_EQ(GetBodyA(dst), InvalidBodyID);
    ASSERT_EQ(GetBodyB(dst), InvalidBodyID);

    auto conf = WheelJointConf{};
    conf.bodyA = bodyA;
    conf.bodyB = bodyB;
    const auto src = Joint{conf};
    ASSERT_EQ(GetBodyA(src), bodyA);
    ASSERT_EQ(GetBodyB(src), bodyB);
    ASSERT_NE(GetType(dst), GetType(src));
    ASSERT_NE(dst, src);

    dst = src;
    EXPECT_EQ(GetBodyA(dst), bodyA);
    EXPECT_EQ(GetBodyB(dst), bodyB);
    EXPECT_EQ(GetType(dst), GetType(src));
    EXPECT_EQ(dst, src);
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

TEST(Joint, TypeCast)
{
    int foo = 5;
    std::any test{foo};
    std::string roo = "wow";
    test = roo;
    test = std::any{foo};
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
}

TEST(Joint, TypeCastWithSansNoneToInt)
{
    constexpr auto number = 10;
    const auto original = [&](){test::sans_none::JointTester v; v.number = number; return v;}();
    EXPECT_EQ(original.number, number);
    auto joint = Joint{original};
    ASSERT_TRUE(joint.has_value());
    test::sans_none::JointTester::resetClass();

    EXPECT_THROW(TypeCast<int>(joint), std::bad_cast);

    EXPECT_EQ(0, test::sans_none::JointTester::defaultConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyAssignmentCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveAssignmentCalled);
}

TEST(Joint, TypeCastWithSansNoneToPointer)
{
    constexpr auto number = 10;
    auto pointer = static_cast<test::sans_none::JointTester*>(nullptr);
    const auto original = [&](){test::sans_none::JointTester v; v.number = number; return v;}();
    ASSERT_EQ(original.number, number);
    auto joint = Joint{original};
    test::sans_none::JointTester::resetClass();

    EXPECT_NO_THROW(pointer = TypeCast<test::sans_none::JointTester>(&joint));

    EXPECT_EQ(0, test::sans_none::JointTester::defaultConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyAssignmentCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveAssignmentCalled);
    ASSERT_TRUE(pointer != nullptr);
    EXPECT_EQ(number, pointer->number);
}

TEST(Joint, AnyCastWithSansNoneToValue)
{
    constexpr auto number = 10;
    auto value = test::sans_none::JointTester{};
    const auto original = [&](){test::sans_none::JointTester v; v.number = number; return v;}();
    EXPECT_EQ(original.number, number);
    auto anyObject = std::any{original};
    test::sans_none::JointTester::resetClass();

    EXPECT_NO_THROW(value = std::any_cast<test::sans_none::JointTester>(anyObject));

    EXPECT_EQ(0, test::sans_none::JointTester::defaultConstructorCalled);
    EXPECT_EQ(1, test::sans_none::JointTester::copyConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyAssignmentCalled);
    EXPECT_EQ(1, test::sans_none::JointTester::moveAssignmentCalled);
    EXPECT_EQ(number, value.number);
}

TEST(Joint, TypeCastWithSansNoneToValue)
{
    constexpr auto number = 10;
    auto value = test::sans_none::JointTester{};
    const auto original = [&](){test::sans_none::JointTester v; v.number = number; return v;}();
    EXPECT_EQ(original.number, number);
    auto jointObject = Joint{original};
    test::sans_none::JointTester::resetClass();

    EXPECT_NO_THROW(value = TypeCast<test::sans_none::JointTester>(jointObject));

    // Seeing both a copy and a move looks sub-optimal to me; what of C++17 copy elision?
    // It's also the case for std::any however as seen in the other test.
    EXPECT_EQ(0, test::sans_none::JointTester::defaultConstructorCalled);
    EXPECT_EQ(1, test::sans_none::JointTester::copyConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::moveConstructorCalled);
    EXPECT_EQ(0, test::sans_none::JointTester::copyAssignmentCalled);
    EXPECT_EQ(1, test::sans_none::JointTester::moveAssignmentCalled);
    EXPECT_EQ(number, value.number);
}

TEST(Joint, TypeCastWithSansNone)
{
    constexpr auto number = 10;
    auto value = test::sans_none::JointTester{};
    const auto original = [&](){test::sans_none::JointTester v; v.number = number; return v;}();
    EXPECT_EQ(original.number, number);
    auto joint = Joint{original};

    EXPECT_NO_THROW(TypeCast<test::sans_none::JointTester&>(joint).number = 3);
    EXPECT_EQ(TypeCast<const test::sans_none::JointTester&>(joint).number, 3);
    EXPECT_NO_THROW(value = TypeCast<test::sans_none::JointTester>(joint));
    EXPECT_EQ(value.number, 3);
    EXPECT_NO_THROW(TypeCast<test::sans_none::JointTester>(&joint)->number = 4);
    EXPECT_EQ(TypeCast<const test::sans_none::JointTester>(joint).number, 4);
    EXPECT_TRUE(joint == joint);
    EXPECT_FALSE(joint != joint);
    EXPECT_EQ(GetBodyA(joint), InvalidBodyID);
    EXPECT_EQ(GetBodyB(joint), InvalidBodyID);
    EXPECT_FALSE(GetCollideConnected(joint));
}

TEST(Joint, ForConstantDataTypeCastIsLikeAnyCast)
{
    const auto foo = Joint{[](){test::sans_none::JointTester v; v.number = 1; return v;}()};
    const auto bar = std::any{[](){test::sans_none::JointTester v; v.number = 1; return v;}()};
    EXPECT_TRUE(TypeCast<const test::sans_none::JointTester*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<const test::sans_none::JointTester*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<test::sans_none::JointTester*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<test::sans_none::JointTester*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<const test::sans_none::JointTester>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<const test::sans_none::JointTester>(&bar) != nullptr);
    EXPECT_TRUE(TypeCast<test::sans_none::JointTester>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<test::sans_none::JointTester>(&bar) != nullptr);
}

TEST(Joint, ForMutableDataTypeCastIsLikeAnyCast)
{
    auto foo = Joint{[](){test::sans_none::JointTester v; v.number = 1; return v;}()};
    auto bar = std::any{[](){test::sans_none::JointTester v; v.number = 1; return v;}()};
    EXPECT_TRUE(TypeCast<const test::sans_none::JointTester*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<const test::sans_none::JointTester*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<test::sans_none::JointTester*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<test::sans_none::JointTester*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<const test::sans_none::JointTester>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<const test::sans_none::JointTester>(&bar) != nullptr);
    EXPECT_TRUE(TypeCast<test::sans_none::JointTester>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<test::sans_none::JointTester>(&bar) != nullptr);
}

TEST(Joint, GetLinearReaction)
{
    auto result = Momentum2{};
    EXPECT_NO_THROW(result = GetLinearReaction(Joint{}));
    EXPECT_EQ(result, Momentum2());
}

TEST(Joint, GetAngularReaction)
{
    auto result = AngularMomentum{};
    EXPECT_NO_THROW(result = GetAngularReaction(Joint{}));
    EXPECT_EQ(result, AngularMomentum());
}

TEST(Joint, SetMotorSpeedThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(SetMotorSpeed(joint, 1_rpm), std::invalid_argument);
}

TEST(Joint, SetMaxMotorForceThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(SetMaxMotorForce(joint, 2_N), std::invalid_argument);
}

TEST(Joint, SetMaxMotorTorqueThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(SetMaxMotorTorque(joint, 2_Nm), std::invalid_argument);
}

TEST(Joint, SetLinearLimitsThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(SetLinearLimits(joint, 2_m, 4_m), std::invalid_argument);
}

TEST(Joint, SetAngularLimitsThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(SetAngularLimits(joint, 2_deg, 4_deg), std::invalid_argument);
}

TEST(Joint, IsLimitEnabledThrows)
{
    EXPECT_THROW(IsLimitEnabled(Joint{}), std::invalid_argument);
}

TEST(Joint, EnableLimitThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(EnableLimit(joint, true), std::invalid_argument);
    EXPECT_THROW(EnableLimit(joint, false), std::invalid_argument);
}

TEST(Joint, IsMotorEnabledThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(IsMotorEnabled(joint), std::invalid_argument);
}

TEST(Joint, EnableMotorThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(EnableMotor(joint, true), std::invalid_argument);
    EXPECT_THROW(EnableMotor(joint, false), std::invalid_argument);
}

TEST(Joint, GetLimitStateThrows)
{
    auto joint = Joint{};
    EXPECT_THROW(GetLimitState(joint), std::invalid_argument);
}

TEST(Joint, EqualsOperator)
{
    const auto j0 = Joint(WheelJointConf());
    EXPECT_TRUE(j0 == j0);
    {
        auto conf = WheelJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        const auto j1 = Joint(conf);
        EXPECT_TRUE(j1 == j1);
        EXPECT_FALSE(j0 == j1);
    }
    {
        auto conf = WheelJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        const auto j1 = Joint(conf);
        EXPECT_TRUE(j1 == j1);
        EXPECT_FALSE(j0 == j1);
    }
    {
        auto conf = WheelJointConf{};
        conf.motorSpeed = 0.12_rpm;
        const auto j1 = Joint(conf);
        EXPECT_TRUE(j1 == j1);
        EXPECT_FALSE(j0 == j1);
    }
    // TODO: test remaining fields.
}

TEST(Joint, NotEqualsOperator)
{
    const auto j0 = Joint(WheelJointConf());
    EXPECT_FALSE(j0 != j0);
    {
        auto conf = WheelJointConf{};
        conf.frequency = 13_Hz;
        const auto j1 = Joint(conf);
        EXPECT_FALSE(j1 != j1);
        EXPECT_TRUE(j0 != j1);
    }
    // TODO: test remaining fields.
}
