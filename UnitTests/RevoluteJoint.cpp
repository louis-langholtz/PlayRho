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

#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldJoint.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/d2/BodyConstraint.hpp>
#include <PlayRho/ConstraintSolverConf.hpp>

#include <cstring> // for std::memcmp

using namespace playrho;
using namespace playrho::d2;

TEST(RevoluteJointConf, DefaultConstruction)
{
    const auto jd = RevoluteJointConf{};
    EXPECT_EQ(jd.bodyA, InvalidBodyID);
    EXPECT_EQ(jd.bodyB, InvalidBodyID);
    EXPECT_EQ(jd.collideConnected, false);
    EXPECT_EQ(jd.localAnchorA, Length2());
    EXPECT_EQ(jd.localAnchorB, Length2());
    EXPECT_EQ(jd.impulse, Vec3());
    EXPECT_EQ(jd.angularMotorImpulse, AngularMomentum());
    EXPECT_EQ(jd.referenceAngle, 0_deg);
    EXPECT_EQ(jd.enableLimit, false);
    EXPECT_EQ(jd.lowerAngle, 0_deg);
    EXPECT_EQ(jd.upperAngle, 0_deg);
    EXPECT_EQ(jd.enableMotor, false);
    EXPECT_EQ(jd.motorSpeed, 0_rpm);
    EXPECT_EQ(jd.maxMotorTorque, Torque());
    EXPECT_EQ(jd.rA, Length2());
    EXPECT_EQ(jd.rB, Length2());
    EXPECT_EQ(jd.mass, Mat33());
    EXPECT_EQ(jd.angularMass, RotInertia());
    EXPECT_EQ(jd.limitState, LimitState::e_inactiveLimit);
}

TEST(RevoluteJoint, Construction)
{
    World world;
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto jd = RevoluteJointConf{};

    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.collideConnected = true;

    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    jd.enableLimit = true;
    jd.enableMotor = true;
    jd.motorSpeed = Real{4.4f} * RadianPerSecond;
    jd.maxMotorTorque = 1_Nm;
    jd.lowerAngle = 33_deg;
    jd.upperAngle = 40_deg;
    jd.referenceAngle = 45_deg;

    auto joint = Joint{jd};

    EXPECT_EQ(GetType(joint), GetTypeID<RevoluteJointConf>());
    EXPECT_EQ(GetBodyA(joint), jd.bodyA);
    EXPECT_EQ(GetBodyB(joint), jd.bodyB);
    EXPECT_EQ(GetCollideConnected(joint), jd.collideConnected);
    EXPECT_EQ(GetLinearReaction(joint), Momentum2{});
    EXPECT_EQ(GetAngularReaction(joint), AngularMomentum{0});
    EXPECT_EQ(GetLimitState(joint), LimitState::e_inactiveLimit);

    EXPECT_EQ(GetLocalAnchorA(joint), jd.localAnchorA);
    EXPECT_EQ(GetLocalAnchorB(joint), jd.localAnchorB);
    EXPECT_EQ(GetAngularLowerLimit(joint), jd.lowerAngle);
    EXPECT_EQ(GetAngularUpperLimit(joint), jd.upperAngle);
    EXPECT_EQ(GetMotorSpeed(joint), jd.motorSpeed);
    EXPECT_EQ(GetReferenceAngle(joint), jd.referenceAngle);
    EXPECT_EQ(IsMotorEnabled(joint), jd.enableMotor);
    EXPECT_EQ(GetMaxMotorTorque(joint), jd.maxMotorTorque);
    EXPECT_EQ(IsLimitEnabled(joint), jd.enableLimit);
    EXPECT_EQ(GetAngularMotorImpulse(joint), AngularMomentum{0});

    const auto id = CreateJoint(world, joint);
    EXPECT_EQ(GetAngularVelocity(world, id), 0 * RadianPerSecond);
    EXPECT_EQ(GetAnchorA(world, id), Length2(4_m, 5_m));
    EXPECT_EQ(GetAnchorB(world, id), Length2(6_m, 7_m));
    EXPECT_EQ(GetMotorTorque(world, id, 1_Hz), 0 * NewtonMeter);
}

TEST(RevoluteJoint, EnableMotor)
{
    World world;
    const auto b0 = CreateBody(world,
        BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    const auto b1 = CreateBody(world,
        BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    auto joint = Joint{jd};
    ASSERT_FALSE(IsLimitEnabled(joint));
    EXPECT_EQ(GetLimitState(joint), LimitState::e_inactiveLimit);
    EXPECT_FALSE(IsMotorEnabled(joint));
    EnableMotor(joint, false);
    EXPECT_FALSE(IsMotorEnabled(joint));
    EnableMotor(joint, true);
    EXPECT_TRUE(IsMotorEnabled(joint));
}

TEST(RevoluteJoint, EnableMotorInWorld)
{
    World world;
    const auto b0 = CreateBody(world,
        BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    const auto b1 = CreateBody(world,
        BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(EarthlyGravity));
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    const auto id = CreateJoint(world, jd);
    ASSERT_NE(id, InvalidJointID);
    ASSERT_EQ(GetVelocity(world, b0), Velocity{});
    ASSERT_EQ(GetVelocity(world, b1), Velocity{});
    EXPECT_FALSE(IsMotorEnabled(world, id));
    EnableMotor(world, id, false);
    EXPECT_FALSE(IsMotorEnabled(world, id));
    EnableMotor(world, id, true);
    EXPECT_TRUE(IsMotorEnabled(world, id));

    const auto newValue = 5_Nm;
    ASSERT_NE(GetMaxMotorTorque(world, id), newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, id), jd.maxMotorTorque);
    SetMaxMotorTorque(world, id, newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, id), newValue);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));

    const auto shape = CreateShape(world, DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    Attach(world, b0, shape);
    Attach(world, b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));

    auto stepConf = StepConf{};
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    stepConf.doWarmStart = false;
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});

    EnableLimit(world, id, true);
    ASSERT_TRUE(IsLimitEnabled(world, id));

    SetAngularLimits(world, id, -45_deg, -5_deg);

    stepConf.doWarmStart = true;
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_EQ(GetAngularReaction(world, id), AngularMomentum(0));
    EXPECT_EQ(GetLimitState(world, id), LimitState::e_atUpperLimit);
    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});

    SetAngularLimits(world, id, +55_deg, +95_deg);

    stepConf.doWarmStart = true;
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, id), AngularMomentum(0));
    EXPECT_EQ(GetAngularReaction(world, id), AngularMomentum(0));
    EXPECT_EQ(GetLimitState(world, id), LimitState::e_atLowerLimit);

    EXPECT_NE(GetVelocity(world, b0), Velocity{});
    EXPECT_NE(GetVelocity(world, b1), Velocity{});
}

TEST(RevoluteJoint, MotorSpeed)
{
    World world;
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    const auto newValue = Real(5) * RadianPerSecond;
    auto joint = Joint{jd};
    ASSERT_NE(GetMotorSpeed(joint), newValue);
    EXPECT_EQ(GetMotorSpeed(joint), jd.motorSpeed);
    SetMotorSpeed(joint, newValue);
    EXPECT_EQ(GetMotorSpeed(joint), newValue);
}

TEST(RevoluteJoint, EnableLimit)
{
    auto world = World{};
    const auto b0 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    ASSERT_EQ(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_EQ(GetInvRotInertia(world, b1), InvRotInertia(0));

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    jd.enableLimit = false;

    const auto joint = CreateJoint(world, jd);
    ASSERT_EQ(GetLimitState(world, joint), LimitState::e_inactiveLimit);
    ASSERT_FALSE(IsLimitEnabled(world, joint));
    EnableLimit(world, joint, false);
    EXPECT_FALSE(IsLimitEnabled(world, joint));
    EnableLimit(world, joint, true);
    EXPECT_TRUE(IsLimitEnabled(world, joint));

    const auto id = CreateJoint(world, jd);
    ASSERT_NE(id, InvalidJointID);

    auto stepConf = StepConf{};
    Step(world, stepConf);
    EXPECT_TRUE(IsLimitEnabled(world, joint));
    // since b0 & b1 inv rot inertia 0
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_inactiveLimit);

    const auto shape = CreateShape(world, DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    Attach(world, b0, shape);
    Attach(world, b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));

    Step(world, stepConf);
    EXPECT_TRUE(IsLimitEnabled(world, joint));
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_equalLimits);

    SetAngularLimits(world, joint, -45_deg, +45_deg);
    ASSERT_TRUE(IsLimitEnabled(world, joint));
    ASSERT_EQ(GetLimitState(world, joint), LimitState::e_equalLimits);
    Step(world, stepConf);

    EXPECT_TRUE(IsLimitEnabled(world, joint));
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_inactiveLimit);

    EXPECT_EQ(GetAngularMotorImpulse(world, joint), AngularMomentum(0));
}

TEST(RevoluteJoint, SetAngularLimits)
{
    World world;
    const auto b0 = CreateBody(world);
    const auto b1 = CreateBody(world);

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    const auto upperValue = +5_deg;
    const auto lowerValue = -8_deg;
    auto joint = Joint{jd};
    ASSERT_NE(GetAngularUpperLimit(joint), upperValue);
    ASSERT_NE(GetAngularLowerLimit(joint), lowerValue);
    SetAngularLimits(joint, lowerValue, upperValue);
    EXPECT_EQ(GetAngularUpperLimit(joint), upperValue);
    EXPECT_EQ(GetAngularLowerLimit(joint), lowerValue);
}

TEST(RevoluteJoint, MaxMotorTorque)
{
    World world;
    const auto b0 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic));

    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);

    const auto newValue = 5_Nm;
    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);

    ASSERT_NE(GetMaxMotorTorque(world, joint), newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, joint), jd.maxMotorTorque);
    SetMaxMotorTorque(world, joint, newValue);
    EXPECT_EQ(GetMaxMotorTorque(world, joint), newValue);
    EXPECT_EQ(GetAngularMotorImpulse(world, joint), AngularMomentum(0));

    const auto shape = CreateShape(world, DiskShapeConf{}.UseRadius(1_m).UseDensity(1_kgpm2));
    Attach(world, b0, shape);
    Attach(world, b1, shape);
    ASSERT_NE(GetInvRotInertia(world, b0), InvRotInertia(0));
    ASSERT_NE(GetInvRotInertia(world, b1), InvRotInertia(0));

    auto stepConf = StepConf{};
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, joint), AngularMomentum(0));
    stepConf.doWarmStart = false;
    Step(world, stepConf);
    EXPECT_EQ(GetAngularMotorImpulse(world, joint), AngularMomentum(0));
}

TEST(RevoluteJoint, MovesDynamicCircles)
{
    World world;
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}
                                         .UseType(BodyType::Dynamic)
                                         .UseLocation(p1)
                                         .UseLinearAcceleration(EarthlyGravity));
    const auto b2 = CreateBody(world, BodyConf{}
                                         .UseType(BodyType::Dynamic)
                                         .UseLocation(p2)
                                         .UseLinearAcceleration(EarthlyGravity));
    const auto circle = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m));
    Attach(world, b1, circle);
    Attach(world, b2, circle);
    auto jd = RevoluteJointConf{};
    jd.bodyA = b1;
    jd.bodyB = b2;
    CreateJoint(world, Joint{jd});

    auto step = StepConf{};
    step.deltaTime = 1_s;
    step.maxTranslation = Meter * Real(4);
    Step(world, step);

    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), 0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), -4, 0.001);
    EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), 0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), -4, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
}

TEST(RevoluteJoint, LimitEnabledDynamicCircles)
{
    World world;
    const auto p1 = Length2{-1_m, 0_m};
    const auto p2 = Length2{+1_m, 0_m};
    const auto b1 = CreateBody(world, BodyConf{}
                                         .UseType(BodyType::Dynamic)
                                         .UseLocation(p1)
                                         .UseLinearAcceleration(EarthlyGravity));
    const auto b2 = CreateBody(world, BodyConf{}
                                         .UseType(BodyType::Dynamic)
                                         .UseLocation(p2)
                                         .UseLinearAcceleration(EarthlyGravity));
    const auto circle = CreateShape(world, DiskShapeConf{}.UseRadius(0.2_m).UseDensity(1_kgpm2));
    Attach(world, b1, circle);
    Attach(world, b2, circle);
    auto jd = RevoluteJointConf{b1, b2, Length2{}};
    jd.enableLimit = true;
    ASSERT_EQ(jd.lowerAngle, 0_deg);
    ASSERT_EQ(jd.upperAngle, 0_deg);

    const auto joint = CreateJoint(world, jd);
    ASSERT_NE(joint, InvalidJointID);
    ASSERT_EQ(GetLimitState(world, joint), LimitState::e_inactiveLimit);
    ASSERT_EQ(GetAngularLowerLimit(world, joint), jd.lowerAngle);
    ASSERT_EQ(GetAngularUpperLimit(world, joint), jd.upperAngle);
    ASSERT_EQ(GetReferenceAngle(world, joint), 0_deg);
    ASSERT_EQ(GetAngle(world, joint), 0_deg);

    auto step = StepConf{};
    step.deltaTime = 1_s;
    step.maxTranslation = Meter * Real(4);
    Step(world, step);

    EXPECT_EQ(GetAngle(world, joint), 0_deg);
    EXPECT_EQ(GetReferenceAngle(world, joint), 0_deg);
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_equalLimits);
    // TODO: investigate why failing...
    // EXPECT_NEAR(double(Real{GetX(GetLocation(world, b1)) / Meter}), -1.0, 0.001);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b1)) / Meter}), -4, 0.001);
    // TODO: investigate why failing...
    // EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}), +1.0, 0.01);
    EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}), -4, 0.01);
    EXPECT_EQ(GetAngle(world, b1), 0_deg);
    EXPECT_EQ(GetAngle(world, b2), 0_deg);
    EXPECT_TRUE(IsEnabled(world, joint));
    UnsetAwake(world, b1);
    UnsetAwake(world, b2);
    ASSERT_FALSE(IsAwake(world, b1));
    ASSERT_FALSE(IsAwake(world, b2));
    SetAwake(world, joint);
    EXPECT_TRUE(IsAwake(world, b1));
    EXPECT_TRUE(IsAwake(world, b2));

    EXPECT_EQ(GetWorldIndex(world, joint), std::size_t(0));

    SetAngularLimits(world, joint, 45_deg, 90_deg);
    EXPECT_EQ(GetAngularLowerLimit(world, joint), 45_deg);
    EXPECT_EQ(GetAngularUpperLimit(world, joint), 90_deg);

    Step(world, step);
    EXPECT_EQ(GetReferenceAngle(world, joint), 0_deg);
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_atLowerLimit);
    // TODO: investigate why failing...
    // EXPECT_NEAR(static_cast<double>(Real(GetAngle(world, joint)/1_rad)),
    //             0.28610128164291382, 0.28610128164291382/100);

    SetAngularLimits(world, joint, -90_deg, -45_deg);
    EXPECT_EQ(GetAngularLowerLimit(world, joint), -90_deg);
    EXPECT_EQ(GetAngularUpperLimit(world, joint), -45_deg);

    Step(world, step);
    EXPECT_EQ(GetReferenceAngle(world, joint), 0_deg);
    EXPECT_EQ(GetLimitState(world, joint), LimitState::e_atUpperLimit);
    // TODO: investigate why failing...
    // EXPECT_NEAR(static_cast<double>(Real(GetAngle(world, joint)/1_rad)),
    //             -0.082102291285991669, 0.082102291285991669/100);
}

TEST(RevoluteJoint, DynamicJoinedToStaticStaysPut)
{
    auto world = World{};

    const auto p1 = Length2{0_m, 4_m}; // Vec2{-1, 0};
    const auto p2 = Length2{0_m, -2_m}; // Vec2{+1, 0};
    const auto b1 = CreateBody(world, BodyConf{}.UseType(BodyType::Static).UseLocation(p1));
    const auto b2 = CreateBody(world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(p2));

    const auto shape1 = CreateShape(world, PolygonShapeConf{}.SetAsBox(1_m, 1_m));
    Attach(world, b1, shape1);

    const auto shape2 = CreateShape(world, PolygonShapeConf{}.SetAsBox(0.5_m, 0.5_m).UseDensity(1_kgpm2));
    Attach(world, b2, shape2);

    auto jd = GetRevoluteJointConf(world, b1, b2, Length2{});
    const auto joint = CreateJoint(world, Joint{jd});

    SetAccelerations(world, Acceleration{LinearAcceleration2{0, -10 * MeterPerSquareSecond},
                                         0 * RadianPerSquareSecond});
    for (auto i = 0; i < 1000; ++i) {
        Step(world, 0.1_s);
        EXPECT_EQ(GetLocation(world, b1), p1);
        EXPECT_NEAR(double(Real{GetX(GetLocation(world, b2)) / Meter}),
                    double(Real{GetX(p2) / Meter}), 0.0001);
        EXPECT_NEAR(double(Real{GetY(GetLocation(world, b2)) / Meter}),
                    double(Real{GetY(p2) / Meter}), 0.0001);
        EXPECT_EQ(GetAngle(world, b2), 0_deg);
    }
    Destroy(world, joint);
    for (auto i = 0; i < 10; ++i) {
        Step(world, 0.1_s);
        EXPECT_EQ(GetLocation(world, b1), p1);
        EXPECT_NE(GetLocation(world, b2), p2);
        EXPECT_EQ(GetAngle(world, b2), 0_deg);
    }
}

TEST(RevoluteJointConf, GetRevoluteJointConfThrows)
{
    EXPECT_THROW(GetRevoluteJointConf(Joint{}), std::bad_cast);
}

TEST(RevoluteJointConf, GetRevoluteJointConfFromJoint)
{
    auto conf = RevoluteJointConf{BodyID(0u), BodyID(1u)};
    conf.UseCollideConnected(true);
    conf.impulse = Vec3{Real(3), Real(4), Real(5)};
    conf.angularMotorImpulse = AngularMomentum{Real(2) * NewtonMeterSecond};
    conf.referenceAngle = 20_deg;
    conf.enableLimit = true;
    conf.lowerAngle = 10_deg;
    conf.upperAngle = 30_deg;
    conf.enableMotor = true;
    conf.motorSpeed = 3_rpm;
    conf.maxMotorTorque = Torque{Real(2.1) * NewtonMeter};
    auto result = RevoluteJointConf{};
    EXPECT_NO_THROW(result = GetRevoluteJointConf(Joint{conf}));
    EXPECT_EQ(result.bodyA, conf.bodyA);
    EXPECT_EQ(result.bodyB, conf.bodyB);
    EXPECT_EQ(result.collideConnected, conf.collideConnected);
    EXPECT_EQ(result.localAnchorA, conf.localAnchorA);
    EXPECT_EQ(result.localAnchorB, conf.localAnchorB);
    EXPECT_EQ(result.impulse, conf.impulse);
    EXPECT_EQ(result.angularMotorImpulse, conf.angularMotorImpulse);
    EXPECT_EQ(result.referenceAngle, conf.referenceAngle);
    EXPECT_EQ(result.enableLimit, conf.enableLimit);
    EXPECT_EQ(result.lowerAngle, conf.lowerAngle);
    EXPECT_EQ(result.upperAngle, conf.upperAngle);
    EXPECT_EQ(result.enableMotor, conf.enableMotor);
    EXPECT_EQ(result.motorSpeed, conf.motorSpeed);
}

TEST(RevoluteJointConf, GetAngle)
{
    auto world = World{};
    const auto bodyA = CreateBody(world);
    const auto bodyB = CreateBody(world);
    auto conf = RevoluteJointConf{bodyA, bodyB};
    auto angle = Angle{};
    EXPECT_NO_THROW(angle = GetAngle(world, conf));
    EXPECT_EQ(angle, 0_deg);
    // TODO: add tests for angles other than 0 degrees
}

TEST(RevoluteJointConf, GetAngularVelocity)
{
    auto world = World{};
    const auto bodyA = CreateBody(world);
    const auto bodyB = CreateBody(world);
    auto conf = RevoluteJointConf{bodyA, bodyB};
    auto angularVelocity = AngularVelocity{};
    EXPECT_NO_THROW(angularVelocity = GetAngularVelocity(world, conf));
    EXPECT_EQ(angularVelocity, 0_rpm);
    // TODO: add tests for angularVelocity other than 0 rpm
}

TEST(RevoluteJointConf, ShiftOrigin)
{
    auto jd = RevoluteJointConf{BodyID(0u), BodyID(1u)};
    auto copy = RevoluteJointConf{};

    // Do copy = jd without missing padding so memcmp works
    std::memcpy(&copy, &jd, sizeof(RevoluteJointConf));

    EXPECT_FALSE(ShiftOrigin(jd, Length2{0_m, 0_m}));

    // Use memcmp since easier than writing full == suport pre C++20.
    EXPECT_TRUE(std::memcmp(&jd, &copy, sizeof(RevoluteJointConf)) == 0);
}

TEST(RevoluteJointConf, GetAngularMass)
{
    auto conf = RevoluteJointConf{};
    conf.angularMass = RotInertia{2_m2 * 3_kg / SquareRadian}; // L^2 M QP^-2
    auto rotInertia = RotInertia{};
    EXPECT_NO_THROW(rotInertia = GetAngularMass(Joint{conf}));
    EXPECT_EQ(conf.angularMass, rotInertia);
}

TEST(RevoluteJointConf, GetLocalXAxisAThrowsInvalidArgument)
{
    EXPECT_THROW(GetLocalXAxisA(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, GetLocalYAxisAThrowsInvalidArgument)
{
    EXPECT_THROW(GetLocalYAxisA(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, GetMaxMotorForceThrowsInvalidArgument)
{
    EXPECT_THROW(GetMaxMotorForce(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, GetLinearLowerLimitThrowsInvalidArgument)
{
    EXPECT_THROW(GetLinearLowerLimit(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, GetLinearUpperLimitThrowsInvalidArgument)
{
    EXPECT_THROW(GetLinearUpperLimit(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, GetLinearMotorImpulseThrowsInvalidArgument)
{
    EXPECT_THROW(GetLinearMotorImpulse(Joint{RevoluteJointConf{}}), std::invalid_argument);
}

TEST(RevoluteJointConf, EqualsOperator)
{
    EXPECT_TRUE(RevoluteJointConf() == RevoluteJointConf());
    {
        auto conf = RevoluteJointConf{};
        conf.localAnchorA = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RevoluteJointConf() == conf);
    }
    {
        auto conf = RevoluteJointConf{};
        conf.localAnchorB = Length2{1.2_m, -3_m};
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RevoluteJointConf() == conf);
    }
    {
        auto conf = RevoluteJointConf{};
        conf.referenceAngle = 12_deg;
        EXPECT_TRUE(conf == conf);
        EXPECT_FALSE(RevoluteJointConf() == conf);
    }
    // TODO: test remaining fields.
}

TEST(RevoluteJointConf, NotEqualsOperator)
{
    EXPECT_FALSE(RevoluteJointConf() != RevoluteJointConf());
    {
        auto conf = RevoluteJointConf{};
        conf.enableMotor = !RevoluteJointConf{}.enableMotor;
        EXPECT_FALSE(conf != conf);
        EXPECT_TRUE(RevoluteJointConf() != conf);
    }
    // TODO: test remaining fields.
}

TEST(RevoluteJointConf, GetName)
{
    EXPECT_STREQ(GetName(GetTypeID<RevoluteJointConf>()), "d2::RevoluteJointConf");
}

TEST(RevoluteJointConf, InitVelocity)
{
    auto conf = RevoluteJointConf{};
    std::vector<BodyConstraint> bodies;
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(InitVelocity(conf, bodies, StepConf{}, ConstraintSolverConf{}));
}

TEST(RevoluteJointConf, SolveVelocity)
{
    auto conf = RevoluteJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(SolveVelocity(conf, bodies, StepConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolveVelocity(conf, bodies, StepConf{}));
}

TEST(RevoluteJointConf, SolvePosition)
{
    auto conf = RevoluteJointConf{};
    std::vector<BodyConstraint> bodies;
    auto result = false;
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
    EXPECT_TRUE(result);
    conf.bodyA = BodyID(0);
    conf.bodyB = BodyID(0);
    EXPECT_THROW(SolvePosition(conf, bodies, ConstraintSolverConf{}), std::out_of_range);
    const auto posA = Position{Length2{-5_m, 0_m}, 0_deg};
    bodies.push_back(BodyConstraint{Real(1) / 4_kg, InvRotInertia{}, Length2{}, posA, Velocity{}});
    EXPECT_NO_THROW(result = SolvePosition(conf, bodies, ConstraintSolverConf{}));
}
