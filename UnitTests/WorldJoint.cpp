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

#include <PlayRho/Dynamics/WorldJoint.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldJoint, GetSetMotorSpeed)
{
    World world;
    const auto b0 = world.CreateBody();
    const auto b1 = world.CreateBody();

    const auto motorSpeed = 4_rpm;
    auto jd = RevoluteJointConf{};
    jd.bodyA = b0;
    jd.bodyB = b1;
    jd.localAnchorA = Length2(4_m, 5_m);
    jd.localAnchorB = Length2(6_m, 7_m);
    jd.motorSpeed = motorSpeed;

    const auto newValue = Real(5) * RadianPerSecond;
    const auto id = CreateJoint(world, jd);
    ASSERT_NE(GetMotorSpeed(world, id), newValue);
    EXPECT_EQ(GetMotorSpeed(world, id), motorSpeed);
    EXPECT_NO_THROW(SetMotorSpeed(world, id, newValue));
    EXPECT_EQ(GetMotorSpeed(world, id), newValue);
    EXPECT_THROW(GetLocalXAxisA(world, id), std::invalid_argument);
    EXPECT_THROW(GetLocalYAxisA(world, id), std::invalid_argument);
}

TEST(WorldJoint, GetWorldIndexFreeFunction)
{
    World world;
    EXPECT_EQ(GetWorldIndex(world, InvalidJointID), JointCounter(-1));
}
