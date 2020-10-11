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

#include <PlayRho/Dynamics/Joints/JointType.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointConf.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJointConf.hpp>
#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>
#include <PlayRho/Dynamics/Joints/MotorJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RopeJointConf.hpp>
#include <PlayRho/Dynamics/Joints/TargetJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WeldJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WheelJointConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(JointType, ToString)
{
    EXPECT_STREQ(ToString(GetTypeID<RevoluteJointConf>()), "d2::RevoluteJointConf");
    EXPECT_STREQ(ToString(GetTypeID<PrismaticJointConf>()), "d2::PrismaticJointConf");
    EXPECT_STREQ(ToString(GetTypeID<DistanceJointConf>()), "d2::DistanceJointConf");
    EXPECT_STREQ(ToString(GetTypeID<PulleyJointConf>()), "d2::PulleyJointConf");
    EXPECT_STREQ(ToString(GetTypeID<TargetJointConf>()), "d2::TargetJointConf");
    EXPECT_STREQ(ToString(GetTypeID<GearJointConf>()), "d2::GearJointConf");
    EXPECT_STREQ(ToString(GetTypeID<WheelJointConf>()), "d2::WheelJointConf");
    EXPECT_STREQ(ToString(GetTypeID<WeldJointConf>()), "d2::WeldJointConf");
    EXPECT_STREQ(ToString(GetTypeID<FrictionJointConf>()), "d2::FrictionJointConf");
    EXPECT_STREQ(ToString(GetTypeID<RopeJointConf>()), "d2::RopeJointConf");
    EXPECT_STREQ(ToString(GetTypeID<MotorJointConf>()), "d2::MotorJointConf");
}
