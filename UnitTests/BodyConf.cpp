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

#include <PlayRho/Dynamics/BodyConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(BodyConf, UseType)
{
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Static).type, BodyType::Static);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Dynamic).type, BodyType::Dynamic);
    EXPECT_EQ(BodyConf{}.UseType(BodyType::Kinematic).type, BodyType::Kinematic);
}

TEST(BodyConf, UsePosition)
{
    const auto p = Position{Length2{3_m, -4_m}, 22_deg};
    EXPECT_EQ(BodyConf{}.Use(p).location, p.linear);
    EXPECT_EQ(BodyConf{}.Use(p).angle, p.angular);
}

TEST(BodyConf, UseVelocity)
{
    const auto v = Velocity{LinearVelocity2{3_mps, -4_mps}, 22_rad / 1_s};
    EXPECT_EQ(BodyConf{}.Use(v).linearVelocity, v.linear);
    EXPECT_EQ(BodyConf{}.Use(v).angularVelocity, v.angular);
}
