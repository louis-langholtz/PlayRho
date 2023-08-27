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

#include <PlayRho/d2/Velocity.hpp>
#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/MovementConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Velocity, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(Velocity), std::size_t(12));
        break;
    case 8:
        EXPECT_EQ(sizeof(Velocity), std::size_t(24));
        break;
    case 16:
        EXPECT_EQ(sizeof(Velocity), std::size_t(48));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(Velocity, CapZeroTimeNoCapStaysSame)
{
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{}, 0_rpm}, 0_s, MovementConf{}).linear,
              (LinearVelocity2{}));
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{}, 0_rpm}, 0_s, MovementConf{}).angular, 0_rpm);
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{+1_mps, +2_mps}, +3_rpm}, 0_s, MovementConf{}).linear,
              (LinearVelocity2{+1_mps, +2_mps}));
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{+1_mps, +2_mps}, +3_rpm}, 0_s, MovementConf{}).angular,
              +3_rpm);
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{-1_mps, -2_mps}, -3_rpm}, 0_s, MovementConf{}).linear,
              (LinearVelocity2{-1_mps, -2_mps}));
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{-1_mps, -2_mps}, -3_rpm}, 0_s, MovementConf{}).angular,
              -3_rpm);
}

TEST(Velocity, CapZeroConfNonZeroTimeGoesToZero)
{
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{}, 0_rpm}, 1_s, MovementConf{}).linear,
              (LinearVelocity2{}));
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{}, 0_rpm}, 1_s, MovementConf{}).angular, 0_rpm);
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{10_mps, 20_mps}, 10_rpm}, 1_s, MovementConf{}).linear,
              (LinearVelocity2{0_mps, 0_mps}));
    EXPECT_EQ(Cap(Velocity{LinearVelocity2{10_mps, 20_mps}, 10_rpm}, 1_s, MovementConf{}).angular,
              0_rpm);
}
