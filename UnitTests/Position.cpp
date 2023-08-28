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

#include <playrho/d2/Position.hpp>

#include <playrho/Math.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Position, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(Position), std::size_t(12));
        break;
    case 8:
        EXPECT_EQ(sizeof(Position), std::size_t(24));
        break;
    case 16:
        EXPECT_EQ(sizeof(Position), std::size_t(48));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(Position, EqualsOperator)
{
    const auto zero = Position{};
    EXPECT_TRUE(zero == zero);
    EXPECT_TRUE((Position{Length2{}, Angle{}} == zero));
    EXPECT_FALSE((Position{Length2{2_m, 0_m}, Angle{}} == zero));
    EXPECT_FALSE((Position{Length2{0_m, 2_m}, Angle{}} == zero));
    EXPECT_FALSE((Position{Length2{}, Real(2) * Radian} == zero));
}

TEST(Position, NotEqualsOperator)
{
    const auto zero = Position{};
    EXPECT_FALSE(zero != zero);
    EXPECT_FALSE((Position{Length2{}, Angle{}} != zero));
    EXPECT_TRUE((Position{Length2{2_m, 0_m}, Angle{}} != zero));
    EXPECT_TRUE((Position{Length2{0_m, 2_m}, Angle{}} != zero));
    EXPECT_TRUE((Position{Length2{}, Real(2) * Radian} != zero));
}

TEST(Position, Addition)
{
    EXPECT_EQ(Position{} + Position{}, Position{});
    EXPECT_EQ((Position{Length2{1_m, 1_m}, 1_rad}) + (Position{Length2{1_m, 1_m}, 1_rad}),
              (Position{Length2{2_m, 2_m}, 2_rad}));
}

TEST(Position, Subtraction)
{
    EXPECT_EQ(Position{} - Position{}, Position{});
    EXPECT_EQ((Position{Length2{1_m, 1_m}, 1_rad}) - (Position{Length2{1_m, 1_m}, 1_rad}),
              (Position{Length2{0_m, 0_m}, 0 * Radian}));
}

TEST(Position, Multiplication)
{
    EXPECT_EQ(Position{} * Real(2), Position{});
    EXPECT_EQ((Position{Length2{1_m, 1_m}, 1_rad}) * Real(2), (Position{Length2{2_m, 2_m}, 2_rad}));
    EXPECT_EQ(Real(2) * (Position{Length2{1_m, 1_m}, 1_rad}), (Position{Length2{2_m, 2_m}, 2_rad}));
}
