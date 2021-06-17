/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Common/Position.hpp>

#include <PlayRho/Common/Math.hpp>

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

TEST(Position, GetPosition)
{
    EXPECT_EQ(GetPosition(Position{}, Position{}, Real(0.0)), (Position{}));
    EXPECT_EQ(GetPosition(Position{}, Position{Length2{2_m, 2_m}, 2_rad}, Real(0.0)),
              (Position{Length2{0_m, 0_m}, 0_rad}));
    EXPECT_EQ(GetPosition(Position{}, Position{Length2{2_m, 2_m}, 2_rad}, Real(0.5)),
              (Position{Length2{1_m, 1_m}, 1_rad}));
    EXPECT_EQ(GetPosition(Position{}, Position{Length2{2_m, 2_m}, 2_rad}, Real(1.0)),
              (Position{Length2{2_m, 2_m}, 2_rad}));

    // Test a case that's maybe less obvious...
    // See https://github.com/louis-langholtz/PlayRho/issues/331#issuecomment-507412550
    const auto p0 = Position{Length2{-0.1615_m, -10.2494_m}, -3.1354_rad};
    const auto p1 = Position{Length2{-0.3850_m, -10.1851_m}, +3.1258_rad};
    const auto p = GetPosition(p0, p1, Real(0.2580));
    constexpr auto abserr = 0.000001;
    EXPECT_NEAR(static_cast<double>(Real(GetX(p.linear) / 1_m)), -0.21916300, abserr);
    EXPECT_NEAR(static_cast<double>(Real(GetY(p.linear) / 1_m)), -10.232810974121094, abserr);
    EXPECT_NEAR(static_cast<double>(Real(p.angular / 1_rad)), -1.52001, abserr);
}
