/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <Box2D/Common/Settings.hpp>

using namespace box2d;

TEST(RealNum, ByteSizeIs_4_8_or_16)
{
    const auto size = sizeof(RealNum);
    EXPECT_TRUE(size == size_t(4) || size == size_t(8) || size == size_t(16));
}

#if 0
TEST(RealNum, BiggerValsIdenticallyInaccurate)
{
    // Check that RealNum doesn't suffer from inconstent inaccuracy (like float has depending on
    // the float's value).
    auto last_delta = float(0);
    auto val = RealNum(1);
    for (auto i = 0; i < 16; ++i)
    {
        const auto next = std::nextafter(val, std::numeric_limits<decltype(val)>::max());
        const auto delta = static_cast<float>(next - val);
        ASSERT_EQ(val + (delta / 2.0f), val);
#if 0
        std::cout << std::hexfloat;
        std::cout << "For " << std::setw(7) << val << ", delta of next value is " << std::setw(7) << delta;
        std::cout << std::defaultfloat;
        std::cout << ": ie. at " << std::setw(6) << val;
        std::cout << std::fixed;
        std::cout << ", delta is " << delta;
        std::cout << std::endl;
#endif
        val *= 2;
        if (last_delta != 0)
        {
            ASSERT_EQ(delta, last_delta);
        }
        last_delta = delta;
    }
}
#endif

TEST(RealNum, beta0)
{
    RealNum zero{0};
    RealNum one{1};
    const auto beta = std::nextafter(zero, one);
    const auto coefficient0 = 1 - beta;
    const auto coefficient1 = beta;
    EXPECT_EQ(coefficient0 + coefficient1, one);
}

TEST(RealNum, beta1)
{
    RealNum zero{0};
    RealNum one{1};
    const auto beta = std::nextafter(one, zero);
    const auto coefficient0 = 1 - beta;
    const auto coefficient1 = beta;
    EXPECT_EQ(coefficient0 + coefficient1, one);
}
