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
#include <PlayRho/Common/Real.hpp>
#include <PlayRho/Common/Settings.hpp>
#include <string>

using namespace playrho;

TEST(Real, ByteSizeIs_4_8_or_16)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    const auto size = sizeof(Real);
    EXPECT_TRUE(size == std::size_t(4) || size == std::size_t(8) || size == std::size_t(16));
}

TEST(Real, GetTypeName)
{
    const auto name = std::string(GetTypeName<Real>());
    
    auto is_expected = false;
    do
    {
        if (name == "float")
        {
            is_expected = true;
            break;
        }
        if (name == "double")
        {
            is_expected = true;
            break;
        }
        if (name == "long double")
        {
            is_expected = true;
            break;
        }
        if (name == "Fixed32")
        {
            is_expected = true;
            break;
        }
        if (name == "Fixed64")
        {
            is_expected = true;
            break;
        }
    } while (false);
    
    EXPECT_TRUE(is_expected);
}

#if 0
TEST(Real, BiggerValsIdenticallyInaccurate)
{
    // Check that Real doesn't suffer from inconstent inaccuracy (like float has depending on
    // the float's value).
    auto last_delta = float(0);
    auto val = Real(1);
    for (auto i = 0; i < 16; ++i)
    {
        const auto next = nextafter(val, std::numeric_limits<decltype(val)>::max());
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

TEST(Real, beta0)
{
    Real zero{0};
    Real one{1};
    const auto beta = nextafter(zero, one);
    const auto coefficient0 = 1 - beta;
    const auto coefficient1 = beta;
    EXPECT_EQ(coefficient0 + coefficient1, one);
}

TEST(Real, beta1)
{
    Real zero{0};
    Real one{1};
    const auto beta = nextafter(one, zero);
    const auto coefficient0 = 1 - beta;
    const auto coefficient1 = beta;
    EXPECT_EQ(coefficient0 + coefficient1, one);
}
