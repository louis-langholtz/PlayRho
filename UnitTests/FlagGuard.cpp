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
#include <cstdint>
#include <cstddef>
#include <PlayRho/Common/FlagGuard.hpp>

using namespace playrho;

TEST(FlagGuard, ByteSizeOverhead)
{
    EXPECT_EQ(sizeof(FlagGuard<std::uint8_t>), sizeof(std::uint8_t*) * 2);
    EXPECT_EQ(sizeof(FlagGuard<std::uint16_t>), sizeof(std::uint16_t*) * 2);
    EXPECT_EQ(sizeof(FlagGuard<std::uint32_t>), sizeof(std::uint32_t*) * 2);
}

TEST(FlagGuard, uint8_t)
{
    std::uint8_t foo = 0;
    ASSERT_EQ(foo, 0);
    {
        FlagGuard<decltype(foo)> guard(foo, 0x1u);
        EXPECT_EQ(foo, 0x1u);
    }
    EXPECT_EQ(foo, 0x0u);
    foo = 0x44u;
    ASSERT_EQ(foo, 0x44u);
    {
        FlagGuard<decltype(foo)> guard(foo, 0x11u);
        EXPECT_EQ(foo, (0x11u | 0x44u));
    }
    EXPECT_EQ(foo, 0x44u);
}
