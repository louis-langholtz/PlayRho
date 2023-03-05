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
#include <PlayRho/Dynamics/Filter.hpp>

using namespace playrho;

TEST(Filter, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    EXPECT_EQ(sizeof(Filter), std::size_t(6));
}

TEST(Filter, DefaultConstruction)
{
    Filter filter;
    EXPECT_EQ(filter.categoryBits, Filter::bits_type(1));
    EXPECT_EQ(filter.maskBits, Filter::bits_type(0xFFFF));
    EXPECT_EQ(filter.groupIndex, Filter::index_type(0));
}

TEST(Filter, Construction)
{
    const auto category = Filter::bits_type(3);
    const auto mask = Filter::bits_type(0xFF01);
    const auto groupIndex = Filter::index_type(5);
    const auto filter = Filter{category, mask, groupIndex};

    EXPECT_EQ(filter.categoryBits, category);
    EXPECT_EQ(filter.maskBits, mask);
    EXPECT_EQ(filter.groupIndex, groupIndex);
}

TEST(Filter, ShouldCollide)
{
    EXPECT_TRUE(ShouldCollide(Filter{}, Filter{}));

    const auto category = Filter::bits_type(3);
    const auto mask = Filter::bits_type(0xFF01);
    const auto groupIndex = Filter::index_type(1);
    auto filter = Filter{category, mask, groupIndex};
    
    EXPECT_TRUE(ShouldCollide(filter, filter));
    
    filter.groupIndex = Filter::index_type(-1);
    EXPECT_FALSE(ShouldCollide(filter, filter));
}

TEST(Filter, Equals)
{
    EXPECT_TRUE(  Filter{} == Filter{});
    EXPECT_TRUE( (Filter{0x1u, 0x2u, -3}) == (Filter{0x1u, 0x2u, -3}));
    EXPECT_FALSE((Filter{0x1u, 0x2u, -3}) == (Filter{0x3u, 0x2u, -1}));
}

TEST(Filter, NotEquals)
{
    EXPECT_FALSE( Filter{} != Filter{});
    EXPECT_FALSE((Filter{0x1u, 0x2u, -3}) != (Filter{0x1u, 0x2u, -3}));
    EXPECT_TRUE( (Filter{0x1u, 0x2u, -3}) != (Filter{0x3u, 0x2u, -1}));
}
