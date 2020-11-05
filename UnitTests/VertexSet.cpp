/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Common/VertexSet.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(VertexSet, Traits)
{
    EXPECT_TRUE((IsIterable<VertexSet>::value));
    EXPECT_FALSE((IsAddable<VertexSet>::value));
    EXPECT_FALSE((IsAddable<VertexSet,VertexSet>::value));
}

TEST(VertexSet, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(VertexSet), std::size_t(40));
#else
            EXPECT_EQ(sizeof(VertexSet), std::size_t(32));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(VertexSet), std::size_t(20));
#else
            EXPECT_EQ(sizeof(VertexSet), std::size_t(16));
#endif
#else
            EXPECT_EQ(sizeof(VertexSet), std::size_t(32));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(VertexSet), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(VertexSet), std::size_t(48)); break;
        default: FAIL(); break;
    }
}

TEST(VertexSet, DefaultConstruction)
{
    const auto set = VertexSet{};
    EXPECT_EQ(set.size(), std::size_t(0));
    EXPECT_EQ(set.begin(), set.end());
    EXPECT_EQ(set.find(Length2{}), set.end());
}

TEST(VertexSet, Add)
{
    auto set = VertexSet{};
    ASSERT_EQ(set.size(), std::size_t(0));

    EXPECT_TRUE(set.add(Length2{1_m, 1_m}));
    EXPECT_EQ(set.size(), std::size_t(1));

    EXPECT_FALSE(set.add(Length2{1_m, 1_m}));
    EXPECT_EQ(set.size(), std::size_t(1));
    
    const auto v = Length2{0_m, 0_m};

    EXPECT_TRUE(set.add(v));
    EXPECT_EQ(set.size(), std::size_t(2));
    
    EXPECT_FALSE(set.add(Length2{1_m, 1_m}));
    EXPECT_EQ(set.size(), std::size_t(2));
    
    EXPECT_FALSE(set.add(v));
    EXPECT_EQ(set.size(), std::size_t(2));
    
    const auto v_prime = v + Length2{
        std::numeric_limits<Real>::min() * Meter,
        std::numeric_limits<Real>::min() * Meter
    };
    
    ASSERT_NE(v, v_prime);
    
    EXPECT_FALSE(set.add(v_prime));
    EXPECT_EQ(set.size(), std::size_t(2));
    
    EXPECT_TRUE(set.add(Length2{4_m, 5_m}));
    EXPECT_EQ(set.size(), std::size_t(3));
    
    EXPECT_TRUE(set.add(Length2{6_m, 5_m}));
    EXPECT_EQ(set.size(), std::size_t(4));

    EXPECT_TRUE(set.add(Length2{8_m, 5_m}));
    EXPECT_EQ(set.size(), std::size_t(5));
}
