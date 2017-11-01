/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Common/VertexSet.hpp>

using namespace playrho;

TEST(VertexSet, ByteSizeIs_32_or_48)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(VertexSet), std::size_t(32)); break;
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

    EXPECT_TRUE(set.add(Length2{Real(1) * Meter, Real(1) * Meter}));
    EXPECT_EQ(set.size(), std::size_t(1));

    EXPECT_FALSE(set.add(Length2{Real(1) * Meter, Real(1) * Meter}));
    EXPECT_EQ(set.size(), std::size_t(1));
    
    const auto v = Length2{Real(0) * Meter, Real(0) * Meter};

    EXPECT_TRUE(set.add(v));
    EXPECT_EQ(set.size(), std::size_t(2));
    
    EXPECT_FALSE(set.add(Length2{Real(1) * Meter, Real(1) * Meter}));
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
    
    EXPECT_TRUE(set.add(Length2{Real(4) * Meter, Real(5) * Meter}));
    EXPECT_EQ(set.size(), std::size_t(3));
    
    EXPECT_TRUE(set.add(Length2{Real(6) * Meter, Real(5) * Meter}));
    EXPECT_EQ(set.size(), std::size_t(4));

    EXPECT_TRUE(set.add(Length2{Real(8) * Meter, Real(5) * Meter}));
    EXPECT_EQ(set.size(), std::size_t(5));
}
