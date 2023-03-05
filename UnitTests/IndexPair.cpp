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
#include <PlayRho/Collision/IndexPair.hpp>

using namespace playrho;

TEST(IndexPair, Init)
{
    IndexPair ip{1, 2};
    EXPECT_EQ(std::get<0>(ip), 1);
    EXPECT_EQ(std::get<1>(ip), 2);
}

TEST(IndexPair, Equality)
{
    IndexPair ip1{2, 3};
    IndexPair ip2{2, 3};
    EXPECT_EQ(ip1, ip1);
    EXPECT_EQ(ip1, ip2);
    EXPECT_EQ(ip2, ip1);
    EXPECT_EQ(ip2, ip2);
}

TEST(IndexPair, Inequality)
{
    IndexPair ip1{2, 3};
    IndexPair ip2{1, 0};
    EXPECT_NE(ip1, ip2);
    EXPECT_NE(ip2, ip1);
}

TEST(IndexPair, InvalidIndex)
{
    const auto invalid_index = InvalidVertex;
    IndexPair ip{invalid_index, 2};    
    EXPECT_EQ(invalid_index, std::get<0>(ip));
    EXPECT_NE(invalid_index, std::get<1>(ip));
}
