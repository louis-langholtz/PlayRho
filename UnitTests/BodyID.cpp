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

#include <PlayRho/Dynamics/BodyID.hpp>

using namespace playrho;

TEST(BodyID, ByteSize)
{
    EXPECT_EQ(sizeof(BodyID), 2u);
}

TEST(BodyID, EqualsOperator)
{
    EXPECT_TRUE(BodyID(0u) == BodyID(0u));
    EXPECT_TRUE(BodyID(1u) == BodyID(1u));
    EXPECT_FALSE(BodyID(2u) == BodyID(1u));
    EXPECT_FALSE(BodyID(1u) == BodyID(2u));
}

TEST(BodyID, NotEqualsOperator)
{
    EXPECT_FALSE(BodyID(0u) != BodyID(0u));
    EXPECT_FALSE(BodyID(1u) != BodyID(1u));
    EXPECT_TRUE(BodyID(2u) != BodyID(1u));
    EXPECT_TRUE(BodyID(1u) != BodyID(2u));
}

TEST(BodyID, swap)
{
    auto b0 = BodyID(0u);
    auto b1 = BodyID(1u);
    EXPECT_NO_THROW(swap(b0, b1));
    EXPECT_EQ(b0, BodyID(1u));
    EXPECT_EQ(b1, BodyID(0u));
}

TEST(BodyID, GetInvalid)
{
    EXPECT_EQ(InvalidBodyID, GetInvalid<BodyID>());
}
