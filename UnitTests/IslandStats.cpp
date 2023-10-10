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

#include <playrho/IslandStats.hpp>

using namespace playrho;

TEST(IslandStats, DefaultConstructor)
{
    IslandStats object{};
    EXPECT_EQ(object.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(object.maxIncImpulse, 0_Ns);
    EXPECT_EQ(object.bodiesSlept, 0);
    EXPECT_EQ(object.contactsUpdated, 0);
    EXPECT_EQ(object.contactsSkipped, 0);
    EXPECT_EQ(object.solved, false);
    EXPECT_EQ(object.positionIters, 0);
    EXPECT_EQ(object.velocityIters, 0);
}

TEST(IslandStats, IncContactsUpdated)
{
    constexpr auto expected = 42u;
    EXPECT_EQ(IslandStats{}.IncContactsUpdated(40u).IncContactsUpdated(2u).contactsUpdated, expected);
}

TEST(IslandStats, IncContactsSkipped)
{
    constexpr auto expected = 42u;
    EXPECT_EQ(IslandStats{}.IncContactsSkipped(40u).IncContactsSkipped(2u).contactsSkipped, expected);
}
