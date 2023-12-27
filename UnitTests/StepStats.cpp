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

#include <playrho/StepStats.hpp>

#include "gtest/gtest.h"

using namespace playrho;

TEST(StepStats, Traits)
{
    EXPECT_TRUE(std::is_default_constructible_v<StepStats>);
#ifdef PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_default_constructible_v<StepStats>);
#else
    EXPECT_TRUE(std::is_nothrow_default_constructible_v<StepStats>);
#endif // PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_trivially_default_constructible_v<StepStats>);
    
    EXPECT_TRUE(std::is_constructible_v<StepStats>);
#ifdef PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_constructible_v<StepStats>);
#else
    EXPECT_TRUE(std::is_nothrow_constructible_v<StepStats>);
#endif // PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_trivially_constructible_v<StepStats>);

    EXPECT_TRUE(std::is_copy_constructible_v<StepStats>);
#ifdef PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_constructible_v<StepStats>);
    EXPECT_FALSE(std::is_trivially_copy_constructible_v<StepStats>);
#else
    EXPECT_TRUE(std::is_nothrow_copy_constructible_v<StepStats>);
    EXPECT_TRUE(std::is_trivially_copy_constructible_v<StepStats>);
#endif
    
    EXPECT_TRUE(std::is_copy_assignable_v<StepStats>);
#ifdef PLAYRHO_USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_assignable_v<StepStats>);
    EXPECT_FALSE(std::is_trivially_copy_assignable_v<StepStats>);
#else
    EXPECT_TRUE(std::is_nothrow_copy_assignable_v<StepStats>);
    EXPECT_TRUE(std::is_trivially_copy_assignable_v<StepStats>);
#endif
    
    EXPECT_TRUE(std::is_destructible_v<StepStats>);
    EXPECT_TRUE(std::is_nothrow_destructible_v<StepStats>);
    EXPECT_TRUE(std::is_trivially_destructible_v<StepStats>);
}

TEST(RegStepStats, DefaultConstructor)
{
    RegStepStats object;
    EXPECT_EQ(object.minSeparation, std::numeric_limits<Length>::infinity());
    EXPECT_EQ(object.islandsFound, static_cast<decltype(object.islandsFound)>(0));
    EXPECT_EQ(object.islandsSolved, static_cast<decltype(object.islandsSolved)>(0));
    EXPECT_EQ(object.contactsAdded, static_cast<decltype(object.contactsAdded)>(0));
    EXPECT_EQ(object.bodiesSlept, static_cast<decltype(object.bodiesSlept)>(0));
    EXPECT_EQ(object.proxiesMoved, static_cast<decltype(object.proxiesMoved)>(0));
    EXPECT_EQ(object.sumPosIters, static_cast<decltype(object.sumPosIters)>(0));
    EXPECT_EQ(object.sumVelIters, static_cast<decltype(object.sumVelIters)>(0));
}

TEST(PreStepStats, Equality)
{
    EXPECT_TRUE(PreStepStats() == PreStepStats());
    {
        auto stats = PreStepStats{};
        ++stats.proxiesCreated;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.proxiesMoved;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsDestroyed;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsAdded;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsIgnored;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsUpdated;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsSkipped;
        EXPECT_FALSE(PreStepStats() == stats);
        EXPECT_FALSE(stats == PreStepStats());
    }
}

TEST(PreStepStats, Inequality)
{
    EXPECT_FALSE(PreStepStats() != PreStepStats());
    {
        auto stats = PreStepStats{};
        ++stats.proxiesCreated;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.proxiesMoved;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsDestroyed;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsAdded;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsIgnored;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsUpdated;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
    {
        auto stats = PreStepStats{};
        ++stats.contactsSkipped;
        EXPECT_TRUE(PreStepStats() != stats);
        EXPECT_TRUE(stats != PreStepStats());
    }
}

TEST(RegStepStats, Equality)
{
    EXPECT_TRUE(RegStepStats() == RegStepStats());
    {
        auto stats = RegStepStats{};
        stats.minSeparation = 1_m;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        stats.maxIncImpulse = 1_Ns;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.islandsFound;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.islandsSolved;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.bodiesSlept;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.maxIslandBodies;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.contactsAdded;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.proxiesMoved;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.sumPosIters;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.sumVelIters;
        EXPECT_FALSE(RegStepStats() == stats);
        EXPECT_FALSE(stats == RegStepStats());
    }
}

TEST(RegStepStats, Inequality)
{
    EXPECT_FALSE(RegStepStats() != RegStepStats());
    {
        auto stats = RegStepStats{};
        stats.minSeparation = 1_m;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        stats.maxIncImpulse = 1_Ns;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.islandsFound;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.islandsSolved;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.bodiesSlept;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.maxIslandBodies;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.contactsAdded;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.proxiesMoved;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.sumPosIters;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
    {
        auto stats = RegStepStats{};
        ++stats.sumVelIters;
        EXPECT_TRUE(RegStepStats() != stats);
        EXPECT_TRUE(stats != RegStepStats());
    }
}

TEST(ToiStepStats, Equality)
{
    EXPECT_TRUE(ToiStepStats() == ToiStepStats());
    {
        auto stats = ToiStepStats{};
        stats.minSeparation = 1_m;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        stats.maxIncImpulse = 1_Ns;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.islandsFound;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.islandsSolved;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsFound;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsAtMaxSubSteps;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsUpdatedToi;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsUpdatedTouching;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsSkippedTouching;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsAdded;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.proxiesMoved;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.sumPosIters;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.sumVelIters;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxDistIters;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxToiIters;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxRootIters;
        EXPECT_FALSE(ToiStepStats() == stats);
        EXPECT_FALSE(stats == ToiStepStats());
    }
}

TEST(ToiStepStats, Inequality)
{
    EXPECT_FALSE(ToiStepStats() != ToiStepStats());
    {
        auto stats = ToiStepStats{};
        stats.minSeparation = 1_m;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        stats.maxIncImpulse = 1_Ns;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.islandsFound;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.islandsSolved;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsFound;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsAtMaxSubSteps;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsUpdatedToi;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsUpdatedTouching;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsSkippedTouching;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.contactsAdded;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.proxiesMoved;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.sumPosIters;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.sumVelIters;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxDistIters;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxToiIters;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
    {
        auto stats = ToiStepStats{};
        ++stats.maxRootIters;
        EXPECT_TRUE(ToiStepStats() != stats);
        EXPECT_TRUE(stats != ToiStepStats());
    }
}
