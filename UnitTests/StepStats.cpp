/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Dynamics/StepStats.hpp>

using namespace playrho;

TEST(StepStats, PreStatsByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(PreStepStats), std::size_t(24)); break;
        case  8: EXPECT_EQ(sizeof(PreStepStats), std::size_t(24)); break;
        case 16: EXPECT_EQ(sizeof(PreStepStats), std::size_t(24)); break;
        default: FAIL(); break;
    }
}

TEST(StepStats, RegStatsByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(RegStepStats), std::size_t(32)); break;
        case  8: EXPECT_EQ(sizeof(RegStepStats), std::size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(RegStepStats), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(StepStats, ToiStatsByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(ToiStepStats), std::size_t(60)); break;
        case  8: EXPECT_EQ(sizeof(ToiStepStats), std::size_t(72)); break;
        case 16: EXPECT_EQ(sizeof(ToiStepStats), std::size_t(96)); break;
        default: FAIL(); break;
    }
}

TEST(StepStats, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(StepStats), std::size_t(116)); break;
        case  8: EXPECT_EQ(sizeof(StepStats), std::size_t(136)); break;
        case 16: EXPECT_EQ(sizeof(StepStats), std::size_t(192)); break;
        default: FAIL(); break;
    }
}

TEST(StepStats, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<StepStats>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_default_constructible<StepStats>::value);
#else
    EXPECT_TRUE(std::is_nothrow_default_constructible<StepStats>::value);
#endif // USE_BOOST_UNITS
    EXPECT_FALSE(std::is_trivially_default_constructible<StepStats>::value);
    
    EXPECT_TRUE(std::is_constructible<StepStats>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_constructible<StepStats>::value);
#else
    EXPECT_TRUE(std::is_nothrow_constructible<StepStats>::value);
#endif // USE_BOOST_UNITS
    EXPECT_FALSE(std::is_trivially_constructible<StepStats>::value);

    EXPECT_TRUE(std::is_copy_constructible<StepStats>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_constructible<StepStats>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<StepStats>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_constructible<StepStats>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<StepStats>::value);
#endif
    
    EXPECT_TRUE(std::is_copy_assignable<StepStats>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_assignable<StepStats>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<StepStats>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_assignable<StepStats>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<StepStats>::value);
#endif
    
    EXPECT_TRUE(std::is_destructible<StepStats>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<StepStats>::value);
    EXPECT_TRUE(std::is_trivially_destructible<StepStats>::value);
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
