/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Common/Settings.hpp>

using namespace box2d;

TEST(Settings, GetVersion)
{
    const auto version = Version{3, 0, 0};
    EXPECT_EQ(GetVersion().major, version.major);
    EXPECT_EQ(GetVersion().minor, version.minor);
    EXPECT_EQ(GetVersion().revision, version.revision);
    EXPECT_EQ(GetVersion(), version);
}

TEST(Settings, GetBuildDetails)
{
    EXPECT_FALSE(GetBuildDetails().empty());
    const auto result = GetBuildDetails();
    EXPECT_NE(result.find_first_of("asserts="), result.npos);
    EXPECT_NE(result.find_first_of("RealNum="), result.npos);
}
