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

#include <PlayRho/Common/Version.hpp>

#include <PlayRho/Defines.hpp> // for PLAYRHO_VERSION_*

using namespace playrho;

TEST(Version, GetVersion)
{
    EXPECT_EQ(GetVersion().major, PLAYRHO_VERSION_MAJOR);
    EXPECT_EQ(GetVersion().minor, PLAYRHO_VERSION_MINOR);
    EXPECT_EQ(GetVersion().revision, PLAYRHO_VERSION_PATCH);
}

TEST(Version, GetBuildDetails)
{
    EXPECT_FALSE(empty(GetBuildDetails()));
    const auto result = GetBuildDetails();
    EXPECT_NE(result.find_first_of("asserts="), result.npos);
    EXPECT_NE(result.find_first_of("Real="), result.npos);
}

TEST(Version, EqualsOperator)
{
    // Uses double parenthesis to avoid macro expansion issue with aggragate initializations...
    const Version version{1, 2, 3};
    EXPECT_TRUE(version == version);
    EXPECT_TRUE((Version{1, 2, 3} == version));
    EXPECT_TRUE((version == Version{1, 2, 3}));
    EXPECT_TRUE((Version{2, 1, 3} == Version{2, 1, 3}));
    EXPECT_FALSE((Version{2, 1, 3} == Version{3, 2, 1}));
    EXPECT_FALSE((Version{2, 1, 3} == Version{2, 3, 1}));
    EXPECT_FALSE((Version{2, 1, 3} == Version{3, 1, 2}));
}

TEST(Version, NotEqualsOperator)
{
    // Uses double parenthesis to avoid macro expansion issue with aggragate initializations...
    const Version version{1, 2, 3};
    EXPECT_FALSE(version != version);
    EXPECT_FALSE((Version{1, 2, 3} != version));
    EXPECT_FALSE((version != Version{1, 2, 3}));
    EXPECT_FALSE((Version{2, 1, 3} != Version{2, 1, 3}));
    EXPECT_TRUE((Version{2, 1, 3} != Version{3, 2, 1}));
    EXPECT_TRUE((Version{2, 1, 3} != Version{2, 3, 1}));
    EXPECT_TRUE((Version{2, 1, 3} != Version{3, 1, 2}));
}
