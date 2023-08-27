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

#include <PlayRho/TypeInfo.hpp> // for GetTypeName

#include <type_traits>
#include <cmath>

TEST(double, GetInvalid)
{
    auto val = playrho::GetInvalid<double>();
    EXPECT_TRUE(std::isnan(val));
    const auto same = std::is_same<decltype(val), double>::value;
    EXPECT_TRUE(same);
}

TEST(double, GetTypeName)
{
    const auto name = playrho::GetTypeName<double>();
    EXPECT_STREQ(name, "double");
}

TEST(double, traits)
{
    EXPECT_TRUE((playrho::IsAddable<double>::value));
    EXPECT_TRUE((playrho::IsAddable<double,double>::value));
    EXPECT_TRUE((playrho::IsAddable<double,float>::value));
    EXPECT_TRUE((playrho::IsAddable<double,int>::value));
}
