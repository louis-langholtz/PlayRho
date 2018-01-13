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
#include <PlayRho/Common/OptionalValue.hpp>

using namespace playrho;

TEST(OptionalValue, DefaultConstruction)
{
    OptionalValue<int> foo;
    EXPECT_FALSE(foo.has_value());
    EXPECT_FALSE(static_cast<bool>(foo));
    EXPECT_FALSE(bool(foo));
}

TEST(OptionalValue, ZeroInitialization)
{
    OptionalValue<int> foo{};
    EXPECT_FALSE(foo.has_value());
    EXPECT_FALSE(static_cast<bool>(foo));
    EXPECT_FALSE(bool(foo));
}

TEST(OptionalValue, Initialization)
{
    OptionalValue<int> foo{5};
    EXPECT_TRUE(foo.has_value());
    EXPECT_TRUE(static_cast<bool>(foo));
    EXPECT_TRUE(bool(foo));
    EXPECT_EQ(*foo, 5);
}

TEST(OptionalValue, AssignmentOperator)
{
    OptionalValue<int> foo{};
    ASSERT_FALSE(foo.has_value());
    
    foo = 2;
    EXPECT_TRUE(foo.has_value());
    EXPECT_TRUE(bool(foo));
    EXPECT_EQ(*foo, 2);
    EXPECT_EQ(foo.value(), 2);
    
    foo = 0;
    EXPECT_TRUE(foo.has_value());
    EXPECT_TRUE(bool(foo));
    EXPECT_EQ(*foo, 0);
    EXPECT_EQ(foo.value(), 0);
    
    OptionalValue<int> boo;
    ASSERT_FALSE(boo.has_value());

    foo = boo;
    EXPECT_FALSE(foo.has_value());
}

TEST(OptionalValue, value_or)
{
    OptionalValue<int> foo{};
    ASSERT_FALSE(foo.has_value());

    EXPECT_EQ(foo.value_or(66), 66);
}
