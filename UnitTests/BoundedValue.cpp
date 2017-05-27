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
#include <Box2D/Common/BoundedValue.hpp>
#include <limits>
#include <cmath>

using namespace box2d;

TEST(BoundedValue, NonNegativeFloat)
{
    EXPECT_EQ(float{NonNegative<float>(1.0f)}, 1.0f);
    EXPECT_EQ(float(NonNegative<float>(1.0f)), float(NonNegative<float>(1.0f)));
    EXPECT_EQ(float(NonNegative<float>(0.0f)), 0.0f);
    EXPECT_EQ(float(NonNegative<float>(std::numeric_limits<float>::infinity())), std::numeric_limits<float>::infinity());

    EXPECT_THROW(NonNegative<float>{-0.00001f}, NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{-1.4f}, NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{-std::numeric_limits<float>::infinity()}, NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{std::numeric_limits<float>::quiet_NaN()}, NonNegative<float>::exception_type);
}

TEST(BoundedValue, NonNegativeDouble)
{
    EXPECT_EQ(double{NonNegative<double>(1.0f)}, 1.0f);
    EXPECT_EQ(double(NonNegative<double>(1.0f)), double(NonNegative<double>(1.0f)));
    EXPECT_EQ(double(NonNegative<double>(0.0f)), 0.0f);
    EXPECT_EQ(double(NonNegative<double>(std::numeric_limits<double>::infinity())), std::numeric_limits<double>::infinity());
    
    EXPECT_THROW(NonNegative<double>{-0.00001f}, NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{-1.4f}, NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{-std::numeric_limits<double>::infinity()}, NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{std::numeric_limits<double>::quiet_NaN()}, NonNegative<double>::exception_type);
}

TEST(BoundedValue, NonNegativeInt)
{
    EXPECT_EQ(int{NonNegative<int>(1)}, 1);
    EXPECT_EQ(int(NonNegative<int>(1)), int(NonNegative<int>(1)));
    EXPECT_EQ(int(NonNegative<int>(0)), 0);
    
    EXPECT_THROW(NonNegative<int>{-1}, NonNegative<int>::exception_type);
    EXPECT_THROW(NonNegative<int>{-2}, NonNegative<int>::exception_type);
}

TEST(BoundedValue, NonPositiveFloat)
{
    EXPECT_EQ(float(NonPositive<float>(-1.0f)), -1.0f);
    EXPECT_EQ(float(NonPositive<float>(-1.0f)), float(NonPositive<float>(-1.0f)));
    EXPECT_EQ(float(NonPositive<float>(0.0f)), 0.0f);
    EXPECT_EQ(float(NonPositive<float>(-std::numeric_limits<float>::infinity())), -std::numeric_limits<float>::infinity());
    
    EXPECT_THROW(NonPositive<float>{0.00001f}, NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{1.4f}, NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{std::numeric_limits<float>::infinity()}, NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{std::numeric_limits<float>::quiet_NaN()}, NonPositive<float>::exception_type);
}

TEST(BoundedValue, NonPositiveDouble)
{
    EXPECT_EQ(double(NonPositive<double>(-1.0f)), -1.0f);
    EXPECT_EQ(double(NonPositive<double>(-1.0f)), double(NonPositive<double>(-1.0f)));
    EXPECT_EQ(double(NonPositive<double>(0.0f)), 0.0f);
    EXPECT_EQ(double(NonPositive<double>(-std::numeric_limits<double>::infinity())), -std::numeric_limits<double>::infinity());
    
    EXPECT_THROW(NonPositive<double>{0.00001f}, NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{1.4f}, NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{std::numeric_limits<double>::infinity()}, NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{std::numeric_limits<double>::quiet_NaN()}, NonPositive<double>::exception_type);
}

TEST(BoundedValue, NonPositiveInt)
{
    EXPECT_EQ(int(NonPositive<int>(-1)), -1);
    EXPECT_EQ(int(NonPositive<int>(-1)), int(NonPositive<int>(-1)));
    EXPECT_EQ(int(NonPositive<int>(0)), 0);
    
    EXPECT_THROW(NonPositive<int>{1}, NonPositive<int>::exception_type);
    EXPECT_THROW(NonPositive<int>{2}, NonPositive<int>::exception_type);
}

TEST(BoundedValue, FiniteDouble)
{
    EXPECT_EQ(double(Finite<double>(0)), 0.0);
    EXPECT_EQ(double(Finite<double>(-1.0)), -1.0);
    EXPECT_EQ(double(Finite<double>(+1.0)), +1.0);

    EXPECT_THROW(Finite<double>{std::numeric_limits<double>::infinity()}, Finite<double>::exception_type);
    EXPECT_THROW(Finite<double>{std::numeric_limits<double>::quiet_NaN()}, Finite<double>::exception_type);
}
