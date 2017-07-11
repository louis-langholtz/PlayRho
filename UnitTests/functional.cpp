/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "gtest/gtest.h"
#include <functional>

TEST(functional, FunctionSize)
{
#ifdef __APPLE__
    EXPECT_EQ(sizeof(std::function<int()>), std::size_t(48));
    EXPECT_EQ(sizeof(std::function<void(int)>), std::size_t(48));
    EXPECT_EQ(sizeof(std::function<float(int)>), std::size_t(48));
    EXPECT_EQ(sizeof(std::function<int(float)>), std::size_t(48));
    EXPECT_EQ(sizeof(std::function<int(double)>), std::size_t(48));
    EXPECT_EQ(sizeof(std::function<int(double, long, void*)>), std::size_t(48));
#endif
#ifdef __linux__
    EXPECT_EQ(sizeof(std::function<int()>), std::size_t(32));
    EXPECT_EQ(sizeof(std::function<void(int)>), std::size_t(32));
    EXPECT_EQ(sizeof(std::function<float(int)>), std::size_t(32));
    EXPECT_EQ(sizeof(std::function<int(float)>), std::size_t(32));
    EXPECT_EQ(sizeof(std::function<int(double)>), std::size_t(32));
    EXPECT_EQ(sizeof(std::function<int(double, long, void*)>), std::size_t(32));
#endif
}

TEST(functional, FunctionSizeGreaterThanFunctionPtr)
{
    using FunctionPtr = int (*)(int);
    using StdFunction = std::function<int(int)>;
    EXPECT_GT(sizeof(StdFunction), sizeof(FunctionPtr));
}
