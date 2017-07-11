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

#include <PlayRho/Common/GrowableStack.hpp>

using namespace box2d;

TEST(GrowableStack, ByteSize)
{
    using T = GrowableStack<std::int32_t, 64>;
    EXPECT_EQ(sizeof(T), sizeof(T::ElementType) * T::GetInitialCapacity() + std::size_t(24));
}

TEST(GrowableStack, PushAndPop)
{
    using T = GrowableStack<std::int32_t, 4>;
    ASSERT_EQ(T::GetInitialCapacity(), T::CountType(4));

    T foo;
    ASSERT_EQ(foo.GetCount(), T::CountType(0));
    ASSERT_EQ(foo.GetCapacity(), T::GetInitialCapacity());

    foo.Push(104);
    EXPECT_EQ(foo.GetCount(), T::CountType(1));
    EXPECT_EQ(foo.GetCapacity(), T::GetInitialCapacity());
    
    EXPECT_EQ(foo.Pop(), 104);
    EXPECT_EQ(foo.GetCount(), T::CountType(0));
    EXPECT_EQ(foo.GetCapacity(), T::GetInitialCapacity());

    foo.Push(1);
    foo.Push(2);
    foo.Push(3);
    foo.Push(4);
    EXPECT_EQ(foo.GetCount(), T::CountType(4));
    EXPECT_EQ(foo.GetCapacity(), T::GetInitialCapacity());

    foo.Push(5);
    EXPECT_EQ(foo.GetCount(), T::CountType(5));
    EXPECT_EQ(foo.GetCapacity(), T::GetInitialCapacity() * T::GetBufferGrowthRate());
    
    EXPECT_EQ(foo.Pop(), 5);
    EXPECT_EQ(foo.GetCount(), T::CountType(4));
    EXPECT_EQ(foo.GetCapacity(), T::GetInitialCapacity() * T::GetBufferGrowthRate());
}
