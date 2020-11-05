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

#include <PlayRho/Common/GrowableStack.hpp>

using namespace playrho;

TEST(GrowableStack, ByteSize)
{
    using T = GrowableStack<std::int32_t, 64>;
#if defined(_WIN32) && !defined(_WIN64)
    EXPECT_EQ(sizeof(T), sizeof(T::ElementType) * T::GetInitialCapacity() + std::size_t(12));
#else
    EXPECT_EQ(sizeof(T), sizeof(T::ElementType) * T::GetInitialCapacity() + std::size_t(24));
#endif
}

TEST(GrowableStack, PushAndPop)
{
    using T = GrowableStack<std::int32_t, 4>;
    ASSERT_EQ(T::GetInitialCapacity(), T::CountType(4));

    T foo;
    ASSERT_EQ(foo.size(), T::CountType(0));
    ASSERT_EQ(foo.capacity(), T::GetInitialCapacity());

    foo.push(104);
    EXPECT_EQ(foo.size(), T::CountType(1));
    EXPECT_EQ(foo.capacity(), T::GetInitialCapacity());
    
    EXPECT_EQ(foo.top(), 104);
    foo.pop();
    EXPECT_EQ(foo.size(), T::CountType(0));
    EXPECT_EQ(foo.capacity(), T::GetInitialCapacity());

    foo.push(1);
    foo.push(2);
    foo.push(3);
    foo.push(4);
    EXPECT_EQ(foo.size(), T::CountType(4));
    EXPECT_EQ(foo.capacity(), T::GetInitialCapacity());

    foo.push(5);
    EXPECT_EQ(foo.size(), T::CountType(5));
    EXPECT_EQ(foo.capacity(), T::GetInitialCapacity() * T::GetBufferGrowthRate());
    
    EXPECT_EQ(foo.top(), 5);
    foo.pop();
    EXPECT_EQ(foo.size(), T::CountType(4));
    EXPECT_EQ(foo.capacity(), T::GetInitialCapacity() * T::GetBufferGrowthRate());

    foo.push(5);
    foo.push(6);
    foo.push(7);
    foo.push(8);
    EXPECT_EQ(foo.size(), T::CountType(8));
    EXPECT_EQ(foo.capacity(), T::CountType(8));

    foo.push(9);
    EXPECT_EQ(foo.size(), T::CountType(9));
    EXPECT_EQ(foo.capacity(), T::CountType(16));
}

