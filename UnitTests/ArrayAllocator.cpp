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

#include <PlayRho/Common/ArrayAllocator.hpp>

using namespace playrho;

TEST(ArrayAllocator, DefaultConstructor)
{
    ArrayAllocator<int> object;
    EXPECT_EQ(object.size(), 0u);
    EXPECT_EQ(object.free_size(), 0u);
    EXPECT_EQ(object.GetIndex(nullptr), static_cast<ArrayAllocator<int>::size_type>(-1));
    EXPECT_FALSE(object.FindFree(0u));
    EXPECT_THROW(object.at(0u), std::out_of_range);
}

TEST(ArrayAllocator, AllocateWithNoFreeIncreasesSizeByOne)
{
    ArrayAllocator<int> object;
    auto index = static_cast<ArrayAllocator<int>::size_type>(-1);
    ASSERT_EQ(object.size(), 0u);
    ASSERT_EQ(object.free_size(), 0u);
    EXPECT_NO_THROW(index = object.Allocate(5));
    EXPECT_EQ(object.size(), 1u);
    EXPECT_EQ(object.free_size(), 0u);
    EXPECT_FALSE(object.FindFree(index));
}

TEST(ArrayAllocator, AllocateWithFreeDecreasesFreeSizeByOne)
{
    ArrayAllocator<int> object;
    ArrayAllocator<int>::size_type index;

    ASSERT_EQ(object.size(), 0u);
    ASSERT_NO_THROW(index = object.Allocate(5));
    ASSERT_NO_THROW(object.Free(index));
    ASSERT_EQ(object.size(), 1u);
    ASSERT_EQ(object.free_size(), 1u);
    ASSERT_TRUE(object.FindFree(index));

    EXPECT_NO_THROW(index = object.Allocate(10));
    EXPECT_EQ(object.size(), 1u);
    EXPECT_EQ(object.free_size(), 0u);
    EXPECT_FALSE(object.FindFree(index));
    EXPECT_NO_THROW(object.at(index));
}

TEST(ArrayAllocator, FreeIncreasesFreeSize)
{
    ArrayAllocator<int> object;

    ASSERT_EQ(object.free_size(), 0u);
    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Allocate(2));
    ASSERT_NO_THROW(object.Allocate(3));
    ASSERT_NO_THROW(object.Allocate(4));
    ASSERT_EQ(object.free_size(), 0u);
    ASSERT_EQ(object.size(), 4u);

    EXPECT_NO_THROW(object.Free(3u));
    EXPECT_EQ(object.free_size(), 1u);
    EXPECT_NO_THROW(object.Free(2u));
    EXPECT_EQ(object.free_size(), 2u);
    EXPECT_NO_THROW(object.Free(1u));
    EXPECT_EQ(object.free_size(), 3u);
    EXPECT_NO_THROW(object.Free(0u));
    EXPECT_EQ(object.free_size(), 4u);
}

TEST(ArrayAllocator, FreeOutOfRangeThrows)
{
    ArrayAllocator<int> object;
    ArrayAllocator<int>::size_type index;

    ASSERT_EQ(object.size(), 0u);
    EXPECT_THROW(object.Free(0u), std::out_of_range);

    ASSERT_NO_THROW(index = object.Allocate(5));
    ASSERT_EQ(index, 0u);
    ASSERT_EQ(object.size(), 1u);
    EXPECT_THROW(object.Free(1u), std::out_of_range);
    EXPECT_THROW(object.Free(2u), std::out_of_range);
    EXPECT_THROW(object.Free(static_cast<ArrayAllocator<int>::size_type>(-1)), std::out_of_range);
}

TEST(ArrayAllocator, clear)
{
    ArrayAllocator<int> object;

    ASSERT_EQ(object.size(), 0u);
    ASSERT_EQ(object.free_size(), 0u);
    EXPECT_NO_THROW(object.clear());
    EXPECT_EQ(object.size(), 0u);
    EXPECT_EQ(object.free_size(), 0u);

    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_EQ(object.size(), 3u);
    ASSERT_EQ(object.free_size(), 0u);
    EXPECT_NO_THROW(object.clear());
    EXPECT_EQ(object.size(), 0u);
    EXPECT_EQ(object.free_size(), 0u);

    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Allocate(1));
    ASSERT_NO_THROW(object.Free(2u));
    ASSERT_EQ(object.size(), 3u);
    ASSERT_EQ(object.free_size(), 1u);
    EXPECT_NO_THROW(object.clear());
    EXPECT_EQ(object.size(), 0u);
    EXPECT_EQ(object.free_size(), 0u);
}

TEST(ArrayAllocator, usedFreeFunction)
{
    ArrayAllocator<int> object;
    EXPECT_EQ(used(object), 0u);
    ASSERT_NO_THROW(object.Allocate(1));
    EXPECT_EQ(used(object), 1u);
    ASSERT_NO_THROW(object.Allocate(2));
    EXPECT_EQ(used(object), 2u);
    ASSERT_NO_THROW(object.Allocate(3));
    EXPECT_EQ(used(object), 3u);
    ASSERT_NO_THROW(object.Free(2u));
    EXPECT_EQ(used(object), 2u);
    ASSERT_NO_THROW(object.Allocate(3));
    ASSERT_NO_THROW(object.Free(1u));
    EXPECT_EQ(used(object), 2u);
    ASSERT_NO_THROW(object.Free(0u));
    EXPECT_EQ(used(object), 1u);
}
