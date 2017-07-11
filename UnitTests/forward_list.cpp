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
#include <forward_list>

TEST(forward_list, EmptyListIsEmpty)
{
    std::forward_list<int> nums;
    EXPECT_TRUE(nums.empty());
}

TEST(forward_list, EmptyListBeginEqualToEnd)
{
    std::forward_list<int> nums;
    EXPECT_EQ(nums.begin(), nums.end());
}

TEST(forward_list, EmptyListDistanceIsZero)
{
    std::forward_list<int> nums;
    EXPECT_EQ(std::distance(nums.begin(), nums.end()), 0);
}

TEST(forward_list, EmptyListBeforeBeginNotEqualToEnd)
{
    std::forward_list<int> nums;
    EXPECT_NE(nums.before_begin(), nums.end());
}

TEST(forward_list, EmptyListNextOfBeforeBeginEqualToEnd)
{
    std::forward_list<int> nums;
    EXPECT_EQ(std::next(nums.before_begin()), nums.end());
}

TEST(forward_list, SingleElementList)
{
    std::forward_list<int> nums = { 1 };
    EXPECT_FALSE(nums.empty());
    EXPECT_NE(nums.begin(), nums.end());
    EXPECT_EQ(std::distance(nums.begin(), nums.end()), 1);
}

TEST(forward_list, PopFromFrontUntilEmptyWhileLoop)
{
    std::forward_list<int> nums = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    
    EXPECT_EQ(std::distance(nums.begin(), nums.end()), 9);
    EXPECT_FALSE(nums.empty());
    
    while (!nums.empty())
    {
        nums.pop_front();
    }

    EXPECT_TRUE(nums.empty());
    
    auto size = 0;
    for (auto iter = nums.begin(); iter != nums.end(); ++iter)
    {
        ++size;
    }
    EXPECT_EQ(size, 0);
}

TEST(forward_list, EraseFromBeginUntilEmptyForLoop)
{
    std::forward_list<int> nums = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    EXPECT_EQ(std::distance(nums.begin(), nums.end()), 9);
    EXPECT_FALSE(nums.empty());
    for (auto prev = nums.before_begin(); !nums.empty(); prev = nums.before_begin())
    {
        nums.erase_after(prev);
    }
    EXPECT_TRUE(nums.empty());
    
    auto size = 0;
    for (auto iter = nums.begin(); iter != nums.end(); ++iter)
    {
        ++size;
    }
    EXPECT_EQ(size, 0);
}

TEST(forward_list, EraseFromBeginUntilEmptyWhileLoop)
{
    std::forward_list<int> nums = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    
    EXPECT_EQ(std::distance(nums.begin(), nums.end()), 9);
    EXPECT_FALSE(nums.empty());
    while (!nums.empty())
    {
        const auto prev = nums.before_begin();
        nums.erase_after(prev);
    }
    EXPECT_TRUE(nums.empty());
    
    auto size = 0;
    for (auto iter = nums.begin(); iter != nums.end(); ++iter)
    {
        ++size;
    }
    EXPECT_EQ(size, 0);
}

