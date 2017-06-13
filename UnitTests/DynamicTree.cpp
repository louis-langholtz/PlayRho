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
#include <Box2D/Collision/DynamicTree.hpp>
#include <type_traits>

using namespace box2d;

TEST(DynamicTree, ByteSizeIs24)
{
    EXPECT_EQ(sizeof(DynamicTree), size_t(24));
}

TEST(DynamicTree, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<DynamicTree>::value);
    
    EXPECT_TRUE(std::is_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_nothrow_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_trivially_constructible<DynamicTree>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_nothrow_copy_constructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<DynamicTree>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<DynamicTree>::value);
    EXPECT_FALSE(std::is_nothrow_copy_assignable<DynamicTree>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<DynamicTree>::value);
    
    EXPECT_TRUE(std::is_destructible<DynamicTree>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree>::value);
    EXPECT_FALSE(std::is_trivially_destructible<DynamicTree>::value);
}

TEST(DynamicTree, DefaultConstruction)
{
    DynamicTree foo;
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(0));    
    EXPECT_TRUE(foo.Validate());
}

TEST(DynamicTree, InitializingConstruction)
{
    constexpr auto initCapacity = DynamicTree::GetDefaultInitialNodeCapacity() * 2;
    DynamicTree foo{initCapacity};
    EXPECT_EQ(foo.GetNodeCapacity(), initCapacity);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(0));
    EXPECT_TRUE(foo.Validate());
}

TEST(DynamicTree, CreateAndDestroyProxy)
{
    DynamicTree foo;

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::size_type(0));

    const auto aabb = AABB{Vec2{3, 1} * Meter, Vec2{-5, -2} * Meter};
    const auto userdata = nullptr;

    const auto pid = foo.CreateProxy(aabb, userdata);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetFatAABB(pid), aabb);
    EXPECT_EQ(foo.GetUserData(pid), userdata);
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(1));

    EXPECT_EQ(foo.ComputeHeight(), DynamicTree::size_type(0));

    foo.DestroyProxy(pid);
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(0));
}

TEST(DynamicTree, FourIdenticalProxies)
{
    DynamicTree foo;
    
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::size_type(0));

    const auto aabb = AABB{Vec2{3, 1} * Meter, Vec2{-5, -2} * Meter};
    const auto userdata = nullptr;
    
    {
        const auto pid = foo.CreateProxy(aabb, userdata);
        EXPECT_EQ(foo.GetFatAABB(pid), aabb);
        EXPECT_EQ(foo.GetUserData(pid), userdata);
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(1));
    EXPECT_EQ(foo.ComputeHeight(), DynamicTree::size_type(0));

    {
        const auto pid = foo.CreateProxy(aabb, userdata);
        EXPECT_EQ(foo.GetFatAABB(pid), aabb);
        EXPECT_EQ(foo.GetUserData(pid), userdata);
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(3));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(1));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(3));
    EXPECT_EQ(foo.ComputeHeight(), DynamicTree::size_type(1));
    
    {
        const auto pid = foo.CreateProxy(aabb, userdata);
        EXPECT_EQ(foo.GetFatAABB(pid), aabb);
        EXPECT_EQ(foo.GetUserData(pid), userdata);
    }
    
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(5));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(1));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(5));
    EXPECT_EQ(foo.ComputeHeight(), DynamicTree::size_type(2));
    
    {
        const auto pid = foo.CreateProxy(aabb, userdata);
        EXPECT_EQ(foo.GetFatAABB(pid), aabb);
        EXPECT_EQ(foo.GetUserData(pid), userdata);
    }
    
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::size_type(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetHeight(), DynamicTree::size_type(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::size_type(0));
    EXPECT_EQ(foo.GetAreaRatio(), RealNum(7));
    EXPECT_EQ(foo.ComputeHeight(), DynamicTree::size_type(2));
}
