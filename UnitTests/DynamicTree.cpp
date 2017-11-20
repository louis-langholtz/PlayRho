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

#include "gtest/gtest.h"
#include <PlayRho/Collision/DynamicTree.hpp>
#include <type_traits>

using namespace playrho;

TEST(DynamicTree, ByteSize)
{
#if defined(_WIN64)
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(32));
#elif defined(_WIN32)
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(24));
#else
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(32));
#endif
}

TEST(DynamicTree, TreeNodeByteSize)
{    
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(48)); break;
        case  8: EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(112)); break;
    }
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
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(0));
    EXPECT_TRUE(foo.Validate());
}

TEST(DynamicTree, ZeroCapacityConstructionThrows)
{
    EXPECT_THROW(DynamicTree{DynamicTree::Size{0}}, InvalidArgument);
}

TEST(DynamicTree, InitializingConstruction)
{
    constexpr auto initCapacity = DynamicTree::GetDefaultInitialNodeCapacity() * 2;
    DynamicTree foo{initCapacity};
    EXPECT_EQ(foo.GetNodeCapacity(), initCapacity);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_TRUE(foo.Validate());
}

TEST(DynamicTree, CopyConstruction)
{
    DynamicTree orig;
    {
        DynamicTree copy{orig};
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(copy.GetMaxBalance(), orig.GetMaxBalance());
    }

    const auto aabb = AABB2D{
        Length2{0_m, 0_m},
        Length2(1_m, 1_m)
    };
    const auto pid = orig.CreateLeaf(aabb, DynamicTree::LeafData{nullptr, nullptr, 0u});
    {
        DynamicTree copy{orig};
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(copy.GetMaxBalance(), orig.GetMaxBalance());
        EXPECT_EQ(copy.GetLeafData(pid), orig.GetLeafData(pid));
    }
}

TEST(DynamicTree, CopyAssignment)
{
    DynamicTree orig;
    {
        DynamicTree copy;
        copy = orig;
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(copy.GetMaxBalance(), orig.GetMaxBalance());
    }
    
    const auto aabb = AABB2D{
        Length2{0_m, 0_m},
        Length2{1_m, 1_m}
    };
    const auto pid = orig.CreateLeaf(aabb, DynamicTree::LeafData{nullptr, nullptr, 0u});
    {
        DynamicTree copy;
        copy = orig;
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(copy.GetMaxBalance(), orig.GetMaxBalance());
        EXPECT_EQ(copy.GetLeafData(pid), orig.GetLeafData(pid));
    }
}

TEST(DynamicTree, CreateAndDestroyProxy)
{
    DynamicTree foo;

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));

    const auto aabb = AABB2D{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto userdata = DynamicTree::LeafData{nullptr, nullptr, 0u};

    const auto pid = foo.CreateLeaf(aabb, userdata);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetAABB(pid), aabb);
    EXPECT_EQ(foo.GetLeafData(pid), userdata);
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(1));

    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(0));

    foo.DestroyLeaf(pid);
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(0));
}

TEST(DynamicTree, FourIdenticalProxies)
{
    DynamicTree foo;
    
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));

    const auto aabb = AABB2D{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto leafData = DynamicTree::LeafData{nullptr, nullptr, 0u};

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(1));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(0));

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(1));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(3));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(1));
    
    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
    }
    
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(5));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(1));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(5));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));
    
    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
    }
    
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(7));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));
    
    foo.RebuildBottomUp();
    
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(3));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(2));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(7));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(3));
}

TEST(DynamicTree, MoveConstruction)
{
    DynamicTree foo;
    
    const auto aabb = AABB2D{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };

    const auto leafData = DynamicTree::LeafData{nullptr, nullptr, 0u};


    const auto leaf0 = foo.CreateLeaf(aabb, leafData);
    const auto leaf1 = foo.CreateLeaf(aabb, leafData);
    const auto leaf2 = foo.CreateLeaf(aabb, leafData);
    const auto leaf3 = foo.CreateLeaf(aabb, leafData);

    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::Size(4));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));

    DynamicTree roo{std::move(foo)};

    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    
    EXPECT_EQ(roo.GetRootIndex(), DynamicTree::Size(4));
    EXPECT_EQ(roo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(roo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(roo.GetLeafCount(), DynamicTree::Size(4));
    
    EXPECT_EQ(roo.GetAABB(leaf0), aabb);
    EXPECT_EQ(roo.GetAABB(leaf1), aabb);
    EXPECT_EQ(roo.GetAABB(leaf2), aabb);
    EXPECT_EQ(roo.GetAABB(leaf3), aabb);
}

TEST(DynamicTree, MoveAssignment)
{
    DynamicTree foo;
    
    const auto aabb = AABB2D{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    
    const auto leafData = DynamicTree::LeafData{nullptr, nullptr, 0u};

    
    const auto leaf0 = foo.CreateLeaf(aabb, leafData);
    const auto leaf1 = foo.CreateLeaf(aabb, leafData);
    const auto leaf2 = foo.CreateLeaf(aabb, leafData);
    const auto leaf3 = foo.CreateLeaf(aabb, leafData);
    
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::Size(4));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));
    
    DynamicTree roo;
    
    roo = std::move(foo);
    
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    
    EXPECT_EQ(roo.GetRootIndex(), DynamicTree::Size(4));
    EXPECT_EQ(roo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(roo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(roo.GetLeafCount(), DynamicTree::Size(4));
    
    EXPECT_EQ(roo.GetAABB(leaf0), aabb);
    EXPECT_EQ(roo.GetAABB(leaf1), aabb);
    EXPECT_EQ(roo.GetAABB(leaf2), aabb);
    EXPECT_EQ(roo.GetAABB(leaf3), aabb);
}

TEST(DynamicTree, CapacityIncreases)
{
    DynamicTree foo{DynamicTree::Size{1}};
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));

    const auto aabb = AABB2D{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto leafData = DynamicTree::LeafData{nullptr, nullptr, 0u};

    foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    
    foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(2));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(4));

    foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(5));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(8));

    foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(8));
}
