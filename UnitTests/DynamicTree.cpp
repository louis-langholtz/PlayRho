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
#include <PlayRho/Collision/DynamicTree.hpp>
#include <type_traits>
#include <algorithm>
#include <iterator>

using namespace playrho;
using namespace playrho::d2;

TEST(DynamicTree, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
#if defined(_WIN64)
    EXPECT_EQ(alignof(DynamicTree), 8u);
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(32));
#elif defined(_WIN32)
    EXPECT_EQ(alignof(DynamicTree), 4u);
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(24));
#else
    EXPECT_EQ(alignof(DynamicTree), 8u);
    EXPECT_EQ(sizeof(DynamicTree), std::size_t(32));
#endif
}

TEST(DynamicTree, VariantDataByteSize)
{
#if defined(_WIN64)
    EXPECT_EQ(alignof(DynamicTree), 8u);
    EXPECT_EQ(sizeof(DynamicTree::VariantData), 8u);
#elif defined(_WIN32)
    EXPECT_EQ(alignof(DynamicTree), 4u);
    EXPECT_EQ(sizeof(DynamicTree::VariantData), 8u);
#else
    EXPECT_EQ(alignof(DynamicTree), 8u);
    EXPECT_EQ(sizeof(DynamicTree::VariantData), 8u);
#endif
}

TEST(DynamicTree, TreeNodeByteSize)
{
    switch (sizeof(Real))
    {
        case  4:
            EXPECT_EQ(alignof(DynamicTree::TreeNode), 4u);
            EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(32));
            break;
        case  8:
            EXPECT_EQ(alignof(DynamicTree::TreeNode), 8u);
            EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(48));
            break;
        case 16:
            EXPECT_EQ(alignof(DynamicTree::TreeNode), 16u);
            EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(80));
            break;
    }
}

TEST(DynamicTreeNode, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree::TreeNode>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<DynamicTree::TreeNode>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<DynamicTree::TreeNode>::value);

    EXPECT_TRUE(std::is_constructible<DynamicTree::TreeNode>::value);
    EXPECT_TRUE((std::is_nothrow_constructible<DynamicTree::TreeNode>::value));
    EXPECT_FALSE(std::is_trivially_constructible<DynamicTree::TreeNode>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<DynamicTree::TreeNode>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_constructible<DynamicTree::TreeNode>::value);
    EXPECT_FALSE(std::is_trivially_copy_constructible<DynamicTree::TreeNode>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_constructible<DynamicTree::TreeNode>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<DynamicTree::TreeNode>::value);
#endif
    
    EXPECT_TRUE(std::is_copy_assignable<DynamicTree::TreeNode>::value);
#ifdef USE_BOOST_UNITS
    EXPECT_FALSE(std::is_nothrow_copy_assignable<DynamicTree::TreeNode>::value);
    EXPECT_FALSE(std::is_trivially_copy_assignable<DynamicTree::TreeNode>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_assignable<DynamicTree::TreeNode>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<DynamicTree::TreeNode>::value);
#endif
    
    EXPECT_TRUE(std::is_destructible<DynamicTree::TreeNode>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree::TreeNode>::value);
    EXPECT_TRUE(std::is_trivially_destructible<DynamicTree::TreeNode>::value);
}

TEST(DynamicTreeUnusedData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_trivially_default_constructible<DynamicTree::UnusedData>::value);
    
    EXPECT_TRUE(std::is_nothrow_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_trivially_constructible<DynamicTree::UnusedData>::value);
    
    EXPECT_TRUE(std::is_copy_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<DynamicTree::UnusedData>::value);
    
    EXPECT_TRUE(std::is_copy_assignable<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<DynamicTree::UnusedData>::value);
    
    EXPECT_TRUE(std::is_destructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree::UnusedData>::value);
    EXPECT_TRUE(std::is_trivially_destructible<DynamicTree::UnusedData>::value);
}

TEST(DynamicTreeBranchData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_trivially_default_constructible<DynamicTree::BranchData>::value);

    EXPECT_TRUE(std::is_copy_constructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<DynamicTree::BranchData>::value);

    EXPECT_TRUE(std::is_copy_assignable<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<DynamicTree::BranchData>::value);

    EXPECT_TRUE(std::is_destructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree::BranchData>::value);
    EXPECT_TRUE(std::is_trivially_destructible<DynamicTree::BranchData>::value);
}

TEST(DynamicTreeLeafData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree::LeafData>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<DynamicTree::LeafData>::value);

    EXPECT_TRUE(std::is_copy_constructible<DynamicTree::LeafData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<DynamicTree::LeafData>::value);

    EXPECT_TRUE(std::is_copy_assignable<DynamicTree::LeafData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<DynamicTree::LeafData>::value);

    EXPECT_TRUE(std::is_destructible<DynamicTree::LeafData>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree::LeafData>::value);
}

TEST(DynamicTreeVariantData, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<DynamicTree::VariantData>::value);
    //EXPECT_TRUE(std::is_trivially_default_constructible<DynamicTree::VariantData>::value);

    EXPECT_TRUE(std::is_nothrow_constructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_constructible<DynamicTree::VariantData>::value);
    //EXPECT_TRUE(std::is_trivially_constructible<DynamicTree::VariantData>::value);

    EXPECT_TRUE(std::is_copy_constructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<DynamicTree::VariantData>::value);

    EXPECT_TRUE(std::is_copy_assignable<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<DynamicTree::VariantData>::value);

    EXPECT_TRUE(std::is_destructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<DynamicTree::VariantData>::value);
    EXPECT_TRUE(std::is_trivially_destructible<DynamicTree::VariantData>::value);
}

// Test class for testing move semantics.
//
// "Move constructors typically 'steal' the resources held by the argument...
//  and leave the argument in some valid but otherwise indeterminate state."
//
// For details on move constructors, see:
//   http://en.cppreference.com/w/cpp/language/move_constructor
//
class Foo {
public:
    static unsigned instantiated;
    bool defaultConstructed{false};
    bool moveConstructed{false};
    bool constMoveConstructed{false};
    bool copyConstructed{false};
    bool released{false};
    unsigned srcInstanceId{0};
    unsigned copyAssigned{0};
    unsigned moveAssigned{0};

    Foo() noexcept: defaultConstructed{true}, srcInstanceId{instantiated}
    {
        ++instantiated;
    };
    
    Foo(const Foo& other) noexcept: copyConstructed{true}, srcInstanceId{other.srcInstanceId}
    {
        ++instantiated;
    };
    
    // Move constructor
    Foo(Foo&& other) noexcept: moveConstructed{true}, srcInstanceId{std::move(other.srcInstanceId)}
    {
        ++instantiated;
        other.released = true;
    };
    
    // Move constructor
    Foo(const Foo&& other) noexcept:
        constMoveConstructed{true}, srcInstanceId{std::move(other.srcInstanceId)}
    {
        ++instantiated;
        // can't update other since const-qualified: other.released = true;
    };

    Foo& operator= (const Foo& other) noexcept
    {
        srcInstanceId = other.srcInstanceId;
        ++copyAssigned;
        released = false;
        return *this;
    }
    
    Foo& operator= (Foo&& other) noexcept
    {
        // See: http://en.cppreference.com/w/cpp/language/move_assignment
        // other is left in "some valid but otherwise indeterminate state".
        srcInstanceId = other.srcInstanceId;
        ++moveAssigned;
        released = false;
        other.released = true;
        return *this;
    }
};

unsigned Foo::instantiated = 0;

TEST(DynamicTree, Basis)
{
    // foo should be default constructed since constructed from value initialization
    const auto foo = Foo{};
    EXPECT_TRUE(foo.defaultConstructed);
    EXPECT_FALSE(foo.constMoveConstructed);
    EXPECT_FALSE(foo.moveConstructed);
    EXPECT_FALSE(foo.copyConstructed);
    EXPECT_FALSE(foo.released);
    EXPECT_EQ(foo.srcInstanceId, 0u);
    EXPECT_EQ(foo.copyAssigned, 0u);
    EXPECT_EQ(foo.moveAssigned, 0u);
    EXPECT_EQ(Foo::instantiated, 1u);
    
    // boo should be copy constructed since constructed by assignment.
    auto boo = foo;
    EXPECT_TRUE(boo.copyConstructed);
    EXPECT_FALSE(boo.constMoveConstructed);
    EXPECT_FALSE(boo.moveConstructed);
    EXPECT_FALSE(boo.defaultConstructed);
    EXPECT_FALSE(boo.released);
    EXPECT_EQ(boo.srcInstanceId, 0u);
    EXPECT_EQ(boo.copyAssigned, 0u);
    EXPECT_EQ(boo.moveAssigned, 0u);
    EXPECT_FALSE(foo.released);
    EXPECT_EQ(Foo::instantiated, 2u);
    
    // moo should be copy constructed since constructed from a const value
    const auto moo = foo;
    EXPECT_TRUE(moo.copyConstructed);
    EXPECT_FALSE(moo.constMoveConstructed);
    EXPECT_FALSE(moo.moveConstructed);
    EXPECT_FALSE(moo.defaultConstructed);
    EXPECT_FALSE(moo.released);
    EXPECT_EQ(moo.srcInstanceId, 0u);
    EXPECT_EQ(moo.copyAssigned, 0u);
    EXPECT_EQ(moo.moveAssigned, 0u);
    EXPECT_FALSE(foo.released);
    EXPECT_EQ(Foo::instantiated, 3u);

    // roo should be const move constructed since constructed from a moved const value
    const auto roo = std::move(foo);
    EXPECT_TRUE(roo.constMoveConstructed);
    EXPECT_FALSE(roo.copyConstructed);
    EXPECT_FALSE(roo.moveConstructed);
    EXPECT_FALSE(roo.defaultConstructed);
    EXPECT_FALSE(roo.released);
    EXPECT_EQ(roo.srcInstanceId, 0u);
    EXPECT_EQ(roo.copyAssigned, 0u);
    EXPECT_EQ(roo.moveAssigned, 0u);
    EXPECT_FALSE(foo.released);
    EXPECT_EQ(Foo::instantiated, 4u);

    // yoo should be move constructed since constructed from a non-const value
    auto yoo = std::move(boo);
    EXPECT_TRUE(yoo.moveConstructed);
    EXPECT_FALSE(yoo.constMoveConstructed);
    EXPECT_FALSE(yoo.copyConstructed);
    EXPECT_FALSE(yoo.defaultConstructed);
    EXPECT_FALSE(yoo.released);
    EXPECT_EQ(yoo.srcInstanceId, 0u);
    EXPECT_EQ(yoo.copyAssigned, 0u);
    EXPECT_EQ(yoo.moveAssigned, 0u);
    EXPECT_TRUE(boo.released);
    EXPECT_EQ(Foo::instantiated, 5u);
    
    // loo should be copy constructed since constructed by assignment.
    const auto loo = boo;
    EXPECT_TRUE(loo.copyConstructed);
    EXPECT_FALSE(loo.moveConstructed);
    EXPECT_FALSE(loo.constMoveConstructed);
    EXPECT_FALSE(loo.defaultConstructed);
    EXPECT_FALSE(loo.released);
    EXPECT_EQ(loo.srcInstanceId, 0u);
    EXPECT_EQ(loo.copyAssigned, 0u);
    EXPECT_EQ(loo.moveAssigned, 0u);
    EXPECT_TRUE(boo.released);
    EXPECT_EQ(Foo::instantiated, 6u);
    
    const auto koo = Foo{};
    
    // yoo should remain move constructed and become copy assigned since assigned from koo
    yoo = koo;
    EXPECT_TRUE(yoo.moveConstructed);
    EXPECT_FALSE(yoo.constMoveConstructed);
    EXPECT_FALSE(yoo.copyConstructed);
    EXPECT_FALSE(yoo.defaultConstructed);
    EXPECT_FALSE(yoo.released);
    EXPECT_EQ(yoo.srcInstanceId, 6u);
    EXPECT_EQ(yoo.copyAssigned, 1u);
    EXPECT_EQ(yoo.moveAssigned, 0u);
    EXPECT_FALSE(koo.released);
    EXPECT_EQ(Foo::instantiated, 7u);
    
    boo = loo;
    EXPECT_TRUE(boo.copyConstructed);
    EXPECT_FALSE(boo.moveConstructed);
    EXPECT_FALSE(boo.constMoveConstructed);
    EXPECT_FALSE(boo.defaultConstructed);
    EXPECT_FALSE(boo.released);
    EXPECT_EQ(boo.srcInstanceId, 0u);
    EXPECT_EQ(boo.copyAssigned, 1u);
    EXPECT_EQ(boo.moveAssigned, 0u);
    EXPECT_FALSE(loo.released);
    EXPECT_EQ(Foo::instantiated, 7u);

    // yoo should remain move constructed and become additionally move assigned since assigned from boo
    yoo = std::move(boo);
    EXPECT_TRUE(yoo.moveConstructed);
    EXPECT_FALSE(yoo.constMoveConstructed);
    EXPECT_FALSE(yoo.copyConstructed);
    EXPECT_FALSE(yoo.defaultConstructed);
    EXPECT_FALSE(yoo.released);
    EXPECT_EQ(yoo.srcInstanceId, 0u);
    EXPECT_EQ(yoo.copyAssigned, 1u);
    EXPECT_EQ(yoo.moveAssigned, 1u);
    EXPECT_TRUE(boo.released);
    EXPECT_EQ(Foo::instantiated, 7u);
}

TEST(DynamicTree, DefaultConstruction)
{
    DynamicTree foo;
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size{0});
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetFreeIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(0));
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
    EXPECT_EQ(foo.FindReference(DynamicTree::GetInvalidSize()), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(0)), DynamicTree::GetInvalidSize());
}

TEST(DynamicTreeLeafData, DefaultConstructor)
{
    EXPECT_EQ(DynamicTree::LeafData().body, BodyID(0u));
    EXPECT_EQ(DynamicTree::LeafData().shape, ShapeID(0u));
    EXPECT_EQ(DynamicTree::LeafData().childIndex, ChildCounter(0u));
}

TEST(DynamicTree, ZeroCapacityConstructionSameAsDefault)
{
    EXPECT_EQ((DynamicTree{DynamicTree::Size{0}}.GetNodeCapacity()), DynamicTree{}.GetNodeCapacity());
    EXPECT_EQ((DynamicTree{DynamicTree::Size{0}}.GetNodeCount()), DynamicTree{}.GetNodeCount());
    EXPECT_EQ((DynamicTree{DynamicTree::Size{0}}.GetRootIndex()), DynamicTree{}.GetRootIndex());
    EXPECT_EQ((DynamicTree{DynamicTree::Size{0}}.GetFreeIndex()), DynamicTree{}.GetFreeIndex());
}

TEST(DynamicTree, InitializingConstruction)
{
    constexpr auto initCapacity = 128u;
    DynamicTree foo(initCapacity);
    EXPECT_EQ(foo.GetNodeCapacity(), initCapacity);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
    EXPECT_EQ(foo.FindReference(DynamicTree::GetInvalidSize()),
              foo.GetNodeCapacity() - 1u);
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
        EXPECT_EQ(GetMaxImbalance(copy), GetMaxImbalance(orig));
    }

    const auto aabb = AABB{
        Length2{0_m, 0_m},
        Length2(1_m, 1_m)
    };
    const auto pid = orig.CreateLeaf(aabb, DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u});
    {
        DynamicTree copy{orig};
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(GetMaxImbalance(copy), GetMaxImbalance(orig));
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
        EXPECT_EQ(GetMaxImbalance(copy), GetMaxImbalance(orig));
    }
    
    const auto aabb = AABB{
        Length2{0_m, 0_m},
        Length2{1_m, 1_m}
    };
    const auto pid = orig.CreateLeaf(aabb, DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u});
    EXPECT_EQ(orig.FindReference(pid), DynamicTree::GetInvalidSize());
    {
        DynamicTree copy;
        copy = orig;
        EXPECT_EQ(copy.GetRootIndex(), orig.GetRootIndex());
        EXPECT_EQ(copy.GetNodeCapacity(), orig.GetNodeCapacity());
        EXPECT_EQ(GetHeight(copy), GetHeight(orig));
        EXPECT_EQ(ComputePerimeterRatio(copy), ComputePerimeterRatio(orig));
        EXPECT_EQ(copy.GetNodeCount(), orig.GetNodeCount());
        EXPECT_EQ(GetMaxImbalance(copy), GetMaxImbalance(orig));
        EXPECT_EQ(copy.GetLeafData(pid), orig.GetLeafData(pid));
        EXPECT_EQ(copy.FindReference(pid), DynamicTree::GetInvalidSize());
    }
}

TEST(DynamicTree, CreateAndDestroyProxy)
{
    DynamicTree foo;

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));

    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto userdata = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};

    const auto pid = foo.CreateLeaf(aabb, userdata);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    const auto nodeCapacity = foo.GetNodeCapacity();
    EXPECT_GE(nodeCapacity, foo.GetNodeCount());
    EXPECT_EQ(foo.GetAABB(pid), aabb);
    EXPECT_EQ(foo.GetLeafData(pid), userdata);
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(1));

    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(0));

    foo.DestroyLeaf(pid);
    EXPECT_EQ(foo.GetNodeCapacity(), nodeCapacity);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(0));
}

TEST(DynamicTree, FourIdenticalProxies)
{
    DynamicTree foo;
    auto nodeCapacity = DynamicTree::Size{};

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    ASSERT_TRUE(ValidateStructure(foo, DynamicTree::GetInvalidSize()));
    ASSERT_TRUE(ValidateStructure(foo, foo.GetFreeIndex()));
    ASSERT_FALSE(ValidateStructure(foo, foo.GetNodeCapacity() + 1));
    ASSERT_FALSE(ValidateMetrics(foo, foo.GetNodeCapacity() + 1));
    ASSERT_TRUE(ValidateMetrics(foo, DynamicTree::GetInvalidSize()));
    ASSERT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    ASSERT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
    ASSERT_EQ(size(foo), 0u);

    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto leafData = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(0));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        ASSERT_EQ(foo.GetRootIndex(), pid);
        EXPECT_TRUE(ValidateStructure(foo, pid));
        EXPECT_TRUE(ValidateMetrics(foo, pid));
        EXPECT_TRUE(ValidateStructure(foo, foo.GetFreeIndex()));
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    nodeCapacity = foo.GetNodeCapacity();
    EXPECT_GE(nodeCapacity, foo.GetNodeCount());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(1));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(size(foo), 1u);

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(1));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(ValidateStructure(foo, pid));
        EXPECT_TRUE(ValidateMetrics(foo, pid));
        EXPECT_TRUE(ValidateStructure(foo, foo.GetFreeIndex()));
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(3));
    EXPECT_GE(foo.GetNodeCapacity(), nodeCapacity);
    nodeCapacity = foo.GetNodeCapacity();
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(1));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(3));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(1));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(2));
    EXPECT_EQ(size(foo), 2u);

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(3));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(ValidateStructure(foo, pid));
        EXPECT_TRUE(ValidateMetrics(foo, pid));
        EXPECT_TRUE(ValidateStructure(foo, foo.GetFreeIndex()));
    }
    
    EXPECT_TRUE(DynamicTree::IsBranch(foo.GetHeight(DynamicTree::Size(4))));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(5));
    EXPECT_GE(foo.GetNodeCapacity(), nodeCapacity);
    nodeCapacity = foo.GetNodeCapacity();
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(1));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(5));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(3));
    EXPECT_EQ(size(foo), 3u);

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(5));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(ValidateStructure(foo, pid));
        EXPECT_TRUE(ValidateMetrics(foo, pid));
        EXPECT_TRUE(ValidateStructure(foo, foo.GetFreeIndex()));
    }
    
    EXPECT_TRUE(DynamicTree::IsLeaf(foo.GetHeight(DynamicTree::Size(5))));
    EXPECT_TRUE(DynamicTree::IsBranch(foo.GetHeight(DynamicTree::Size(6))));
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(5)), DynamicTree::Size(6));
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(6)), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));
    EXPECT_EQ(size(foo), 4u);

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_GE(foo.GetNodeCapacity(), nodeCapacity);
    nodeCapacity = foo.GetNodeCapacity();
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(7));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));

    EXPECT_FALSE(ValidateStructure(foo, foo.GetNodeCapacity() + 1));
    EXPECT_FALSE(ValidateMetrics(foo, foo.GetNodeCapacity() + 1));
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
    
    foo.RebuildBottomUp();
    
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_GE(foo.GetNodeCapacity(), nodeCapacity);
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(3));
    EXPECT_EQ(GetMaxImbalance(foo), DynamicTree::Height(2));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(7));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(3));
}

TEST(DynamicTree, MoveConstruction)
{
    DynamicTree foo;
    
    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };

    const auto leafData = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};

    const auto leaf0 = foo.CreateLeaf(aabb, leafData);
    const auto leaf1 = foo.CreateLeaf(aabb, leafData);
    const auto leaf2 = foo.CreateLeaf(aabb, leafData);
    const auto leaf3 = foo.CreateLeaf(aabb, leafData);

    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::Size(4));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    ASSERT_GE(foo.GetNodeCapacity(), foo.GetNodeCount());
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));

    DynamicTree roo{std::move(foo)};

    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    
    EXPECT_EQ(roo.GetRootIndex(), DynamicTree::Size(4));
    EXPECT_EQ(roo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_GE(roo.GetNodeCapacity(), roo.GetNodeCount());
    EXPECT_EQ(roo.GetLeafCount(), DynamicTree::Size(4));
    
    EXPECT_EQ(roo.GetAABB(leaf0), aabb);
    EXPECT_EQ(roo.GetAABB(leaf1), aabb);
    EXPECT_EQ(roo.GetAABB(leaf2), aabb);
    EXPECT_EQ(roo.GetAABB(leaf3), aabb);
}

TEST(DynamicTree, MoveAssignment)
{
    DynamicTree foo;
    
    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    
    const auto leafData = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};

    const auto leaf0 = foo.CreateLeaf(aabb, leafData);
    const auto leaf1 = foo.CreateLeaf(aabb, leafData);
    const auto leaf2 = foo.CreateLeaf(aabb, leafData);
    const auto leaf3 = foo.CreateLeaf(aabb, leafData);
    
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::Size(4));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    ASSERT_GE(foo.GetNodeCapacity(), foo.GetNodeCount());
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));
    
    DynamicTree roo;
    
    roo = std::move(foo);
    
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetFreeIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    
    EXPECT_EQ(roo.GetRootIndex(), DynamicTree::Size(4));
    EXPECT_EQ(roo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_GE(roo.GetNodeCapacity(), roo.GetNodeCount());
    EXPECT_EQ(roo.GetLeafCount(), DynamicTree::Size(4));
    
    EXPECT_EQ(roo.GetAABB(leaf0), aabb);
    EXPECT_EQ(roo.GetAABB(leaf1), aabb);
    EXPECT_EQ(roo.GetAABB(leaf2), aabb);
    EXPECT_EQ(roo.GetAABB(leaf3), aabb);
}

TEST(DynamicTree, CreateLeaf)
{
    DynamicTree foo{DynamicTree::Size{1}};
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));

    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto leafData = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};

    const auto l1 = foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    const auto p1 = foo.GetOther(l1);
    EXPECT_TRUE(p1 == DynamicTree::GetInvalidSize());

    const auto l2 = foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(2));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(4));
    const auto p2 = foo.GetOther(l2);
    EXPECT_TRUE(foo.GetBranchData(p2).child1 == l2 || foo.GetBranchData(p2).child2 == l2);
    ASSERT_NE(foo.GetOther(l1), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l1)).child1 == l1 || foo.GetBranchData(foo.GetOther(l1)).child2 == l1);

    const auto l3 = foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(5));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(8));
    const auto p3 = foo.GetOther(l3);
    EXPECT_TRUE(foo.GetBranchData(p3).child1 == l3 || foo.GetBranchData(p3).child2 == l3);
    ASSERT_NE(foo.GetOther(l2), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l2)).child1 == l2 || foo.GetBranchData(foo.GetOther(l2)).child2 == l2);
    ASSERT_NE(foo.GetOther(l1), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l1)).child1 == l1 || foo.GetBranchData(foo.GetOther(l1)).child2 == l1);

    const auto l4 = foo.CreateLeaf(aabb, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(4));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(8));
    const auto p4 = foo.GetOther(l4);
    EXPECT_TRUE(foo.GetBranchData(p4).child1 == l4 || foo.GetBranchData(p4).child2 == l4);
    ASSERT_NE(foo.GetOther(l3), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l3)).child1 == l3 || foo.GetBranchData(foo.GetOther(l3)).child2 == l3);
    ASSERT_NE(foo.GetOther(l2), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l2)).child1 == l2 || foo.GetBranchData(foo.GetOther(l2)).child2 == l2);
    ASSERT_NE(foo.GetOther(l1), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l1)).child1 == l1 || foo.GetBranchData(foo.GetOther(l1)).child2 == l1);

    const auto l5 = foo.CreateLeaf(AABB{Length2{2_m, 4_m}, Length2{-1_m, 2_m}}, leafData);
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(5));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(9));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(16));
    const auto p5 = foo.GetOther(l5);
    EXPECT_TRUE(foo.GetBranchData(p5).child1 == l5 || foo.GetBranchData(p5).child2 == l5);
    ASSERT_NE(foo.GetOther(l4), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l4)).child1 == l4 || foo.GetBranchData(foo.GetOther(l4)).child2 == l4);
    ASSERT_NE(foo.GetOther(l3), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l3)).child1 == l3 || foo.GetBranchData(foo.GetOther(l3)).child2 == l3);
    ASSERT_NE(foo.GetOther(l2), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l2)).child1 == l2 || foo.GetBranchData(foo.GetOther(l2)).child2 == l2);
    ASSERT_NE(foo.GetOther(l1), DynamicTree::GetInvalidSize());
    EXPECT_TRUE(foo.GetBranchData(foo.GetOther(l1)).child1 == l1 || foo.GetBranchData(foo.GetOther(l1)).child2 == l1);
}

TEST(DynamicTree, UpdateLeaf)
{
    auto leafs{std::vector<DynamicTree::Size>{}};

    DynamicTree foo{DynamicTree::Size{1}};
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    
    const auto aabb = AABB{Length2{3_m, 1_m}, Length2{-5_m, -2_m}};
    const auto leafData = DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0u};
    
    leafs.push_back(foo.CreateLeaf(aabb, leafData)); // 1
    leafs.push_back(foo.CreateLeaf(aabb, leafData)); // 2
    leafs.push_back(foo.CreateLeaf(aabb, leafData)); // 3
    leafs.push_back(foo.CreateLeaf(aabb, leafData)); // 4
    leafs.push_back(foo.CreateLeaf(AABB{Length2{2_m, 4_m}, Length2{-1_m, 2_m}}, leafData)); // 5
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(5));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(9));
    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(16));
    
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[0], AABB{Length2{1.5_m, -2_m}, Length2{-1_m, -3_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[1], AABB{Length2{10_m, 12_m}, Length2{1_m, 3_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[2], AABB{Length2{4_m, 5_m}, Length2{2_m, -2_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[3], AABB{Length2{2_m, 3_m}, Length2{5_m, 6_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[4], AABB{Length2{1.5_m, -2_m}, Length2{-1_m, -2.5_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[1], AABB{Length2{1_m, 2_m}, Length2{-2_m, -3_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });

    leafs.push_back(foo.CreateLeaf(AABB{Length2{-2_m, -4_m}, Length2{1_m, -2_m}}, leafData)); // 6
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(6));
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });

    foo.UpdateLeaf(leafs[2], AABB{Length2{-4_m, -5_m}, Length2{2_m, -2_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });

    leafs.push_back(foo.CreateLeaf(AABB{Length2{-0.2_m, -0.3_m}, Length2{4.1_m, 4.2_m}}, leafData)); // 7
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(7));
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });

    leafs.push_back(foo.CreateLeaf(AABB{Length2{-0.2_m, -0.3_m}, Length2{4.1_m, 4.2_m}}, leafData)); // 8
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(8));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(15));
    EXPECT_EQ(foo.GetHeight(foo.GetRootIndex()), DynamicTree::Height{4});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[0], AABB{Length2{10.5_m, 8_m}, Length2{-1_m, -3_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[7], AABB{Length2{-1.2_m, -1.3_m}, Length2{4.1_m, 4.2_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    leafs.push_back(foo.CreateLeaf(AABB{Length2{10_m, -10_m}, Length2{1.1_m, 11_m}}, leafData)); // 9
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(9));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(17));
    EXPECT_EQ(foo.GetHeight(foo.GetRootIndex()), DynamicTree::Height{4});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    foo.UpdateLeaf(leafs[8], AABB{Length2{-20_m, -11_m}, Length2{1.1_m, 11_m}});
    std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
        ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
        EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
    });
    
    
    for (auto i = 0; i < 200; ++i)
    {
        leafs.push_back(foo.CreateLeaf(AABB{Length2{11_m + i * 1_m, 0_m + i * 1_m}, Length2{1.1_m, 11_m}}, leafData));
    }
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(209));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(417));
    EXPECT_EQ(foo.GetHeight(foo.GetRootIndex()), DynamicTree::Height{8});

    std::for_each(begin(leafs), end(leafs), [&foo,&leafs](const auto leaf) {
        const auto aabb{foo.GetAABB(leaf)};
        foo.UpdateLeaf(leaf, AABB{aabb.ranges[1], aabb.ranges[0]});
        std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
            ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
            EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
        });
    });
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));

    std::for_each(begin(leafs), end(leafs), [&foo,&leafs](const auto leaf) {
        const auto aabb{foo.GetAABB(leaf)};
        foo.UpdateLeaf(leaf, GetMovedAABB(aabb, Length2{6_m, 0_m}));
        std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
            ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
            EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
        });
    });
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));

    std::for_each(begin(leafs), end(leafs), [&foo,&leafs](const auto leaf) {
        const auto aabb{foo.GetAABB(leaf)};
        foo.UpdateLeaf(leaf, GetFattenedAABB(aabb, 0.5_m));
        std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
            ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
            EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
        });
    });
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));

    std::for_each(begin(leafs), end(leafs), [&foo,&leafs](const auto leaf) {
        foo.UpdateLeaf(leaf, AABB{Length2{}, Length2{}});
        std::for_each(begin(leafs), end(leafs), [&foo](const auto leaf) {
            ASSERT_NE(foo.GetOther(leaf), DynamicTree::GetInvalidSize());
            EXPECT_TRUE(foo.GetBranchData(foo.GetOther(leaf)).child1 == leaf || foo.GetBranchData(foo.GetOther(leaf)).child2 == leaf);
        });
    });
    EXPECT_TRUE(ValidateStructure(foo, foo.GetRootIndex()));
    EXPECT_TRUE(ValidateMetrics(foo, foo.GetRootIndex()));
}

TEST(DynamicTree, Clear)
{
    DynamicTree foo{};

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    ASSERT_EQ(foo.GetFreeIndex(), DynamicTree::GetInvalidSize());
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());

    EXPECT_NO_THROW(foo.Clear());
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetFreeIndex(), DynamicTree::GetInvalidSize());
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());

    ASSERT_NO_THROW(foo.CreateLeaf(AABB{}, DynamicTree::LeafData()));
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    ASSERT_GE(foo.GetNodeCapacity(), DynamicTree::Size(1));
    ASSERT_EQ(foo.GetLeafCount(), DynamicTree::Size(1));
    ASSERT_EQ(foo.GetFreeIndex(), DynamicTree::GetInvalidSize());
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::Size(0));

    EXPECT_NO_THROW(foo.Clear());
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetFreeIndex(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());

    const auto capacity = foo.GetNodeCapacity();
    auto numLeafs = foo.GetLeafCount();
    while (foo.GetNodeCount() <= capacity)
    {
        ASSERT_NO_THROW(foo.CreateLeaf(AABB{}, DynamicTree::LeafData()));
        ASSERT_GT(foo.GetLeafCount(), numLeafs);
        ASSERT_GE(foo.GetNodeCapacity(), capacity);
        numLeafs = foo.GetLeafCount();
    }
    ASSERT_EQ(foo.GetNodeCapacity(), 4u);

    EXPECT_NO_THROW(foo.Clear());
    EXPECT_EQ(foo.GetNodeCapacity(), 4u);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetLeafCount(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetFreeIndex(), DynamicTree::Size(0));
    EXPECT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
}

TEST(DynamicTree, QueryFF)
{
    auto foo = DynamicTree{};
    auto ncalls = 0;
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{}, DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0});
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{LengthInterval{-10_m, 10_m}, LengthInterval{-20_m, 20_m}},
                   DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0});
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{LengthInterval{-10_m, 10_m}, LengthInterval{-20_m, 20_m}},
                   DynamicTree::LeafData{BodyID(1u), ShapeID(0u), 0});
    Query(foo, AABB{LengthInterval{-20_m, 20_m}, LengthInterval{-20_m, 20_m}}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 1);
    ncalls = 0;
    Query(foo, AABB{LengthInterval{-20_m, 20_m}, LengthInterval{-20_m, 20_m}}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::Continue;
    });
    EXPECT_EQ(ncalls, 2);
}
