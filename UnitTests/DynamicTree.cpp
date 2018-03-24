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
#include <PlayRho/Collision/DynamicTree.hpp>
#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

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
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(36));
#else
            EXPECT_EQ(sizeof(DynamicTree::TreeNode), std::size_t(48));
#endif
            break;
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
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(0));
    EXPECT_TRUE(foo.Validate());
    EXPECT_EQ(foo.FindReference(DynamicTree::GetInvalidSize()),
              foo.GetNodeCapacity() - 1u);
    ASSERT_GT(foo.GetNodeCapacity(), DynamicTree::Size(0));
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(0)),
              DynamicTree::GetInvalidSize());
}

TEST(DynamicTree, ZeroCapacityConstructionThrows)
{
    EXPECT_THROW(DynamicTree{DynamicTree::Size{0}}, InvalidArgument);
}

TEST(DynamicTree, InitializingConstruction)
{
    constexpr const auto initCapacity = DynamicTree::GetDefaultInitialNodeCapacity() * 2;
    DynamicTree foo{initCapacity};
    EXPECT_EQ(foo.GetNodeCapacity(), initCapacity);
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));
    EXPECT_TRUE(foo.Validate());
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
        EXPECT_EQ(copy.GetMaxBalance(), orig.GetMaxBalance());
    }

    const auto aabb = AABB{
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
    
    const auto aabb = AABB{
        Length2{0_m, 0_m},
        Length2{1_m, 1_m}
    };
    const auto pid = orig.CreateLeaf(aabb, DynamicTree::LeafData{nullptr, nullptr, 0u});
    EXPECT_EQ(orig.FindReference(pid), DynamicTree::GetInvalidSize());
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
        EXPECT_EQ(copy.FindReference(pid), DynamicTree::GetInvalidSize());
    }
}

TEST(DynamicTree, CreateAndDestroyProxy)
{
    DynamicTree foo;

    ASSERT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    ASSERT_EQ(foo.GetNodeCount(), DynamicTree::Size(0));

    const auto aabb = AABB{
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
    ASSERT_EQ(foo.GetRootIndex(), DynamicTree::GetInvalidSize());
    ASSERT_TRUE(foo.ValidateStructure(DynamicTree::GetInvalidSize()));
    ASSERT_FALSE(foo.ValidateStructure(foo.GetNodeCapacity() + 1));
    ASSERT_FALSE(foo.ValidateMetrics(foo.GetNodeCapacity() + 1));
    ASSERT_TRUE(foo.ValidateMetrics(DynamicTree::GetInvalidSize()));
    ASSERT_TRUE(foo.Validate());

    const auto aabb = AABB{
        Length2{3_m, 1_m},
        Length2{-5_m, -2_m}
    };
    const auto leafData = DynamicTree::LeafData{nullptr, nullptr, 0u};

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(0));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        ASSERT_EQ(foo.GetRootIndex(), pid);
        EXPECT_TRUE(foo.ValidateStructure(pid));
        EXPECT_TRUE(foo.ValidateMetrics(pid));
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(1));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(0));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(1));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(0));

    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(1));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(foo.ValidateStructure(pid));
        EXPECT_TRUE(foo.ValidateMetrics(pid));
    }

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(3));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(1));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(3));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(1));
    
    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(3));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(foo.ValidateStructure(pid));
        EXPECT_TRUE(foo.ValidateMetrics(pid));
    }
    
    EXPECT_TRUE(DynamicTree::IsBranch(foo.GetHeight(DynamicTree::Size(4))));
    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(5));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(1));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(5));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));
    
    {
        const auto pid = foo.CreateLeaf(aabb, leafData);
        EXPECT_EQ(pid, DynamicTree::Size(5));
        EXPECT_EQ(foo.GetAABB(pid), aabb);
        EXPECT_EQ(foo.GetLeafData(pid), leafData);
        EXPECT_TRUE(foo.ValidateStructure(pid));
        EXPECT_TRUE(foo.ValidateMetrics(pid));
    }
    
    EXPECT_TRUE(DynamicTree::IsLeaf(foo.GetHeight(DynamicTree::Size(5))));
    EXPECT_TRUE(DynamicTree::IsBranch(foo.GetHeight(DynamicTree::Size(6))));
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(5)), DynamicTree::Size(6));
    EXPECT_EQ(foo.FindReference(DynamicTree::Size(6)), DynamicTree::Size(3));

    EXPECT_EQ(foo.GetNodeCount(), DynamicTree::Size(7));
    EXPECT_EQ(foo.GetNodeCapacity(), DynamicTree::GetDefaultInitialNodeCapacity());
    EXPECT_EQ(GetHeight(foo), DynamicTree::Height(2));
    EXPECT_EQ(foo.GetMaxBalance(), DynamicTree::Height(0));
    EXPECT_EQ(ComputePerimeterRatio(foo), Real(7));
    EXPECT_EQ(ComputeHeight(foo), DynamicTree::Height(2));

    EXPECT_TRUE(foo.Validate());
    
    foo.RebuildBottomUp();
    
    EXPECT_TRUE(foo.Validate());
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
    
    const auto aabb = AABB{
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
    
    const auto aabb = AABB{
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

    const auto aabb = AABB{
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

TEST(DynamicTree, QueryFF)
{
    auto foo = DynamicTree{};
    auto ncalls = 0;
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{}, DynamicTree::LeafData{nullptr, nullptr, 0});
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{LengthInterval{-10_m, 10_m}, LengthInterval{-20_m, 20_m}},
                   DynamicTree::LeafData{nullptr, nullptr, 0});
    Query(foo, AABB{}, [&] (DynamicTree::Size) {
        ++ncalls;
        return DynamicTreeOpcode::End;
    });
    EXPECT_EQ(ncalls, 0);
    foo.CreateLeaf(AABB{LengthInterval{-10_m, 10_m}, LengthInterval{-20_m, 20_m}},
                   DynamicTree::LeafData{nullptr, nullptr, 0});
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
