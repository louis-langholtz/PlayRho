/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_DYNAMICTREE_HPP
#define PLAYRHO_COLLISION_DYNAMICTREE_HPP

/// @file
/// Declaration of the <code>DynamicTree</code> class.

#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Common/Settings.hpp>

#include <functional>
#include <type_traits>
#include <utility>

namespace playrho {
namespace d2 {

class Fixture;
class Body;

/// @brief A dynamic AABB tree broad-phase.
///
/// @details A dynamic tree arranges data in a binary tree to accelerate
///   queries such as volume queries and ray casts. Leafs are proxies
///   with an AABB.
///
/// @note This code was inspired by Nathanael Presson's <code>btDbvt</code>.
/// @note Nodes are pooled and relocatable, so we use node indices rather than pointers.
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
///
/// @sa http://www.randygaul.net/2013/08/06/dynamic-aabb-tree/
/// @sa http://www.cs.utah.edu/~thiago/papers/rotations.pdf ("Fast, Effective
///    BVH Updates for Animated Scenes")
///
class DynamicTree
{
public:
    /// @brief Size type.
    using Size = ContactCounter;
    
    class TreeNode;
    struct UnusedData;
    struct BranchData;
    struct LeafData;
    union VariantData;
    
    /// @brief Gets the invalid size value.
    static PLAYRHO_CONSTEXPR inline Size GetInvalidSize() noexcept
    {
        return static_cast<Size>(-1);
    }
    
    /// @brief Strong type for heights.
    using Height = ContactCounter;
    
    /// @brief Invalid height constant value.
    static PLAYRHO_CONSTEXPR const auto InvalidHeight = static_cast<Height>(-1);

    /// @brief Gets the invalid height value.
    static PLAYRHO_CONSTEXPR inline Height GetInvalidHeight() noexcept
    {
        return InvalidHeight;
    }
    
    /// @brief Gets whether the given height is the height for an "unused" node.
    static PLAYRHO_CONSTEXPR inline bool IsUnused(Height value) noexcept
    {
        return value == GetInvalidHeight();
    }
    
    /// @brief Gets whether the given height is the height for a "leaf" node.
    static PLAYRHO_CONSTEXPR inline bool IsLeaf(Height value) noexcept
    {
        return value == 0;
    }
    
    /// @brief Gets whether the given height is a height for a "branch" node.
    static PLAYRHO_CONSTEXPR inline bool IsBranch(Height value) noexcept
    {
        return !IsUnused(value) && !IsLeaf(value);
    }

    /// @brief Gets the default initial node capacity.
    static PLAYRHO_CONSTEXPR inline Size GetDefaultInitialNodeCapacity() noexcept;

    /// @brief Constructing the tree initializes the node pool.
    explicit DynamicTree(Size nodeCapacity = GetDefaultInitialNodeCapacity());

    /// @brief Destroys the tree, freeing the node pool.
    ~DynamicTree() noexcept;

    /// @brief Copy constructor.
    DynamicTree(const DynamicTree& other);
    
    /// @brief Move constructor.
    DynamicTree(DynamicTree&& other) noexcept;

    /// @brief Copy assignment operator.
    DynamicTree& operator= (const DynamicTree& other);

    /// @brief Move assignment operator.
    DynamicTree& operator= (DynamicTree&& other) noexcept;

    /// @brief Creates a new leaf node.
    /// @details Creates a leaf node for a tight fitting AABB and the given data.
    /// @note The indices of leaf nodes that have been destroyed get reused for new nodes.
    /// @post If the root index had been the <code>GetInvalidSize()</code>, then it will
    ///   be set to the index returned from this method.
    /// @return Index of the created leaf node.
    Size CreateLeaf(const AABB& aabb, const LeafData& leafData);

    /// @brief Destroys a leaf node.
    /// @warning Behavior is undefined if the given index is not valid.
    void DestroyLeaf(Size index);

    /// @brief Updates a leaf node with a new AABB value.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Leaf node's ID. Behavior is undefined if this is not a valid ID.
    /// @param aabb New axis aligned bounding box for the leaf node.
    void UpdateLeaf(Size index, const AABB& aabb);

    /// @brief Gets the user data for the node identified by the given identifier.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Identifier of node to get the user data for.
    /// @return User data for the specified node.
    LeafData GetLeafData(Size index) const noexcept;

    /// @brief Sets the leaf data for the element at the given index to the given value.
    void SetLeafData(Size index, LeafData value) noexcept;

    /// @brief Gets the AABB for a leaf or branch (a non-unused node).
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Leaf or branch node's ID. Must be a valid ID.
    AABB GetAABB(Size index) const noexcept;

    /// @brief Gets the height value for the identified node.
    /// @warning Behavior is undefined if the given index is not valid.
    Height GetHeight(Size index) const noexcept;
    
    /// @brief Gets the branch data for the identified node.
    /// @warning Behavior is undefined if the given index in not a valid branch node.
    BranchData GetBranchData(Size index) const noexcept;

    /// @brief Validates this tree.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool Validate() const;

    /// @brief Validates the structure of this tree from the given index.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool ValidateStructure(Size index) const noexcept;

    /// @brief Validates the metrics of this tree from the given index.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool ValidateMetrics(Size index) const noexcept;

    /// @brief Gets the root index.
    Size GetRootIndex() const noexcept;

    /// @brief Gets the maximum balance.
    /// @details This gets the maximum balance of nodes in the tree.
    /// @note The balance is the difference in height of the two children of a node.
    Height GetMaxBalance() const noexcept;

    /// @brief Gets the ratio of the sum of the perimeters of nodes to the root perimeter.
    /// @note Zero is returned if no proxies exist at the time of the call.
    /// @return Value of zero or more.
    Length ComputeTotalPerimeter() const noexcept;

    /// @brief Builds an optimal tree.
    /// @note This operation is very expensive.
    /// @note Meant for testing.
    void RebuildBottomUp();

    /// @brief Shifts the world origin.
    /// @note Useful for large worlds.
    /// @note The shift formula is: <code>position -= newOrigin</code>.
    /// @param newOrigin the new origin with respect to the old origin.
    void ShiftOrigin(Length2 newOrigin);
    
    /// @brief Computes the height of the tree from a given node.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index ID of node to compute height from.
    /// @return 0 unless the given index is to a branch node.
    Height ComputeHeight(Size index) const noexcept;

    /// @brief Gets the current node capacity of this tree.
    Size GetNodeCapacity() const noexcept;

    /// @brief Gets the current node count.
    /// @return Count of existing proxies (count of nodes currently allocated).
    Size GetNodeCount() const noexcept;
    
    /// @brief Gets the current leaf node count.
    /// @details Gets the current leaf node count.
    Size GetLeafCount() const noexcept;

    /// @brief Finds the lowest cost node.
    /// @warning Behavior is undefined if the tree doesn't have a valid root.
    Size FindLowestCostNode(AABB leafAABB) const noexcept;
    
    /// @brief Finds first node which references the given index.
    /// @note Primarily intended for unit testing and/or debugging.
    /// @return Index of node referencing the given index, or the value of
    ///   <code>GetInvalidSize()</code>.
    Size FindReference(Size index) const noexcept;

private:
    
    /// @brief Sets the node capacity to the given value.
    void SetNodeCapacity(Size value) noexcept;

    /// @brief Allocates a node.
    /// @details This allocates a node from the free list that can be used as either a leaf
    ///   node or a branch node.
    Size AllocateNode() noexcept;

    /// @brief Allocates a leaf node.
    /// @details This allocates a node from the free list as a leaf node.
    Size AllocateNode(const LeafData& node, AABB aabb) noexcept;

    /// @brief Allocates a branch node.
    /// @details This allocates a node from the free list as a branch node.
    /// @post The free list no longer references the returned index.
    Size AllocateNode(const BranchData& node, AABB aabb, Height height,
                      Size parent = GetInvalidSize()) noexcept;
 
    /// @brief Frees the specified node.
    ///
    /// @warning Behavior is undefined if the given index is not valid.
    /// @warning Specified node must be a "leaf" or "branch" node.
    ///
    /// @pre Specified node's other index is the invalid size index.
    /// @pre Specified node isn't referenced by any other nodes.
    /// @post The free list links to the given index.
    ///
    void FreeNode(Size index) noexcept;

    /// @brief Inserts the specified node.
    /// @details Does a leaf insertion of the node with the given index into the dynamic tree.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @post The root index is set to the given index if the root index had been
    ///   <code>GetInvalidSize()</code>.
    void InsertLeaf(Size index);

    /// @brief Removes the specified node.
    /// @details Does a leaf removal of the node with the given index.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @post The specified leaf index is not referenced by any other nodes.
    /// @post The identified node has its other value set to the invalid size.
    void RemoveLeaf(Size leaf);
    
    /// @brief Rebalances the tree from the given index.
    /// @warning Behavior is undefined if the given index is not valid.
    void Rebalance(Size index);
    
    /// @brief Gets the parent node index of the node identified by the given index.
    Size GetParent(Size index) const noexcept;

    /// @brief Sets the parent node index of the node identified by the given index.
    void SetParent(Size index, Size newParent) noexcept;
    
    /// @brief Sets the height of the node identified by the given index to the given value.
    void SetHeight(Size index, Height value) noexcept;

    /// @brief Gets the child-1 index value of the node identified by the given index.
    Size GetChild1(Size index) const noexcept;

    /// @brief Sets the child-1 index value of the node identified by the given index.
    void SetChild1(Size index, Size value) noexcept;
    
    /// @brief Gets the child-2 index value of the node identified by the given index.
    Size GetChild2(Size index) const noexcept;

    /// @brief Sets the child-2 index value of the node identified by the given index.
    void SetChild2(Size index, Size value) noexcept;
    
    /// @brief Sets the AABB of the node identified by the given index to the given value.
    void SetAABB(Size index, AABB value) noexcept;
    
    /// @brief Swaps the child index of the node identified by the given index.
    void SwapChild(Size index, Size oldChild, Size newChild) noexcept;
    
    /// @brief Sets the child indexes for the node identified by the given index.
    void SetChildren(Size index, Size child1, Size child2) noexcept;
    
    TreeNode* m_nodes; ///< Nodes. @details Initialized on construction.

    Size m_root = GetInvalidSize(); ///< Index of root element in m_nodes or <code>GetInvalidSize()</code>.

    Size m_nodeCount = Size{0}; ///< Node count. @details Count of currently allocated nodes.
    Size m_nodeCapacity; ///< Node capacity. @details Size of buffer allocated for nodes.

    Size m_leafCount = 0; ///< Leaf count. @details Count of currently allocated leaf nodes.

    Size m_freeListIndex = Size{0}; ///< Free list. @details Index to free nodes.
};

/// @brief Unused data of a tree node.
struct DynamicTree::UnusedData
{
    // Intentionally empty.
    // This exists for symetry and as placeholder in case this needs to later be used.
};

/// @brief Branch data of a tree node.
struct DynamicTree::BranchData
{
    Size child1; ///< @brief Child 1.
    Size child2; ///< @brief Child 2.
};

/// @brief Leaf data of a tree node.
/// @details This is the leaf node specific data for a <code>DynamicTree::TreeNode</code>.
///   It's data that only pertains to leaf nodes.
/// @note This class is used in the <code>DynamicTree::VariantData</code> union within a
///   <code>DynamicTree::TreeNode</code>.
///   This has ramifications on this class's data contents and size.
struct DynamicTree::LeafData
{
    // In terms of what needs to be in this structure, it minimally needs to have enough
    // information in it to identify the child shape for which the node's AABB represents,
    // and its associated body. A pointer to the fixture and the index of the child in
    // its shape could suffice for this. Meanwhile, a Contact is defined to be the
    // recognition of an overlap between two child shapes having different bodies making
    // the caching of the bodies a potential speed-up opportunity.

    /// @brief Cached pointer to associated body.
    /// @note This field serves merely to potentially avoid the lookup of the body through
    ///   the fixture. It may or may not be worth the extra 8-bytes or so required for it.
    /// @note On 64-bit architectures, this is an 8-byte sized field. As an 8-byte field it
    ///   conceptually identifies 2^64 separate bodies within a world. As a practical matter
    ///   however, even a 4-byte index which could identify 2^32 bodies, is still larger than
    ///   is usable. This suggests that space could be saved by using indexes into arrays of
    ///   bodies instead of direct pointers to memory.
    Body* body;
    
    /// @brief Pointer to associated Fixture.
    /// @note On 64-bit architectures, this is an 8-byte sized field. As an 8-byte field it
    ///   conceptually identifies 2^64 separate fixtures within a world. As a practical matter
    ///   however, even a 4-byte index which could identify 2^32 fixtures, is still larger than
    ///   is usable. This suggests that space could be saved by using indexes into arrays of
    ///   fixtures instead of direct pointers to memory.
    Fixture* fixture;

    /// @brief Child index of related Shape.
    ChildCounter childIndex;
};

/// @brief Equality operator.
/// @relatedalso DynamicTree::LeafData
PLAYRHO_CONSTEXPR inline bool operator== (const DynamicTree::LeafData& lhs,
                                          const DynamicTree::LeafData& rhs) noexcept
{
    return lhs.fixture == rhs.fixture && lhs.childIndex == rhs.childIndex;
}

/// @brief Inequality operator.
/// @relatedalso DynamicTree::LeafData
PLAYRHO_CONSTEXPR inline bool operator!= (const DynamicTree::LeafData& lhs,
                                          const DynamicTree::LeafData& rhs) noexcept
{
    return !(lhs == rhs);
}

/// @brief Variant data.
/// @note A union is used intentionally to save space.
union DynamicTree::VariantData
{
    /// @brief Unused/free-list specific data.
    UnusedData unused;
    
    /// @brief Leaf specific data.
    LeafData leaf;
    
    /// @brief Branch specific data.
    BranchData branch;
    
    /// @brief Default constructor.
    PLAYRHO_CONSTEXPR inline VariantData() noexcept: unused{} {}

    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline VariantData(UnusedData value) noexcept: unused{value} {}
    
    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline VariantData(LeafData value) noexcept: leaf{value} {}
    
    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline VariantData(BranchData value) noexcept: branch{value} {}
};

/// @brief Is unused.
/// @details Determines whether the given dynamic tree node is an unused node.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsUnused(const DynamicTree::TreeNode& node) noexcept;

/// @brief Is leaf.
/// @details Determines whether the given dynamic tree node is a leaf node.
///   Leaf nodes have a pointer to user data.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsLeaf(const DynamicTree::TreeNode& node) noexcept;

/// @brief Is branch.
/// @details Determines whether the given dynamic tree node is a branch node.
///   Branch nodes have 2 indices to child nodes.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsBranch(const DynamicTree::TreeNode& node) noexcept;

/// @brief A node in the dynamic tree.
/// @note Users do not interact with this directly.
/// @note By using indexes to other tree nodes, these don't need to be updated
///   if the memory for other nodes is relocated.
/// @note On some 64-bit architectures, pointers are 8-bytes, while indices need only be
///   4-bytes. So using indices can also save 4-bytes.
/// @note This data structure is 48-bytes large on at least one 64-bit platform.
class DynamicTree::TreeNode
{
public:
    ~TreeNode() = default;
    
    /// @brief Copy constructor.
    PLAYRHO_CONSTEXPR inline TreeNode(const TreeNode& other) = default;

    /// @brief Move constructor.
    PLAYRHO_CONSTEXPR inline TreeNode(TreeNode&& other) = default;

    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline explicit TreeNode(Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_other{other}, m_variant{UnusedData{}}
    {
        assert(IsUnused(m_height));
    }

    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline TreeNode(const LeafData& value, AABB aabb,
                       Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_height{0}, m_other{other}, m_aabb{aabb}, m_variant{value}
    {
        assert(IsLeaf(m_height));
    }
    
    /// @brief Initializing constructor.
    PLAYRHO_CONSTEXPR inline TreeNode(const BranchData& value, AABB aabb, Height height,
                       Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_height{height}, m_other{other}, m_aabb{aabb}, m_variant{value}
    {
        assert(IsBranch(m_height));
    }
    
    /// @brief Copy assignment operator.
    TreeNode& operator= (const TreeNode& other)
    {
        m_height = other.m_height;
        m_other = other.m_other;
        m_aabb = other.m_aabb;
        switch (other.m_height)
        {
            case DynamicTree::InvalidHeight:
                m_variant.unused = other.m_variant.unused;
                break;
            case DynamicTree::Size{0}:
                m_variant.leaf = other.m_variant.leaf;
                break;
            default:
                m_variant.branch = other.m_variant.branch;
                break;
        }
        return *this;
    }
    
    /// @brief Move assignment operator.
    TreeNode& operator= (TreeNode&& other) = default;
    
    /// @brief Gets the node "height".
    PLAYRHO_CONSTEXPR inline Height GetHeight() const noexcept
    {
        return m_height;
    }
    
    /// @brief Sets the node "height" to the given value.
    PLAYRHO_CONSTEXPR inline TreeNode& SetHeight(Height value) noexcept
    {
        m_height = value;
        return *this;
    }
    
    /// @brief Gets the node's "other" index.
    PLAYRHO_CONSTEXPR inline Size GetOther() const noexcept
    {
        return m_other;
    }
                
    /// @brief Sets the node's "other" index to the given value.
    PLAYRHO_CONSTEXPR inline TreeNode& SetOther(Size other) noexcept
    {
        m_other = other;
        return *this;
    }

    /// @brief Gets the node's AABB.
    PLAYRHO_CONSTEXPR inline AABB GetAABB() const noexcept
    {
        return m_aabb;
    }

    /// @brief Sets the node's AABB.
    PLAYRHO_CONSTEXPR inline TreeNode& SetAABB(AABB value) noexcept
    {
        m_aabb = value;
        return *this;
    }
    
    /// @brief Gets the node as an "unused" value.
    PLAYRHO_CONSTEXPR inline UnusedData AsUnused() const noexcept
    {
        assert(IsUnused(m_height));
        return m_variant.unused;
    }
    
    /// @brief Gets the node as a "leaf" value.
    PLAYRHO_CONSTEXPR inline LeafData AsLeaf() const noexcept
    {
        assert(IsLeaf(m_height));
        return m_variant.leaf;
    }
    
    /// @brief Gets the node as a "branch" value.
    PLAYRHO_CONSTEXPR inline BranchData AsBranch() const noexcept
    {
        assert(IsBranch(m_height));
        return m_variant.branch;
    }

    /// @brief Gets the node as an "unused" value.
    PLAYRHO_CONSTEXPR inline UnusedData& AsUnused() noexcept
    {
        assert(IsUnused(m_height));
        return m_variant.unused;
    }
    
    /// @brief Gets the node as a "leaf" value.
    PLAYRHO_CONSTEXPR inline LeafData& AsLeaf() noexcept
    {
        assert(IsLeaf(m_height));
        return m_variant.leaf;
    }
    
    /// @brief Gets the node as a "branch" value.
    PLAYRHO_CONSTEXPR inline BranchData& AsBranch() noexcept
    {
        assert(IsBranch(m_height));
        return m_variant.branch;
    }

    /// @brief Assign's the node's value.
    PLAYRHO_CONSTEXPR inline TreeNode& Assign(Size height, const UnusedData& value) noexcept
    {
        assert(height == DynamicTree::GetInvalidHeight());
        m_height = height;
        AsUnused() = value;
        return *this;
    }
    
    /// @brief Assign's the node's value.
    PLAYRHO_CONSTEXPR inline TreeNode& Assign(Size height, const LeafData& value) noexcept
    {
        assert(height == 0);
        m_height = height;
        AsLeaf() = value;
        return *this;
    }
    
    /// @brief Assign's the node's value.
    PLAYRHO_CONSTEXPR inline TreeNode& Assign(Size height, const BranchData& value) noexcept
    {
        assert(height > 0);
        m_height = height;
        AsBranch() = value;
        return *this;
    }

private:
    /// @brief Height.
    /// @details "Height" for tree balancing.
    /// @note 0 if leaf node.
    /// @note Value of <code>DynamicTree::GetInvalidHeight()</code> if free (unallocated) node.
    Height m_height = GetInvalidHeight();

    /// @brief Other node.
    Size m_other = DynamicTree::GetInvalidSize(); ///< Index of another node.
    
    /// @brief AABB.
    AABB m_aabb;
    
    /// @brief Variant data for the node.
    VariantData m_variant;
};

PLAYRHO_CONSTEXPR inline DynamicTree::Size DynamicTree::GetDefaultInitialNodeCapacity() noexcept
{
    return Size{32};
}

inline DynamicTree::Size DynamicTree::GetRootIndex() const noexcept
{
    return m_root;
}

inline DynamicTree::Size DynamicTree::GetNodeCapacity() const noexcept
{
    return m_nodeCapacity;
}

inline DynamicTree::Size DynamicTree::GetNodeCount() const noexcept
{
    return m_nodeCount;
}

inline DynamicTree::Size DynamicTree::GetLeafCount() const noexcept
{
    return m_leafCount;
}

inline DynamicTree::Height DynamicTree::GetHeight(Size index) const noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    return m_nodes[index].GetHeight();
}

inline void DynamicTree::SetHeight(Size index, Height value) noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    m_nodes[index].SetHeight(value);
}

inline AABB DynamicTree::GetAABB(Size index) const noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(!IsUnused(m_nodes[index].GetHeight()));
    return m_nodes[index].GetAABB();
}

inline void DynamicTree::SetAABB(Size index, AABB value) noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(!IsUnused(m_nodes[index].GetHeight()));
    m_nodes[index].SetAABB(value);
}

inline DynamicTree::BranchData DynamicTree::GetBranchData(Size index) const noexcept
    {
        assert(index != GetInvalidSize());
        assert(index < m_nodeCapacity);
        assert(IsBranch(m_nodes[index].GetHeight()));
        return m_nodes[index].AsBranch();
    }

inline DynamicTree::LeafData DynamicTree::GetLeafData(Size index) const noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index].GetHeight()));
    return m_nodes[index].AsLeaf();
}

inline void DynamicTree::SetLeafData(Size index, LeafData value) noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index].GetHeight()));
    m_nodes[index].AsLeaf() = value;
}

inline DynamicTree::Size DynamicTree::GetParent(Size index) const noexcept
{
    assert(index != GetInvalidSize());
    assert(!IsUnused(m_nodes[index].GetHeight()));
    return m_nodes[index].GetOther();
}

inline void DynamicTree::SetParent(Size index, Size newParent) noexcept
{
    assert(index != GetInvalidSize());
    assert(!IsUnused(m_nodes[index].GetHeight()));
    assert(newParent == GetInvalidSize() || !IsUnused(m_nodes[newParent].GetHeight()));
    m_nodes[index].SetOther(newParent);
}

inline DynamicTree::Size DynamicTree::GetChild1(Size index) const noexcept
{
    assert(IsBranch(m_nodes[index].GetHeight()));
    return m_nodes[index].AsBranch().child1;
}

inline void DynamicTree::SetChild1(Size index, Size value) noexcept
{
    assert(IsBranch(m_nodes[index].GetHeight()));
    m_nodes[index].AsBranch().child1 = value;
}

inline DynamicTree::Size DynamicTree::GetChild2(Size index) const noexcept
{
    assert(IsBranch(m_nodes[index].GetHeight()));
    return m_nodes[index].AsBranch().child2;
}

inline void DynamicTree::SetChild2(Size index, Size value) noexcept
{
    assert(IsBranch(m_nodes[index].GetHeight()));
    m_nodes[index].AsBranch().child2 = value;
}

// Free functions...

/// @brief Whether this node is free (or allocated).
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsUnused(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsUnused(node.GetHeight());
}

/// @brief Whether or not this node is a leaf node.
/// @note This has constant complexity.
/// @return <code>true</code> if this is a leaf node, <code>false</code> otherwise.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsLeaf(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsLeaf(node.GetHeight());
}

/// @brief Is branch.
/// @details Determines whether the given node is a "branch" node.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline bool IsBranch(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsBranch(node.GetHeight());
}

/// @brief Gets the AABB of the given dynamic tree node.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline AABB GetAABB(const DynamicTree::TreeNode& node) noexcept
{
    assert(!IsUnused(node));
    return node.GetAABB();
}

/// @brief Gets the next index of the given node.
/// @warning Behavior is undefined if the given node is not an "unused" node.
/// @relatedalso DynamicTree::TreeNode
PLAYRHO_CONSTEXPR inline DynamicTree::Size GetNext(const DynamicTree::TreeNode& node) noexcept
{
    assert(IsUnused(node));
    return node.GetOther();
}

/// @brief Sets the parent index of the given node.
/// @warning Behavior is undefined if the given node is not a "branch" node.
/// @relatedalso DynamicTree::TreeNode
inline DynamicTree::TreeNode& SetParent(DynamicTree::TreeNode& node,
                                        DynamicTree::Size other) noexcept
{
    assert(IsBranch(node));
    return node.SetOther(other);
}

/// @brief Computes the height of the given dynamic tree.
inline DynamicTree::Height ComputeHeight(const DynamicTree& tree) noexcept
{
    return tree.ComputeHeight(tree.GetRootIndex());
}

/// @brief Gets the height of the binary tree.
/// @return Height of the tree (as stored in the root node) or 0 if the root node is not valid.
/// @relatedalso DynamicTree
inline DynamicTree::Height GetHeight(const DynamicTree& tree) noexcept
{
    const auto index = tree.GetRootIndex();
    return (index != DynamicTree::GetInvalidSize())? tree.GetHeight(index): DynamicTree::Height{0};
}

/// @brief Gets the AABB for the given dynamic tree.
/// @details Gets the AABB that encloses all other AABB instances that are within the
///   given dynamic tree.
/// @return Enclosing AABB or the "unset" AABB.
/// @relatedalso DynamicTree
inline AABB GetAABB(const DynamicTree& tree) noexcept
{
    const auto index = tree.GetRootIndex();
    return (index != DynamicTree::GetInvalidSize())? tree.GetAABB(index): AABB{};
}

/// @brief Tests for overlap of the elements identified in the given dynamic tree.
/// @relatedalso DynamicTree
inline bool TestOverlap(const DynamicTree& tree,
                        DynamicTree::Size leafIdA, DynamicTree::Size leafIdB) noexcept
{
    return TestOverlap(tree.GetAABB(leafIdA), tree.GetAABB(leafIdB));
}

/// @brief Gets the ratio of the sum of the perimeters of nodes to the root perimeter.
/// @note Zero is returned if no proxies exist at the time of the call.
/// @return Value of zero or more.
inline Real ComputePerimeterRatio(const DynamicTree& tree) noexcept
{
    const auto root = tree.GetRootIndex();
    if (root != DynamicTree::GetInvalidSize())
    {
        const auto rootPerimeter = GetPerimeter(tree.GetAABB(root));
        const auto total = tree.ComputeTotalPerimeter();
        return total / rootPerimeter;
    }
    return 0;
}

/// @brief Opcodes for dynamic tree callbacks.
enum class DynamicTreeOpcode
{
    End,
    Continue,
};

/// @brief Query callback type.
using DynamicTreeSizeCB = std::function<DynamicTreeOpcode(DynamicTree::Size)>;

/// @brief Query the given dynamic tree and find nodes overlapping the given AABB.
/// @note The callback instance is called for each leaf node that overlaps the supplied AABB.
void Query(const DynamicTree& tree, const AABB& aabb,
           const DynamicTreeSizeCB& callback);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_COLLISION_DYNAMICTREE_HPP
