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
/// Declaration of the DynamicTree class.

#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Common/Settings.hpp>

#include <functional>
#include <type_traits>

namespace playrho {

/// @brief A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
///
/// @details A dynamic tree arranges data in a binary tree to accelerate
///   queries such as volume queries and ray casts. Leafs are proxies
///   with an AABB.
///
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

    /// @brief Leaf data type alias.
    using LeafData = void*;

    class TreeNode;
    struct UnusedData;
    struct BranchData;
    union VariantData;
    
    /// @brief Gets the invalid size value.
    static constexpr Size GetInvalidSize() noexcept
    {
        return static_cast<Size>(-1);
    }
    
    /// @brief Strong type for DynamicTree heights.
    using Height = ContactCounter;
    
    /// @brief Gets the invalid height value.
    static constexpr Height GetInvalidHeight() noexcept
    {
        return static_cast<Height>(-1);
    }
    
    /// @brief Gets whether the given height is the height for an "unused" node.
    static constexpr bool IsUnused(Height value) noexcept
    {
        return value == GetInvalidHeight();
    }
    
    /// @brief Gets whether the given height is the height for a "leaf" node.
    static constexpr bool IsLeaf(Height value) noexcept
    {
        return value == 0;
    }
    
    /// @brief Gets whether the given height is a height for a "branch" node.
    static constexpr bool IsBranch(Height value) noexcept
    {
        return !IsUnused(value) && !IsLeaf(value);
    }
    
    /// @brief Query callback type.
    using QueryCallback = std::function<bool(Size)>;
    
    /// @brief For each callback type.
    using ForEachCallback = std::function<void(Size)>;

    /// @brief Ray cast callback function.
    /// @note Return 0 to terminate raycasting, or > 0 to update the segment bounding box.
    using RayCastCallback = std::function<Real(const RayCastInput&, Size)>;

    /// @brief Gets the default initial node capacity.
    static constexpr Size GetDefaultInitialNodeCapacity() noexcept;

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
    /// @details Creates a leaf node for a tight fitting AABB and a userData pointer.
    /// @note The indices of leaf nodes that have been destroyed get reused for new nodes.
    /// @post If the root index had been the GetInvalidSize(), then it will be set to the index
    ///   returned from this method.
    /// @return Index of the created leaf node.
    Size CreateLeaf(const AABB& aabb, LeafData leafData);

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
    
    /// @brief Gets BranchData for the identified node.
    /// @warning Behavior is undefined if the given index in not a valid branch node.
    BranchData GetBranchData(Size index) const noexcept;

    /// @brief Query an AABB for overlapping proxies.
    /// @note The callback instance is called for each leaf node that overlaps the supplied AABB.
    void Query(const AABB& aabb, const QueryCallback& callback) const;

    /// @brief Calls the given callback for each of the entries overlapping the given AABB.
    void ForEach(const AABB& aabb, const ForEachCallback& callback) const;

    /// @brief Ray-cast against the proxies in the tree.
    ///
    /// @note This relies on the callback to perform an exact ray-cast in the case where the
    ///    leaf node contains a shape.
    /// @note The callback also performs collision filtering.
    /// @note Performance is roughly k * log(n), where k is the number of collisions and n is the
    ///   number of leaf nodes in the tree.
    ///
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback A callback instance function that's called for each leaf that is hit
    ///   by the ray. The callback should return 0 to terminate raycasting, or greater than 0
    ///   to update the segment bounding box. Values less than zero are ignored.
    ///
    void RayCast(const RayCastInput& input, const RayCastCallback& callback) const;

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
    /// @note The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(Length2D newOrigin);
    
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
    Size GetProxyCount() const noexcept;

    /// @brief Finds the lowest cost node.
    /// @warning Behavior is undefined if the tree doesn't have a valid root.
    Size FindLowestCostNode(AABB leafAABB) const noexcept;

private:
    
    /// @brief Sets the node capacity to the given value.
    void SetNodeCapacity(Size value) noexcept;

    /// @brief Allocates a node.
    /// @details This allocates a node from the free list as a leaf node.
    Size AllocateNode(const LeafData& node, AABB aabb) noexcept;

    /// @brief Allocates a node.
    /// @details This allocates a node from the free list as a branch node.
    /// @post The free list no longer references the returned index.
    Size AllocateNode(const BranchData& node, AABB aabb, Height height,
                      Size other = GetInvalidSize()) noexcept;
        
    Size FindReference(Size index) const noexcept;

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
    /// @post The root index is set to the given index if the root index had been GetInvalidSize().
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
    
    Size GetParent(Size index) const noexcept;
    void SetParent(Size index, Size newParent) noexcept;
    
    void SetHeight(Size index, Height value) noexcept;

    Size GetChild1(Size index) const noexcept;
    void SetChild1(Size index, Size value) noexcept;
    
    Size GetChild2(Size index) const noexcept;
    void SetChild2(Size index, Size value) noexcept;
    
    void SetAABB(Size index, AABB value) noexcept;
    
    void SwapChild(Size index, Size oldChild, Size newChild) noexcept;
    
    void SetChildren(Size index, Size child1, Size child2) noexcept;
    
    TreeNode* m_nodes; ///< Nodes. @details Initialized on construction.

    Size m_root = GetInvalidSize(); ///< Index of root element in m_nodes or GetInvalidSize().

    Size m_nodeCount = Size{0}; ///< Node count. @details Count of currently allocated nodes.
    Size m_nodeCapacity; ///< Node capacity. @details Size of buffer allocated for nodes.

    Size m_proxyCount = 0; ///< Proxy count. @details Count of currently allocated leaf nodes.

    Size m_freeListIndex = Size{0}; ///< Free list. @details Index to free nodes.
};

/// @brief Unused data of a TreeNode.
struct DynamicTree::UnusedData
{
    // Intentionally empty.
    // This exists for symetry and as placeholder in case this needs to later be used.
};

/// @brief Branch data of a TreeNode.
struct DynamicTree::BranchData
{
    Size child1; ///< @brief Child 1.
    Size child2; ///< @brief Child 2.
};

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
    constexpr VariantData() noexcept: unused{} {}

    /// @brief Initializing constructor.
    constexpr VariantData(UnusedData value) noexcept: unused{value} {}
    
    /// @brief Initializing constructor.
    constexpr VariantData(LeafData value) noexcept: leaf{value} {}
    
    /// @brief Initializing constructor.
    constexpr VariantData(BranchData value) noexcept: branch{value} {}
};

/// @brief Is unused.
/// @details Determines whether the given DynamicTree node is an unused node.
/// @relatedalso DynamicTree::TreeNode
constexpr bool IsUnused(const DynamicTree::TreeNode& node) noexcept;

/// @brief Is leaf.
/// @details Determines whether the given DynamicTree node is a leaf node.
///   Leaf nodes have a pointer to user data.
/// @relatedalso DynamicTree::TreeNode
constexpr bool IsLeaf(const DynamicTree::TreeNode& node) noexcept;

/// @brief Is branch.
/// @details Determines whether the given DynamicTree node is a branch node.
///   Branch nodes have 2 indices to child nodes.
/// @relatedalso DynamicTree::TreeNode
constexpr bool IsBranch(const DynamicTree::TreeNode& node) noexcept;

/// @brief A node in the dynamic tree.
/// @note Users do not interact with this directly.
/// @note By using indexes to other tree nodes, these don't need to be updated
///   if the memory for other nodes is relocated.
/// @note On some 64-bit architectures, pointers are 8-bytes, while indices need only be
///   4-bytes. So using indices can also save 4-bytes.
/// @note This data structure is 32-bytes large on at least one 64-bit platform.
class DynamicTree::TreeNode
{
public:
    ~TreeNode() = default;
    
    /// @brief Copy constructor.
    constexpr TreeNode(const TreeNode& other) = default;

    /// @brief Move constructor.
    constexpr TreeNode(TreeNode&& other) = default;

    /// @brief Initializing constructor.
    constexpr TreeNode(Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_other{other}, m_variant{UnusedData{}}
    {
        assert(IsUnused(m_height));
    }

    /// @brief Initializing constructor.
    constexpr TreeNode(const LeafData& value, AABB aabb,
                       Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_height{0}, m_other{other}, m_aabb{aabb}, m_variant{value}
    {
        assert(IsLeaf(m_height));
    }
    
    /// @brief Initializing constructor.
    constexpr TreeNode(const BranchData& value, AABB aabb, Height height,
                       Size other = DynamicTree::GetInvalidSize()) noexcept:
        m_height{height}, m_other{other}, m_aabb{aabb}, m_variant{value}
    {
        assert(IsBranch(m_height));
    }
    
    /// @brief Copy assignment operator.
    TreeNode& operator= (const TreeNode& other) = default;
    
    /// @brief Move assignment operator.
    TreeNode& operator= (TreeNode&& other) = default;
    
    /// @brief Gets the node "height".
    constexpr Height GetHeight() const noexcept
    {
        return m_height;
    }
    
    /// @brief Sets the node "height" to the given value.
    constexpr TreeNode& SetHeight(Height value) noexcept
    {
        m_height = value;
        return *this;
    }
    
    /// @brief Gets the node's "other" index.
    constexpr Size GetOther() const noexcept
    {
        return m_other;
    }
                
    /// @brief Sets the node's "other" index to the given value.
    constexpr TreeNode& SetOther(Size other) noexcept
    {
        m_other = other;
        return *this;
    }

    /// @brief Gets the node's AABB.
    constexpr AABB GetAABB() const noexcept
    {
        return m_aabb;
    }

    /// @brief Sets the node's AABB.
    constexpr TreeNode& SetAABB(AABB value) noexcept
    {
        m_aabb = value;
        return *this;
    }
    
    /// @brief Gets the node as an "unused" value.
    constexpr UnusedData AsUnused() const noexcept
    {
        assert(IsUnused(m_height));
        return m_variant.unused;
    }
    
    /// @brief Gets the node as a "leaf" value.
    constexpr LeafData AsLeaf() const noexcept
    {
        assert(IsLeaf(m_height));
        return m_variant.leaf;
    }
    
    /// @brief Gets the node as a "branch" value.
    constexpr BranchData AsBranch() const noexcept
    {
        assert(IsBranch(m_height));
        return m_variant.branch;
    }

    /// @brief Gets the node as an "unused" value.
    constexpr UnusedData& AsUnused() noexcept
    {
        assert(IsUnused(m_height));
        return m_variant.unused;
    }
    
    /// @brief Gets the node as a "leaf" value.
    constexpr LeafData& AsLeaf() noexcept
    {
        assert(IsLeaf(m_height));
        return m_variant.leaf;
    }
    
    /// @brief Gets the node as a "branch" value.
    constexpr BranchData& AsBranch() noexcept
    {
        assert(IsBranch(m_height));
        return m_variant.branch;
    }

    /// @brief Assign's the node's value.
    constexpr TreeNode& Assign(Size height, const UnusedData& value) noexcept
    {
        assert(height == DynamicTree::GetInvalidHeight());
        m_height = height;
        AsUnused() = value;
        return *this;
    }
    
    /// @brief Assign's the node's value.
    constexpr TreeNode& Assign(Size height, const LeafData& value) noexcept
    {
        assert(height == 0);
        m_height = height;
        AsLeaf() = value;
        return *this;
    }
    
    /// @brief Assign's the node's value.
    constexpr TreeNode& Assign(Size height, const BranchData& value) noexcept
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
    /// @note Value of DynamicTree::GetInvalidHeight() if free/un-allocated node.
    Height m_height = GetInvalidHeight();

    /// @brief Other node.
    Size m_other = DynamicTree::GetInvalidSize(); ///< Index of another node.
                
    AABB m_aabb;
                
    VariantData m_variant;
};

constexpr DynamicTree::Size DynamicTree::GetDefaultInitialNodeCapacity() noexcept
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

inline DynamicTree::Size DynamicTree::GetProxyCount() const noexcept
{
    return m_proxyCount;
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
constexpr inline bool IsUnused(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsUnused(node.GetHeight());
}

/// @brief Whether or not this node is a leaf node.
/// @note This has constant complexity.
/// @return <code>true</code> if this is a leaf node, <code>false</code> otherwise.
/// @relatedalso DynamicTree::TreeNode
constexpr inline bool IsLeaf(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsLeaf(node.GetHeight());
}

/// @brief Is branch.
/// @details Determines whether the given node is a "branch" node.
/// @relatedalso DynamicTree::TreeNode
constexpr inline bool IsBranch(const DynamicTree::TreeNode& node) noexcept
{
    return DynamicTree::IsBranch(node.GetHeight());
}

/// @brief Gets the AABB of the given DynamicTree node.
/// @relatedalso DynamicTree::TreeNode
constexpr inline AABB GetAABB(const DynamicTree::TreeNode& node) noexcept
{
    assert(!IsUnused(node));
    return node.GetAABB();
}

/// @brief Gets the next index of the given node.
/// @warning Behavior is undefined if the given node is not an "unused" node.
/// @relatedalso DynamicTree::TreeNode
constexpr inline DynamicTree::Size GetNext(const DynamicTree::TreeNode& node) noexcept
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

/// @brief Gets the AABB for the given DynamicTree.
/// @details Gets the AABB that encloses all other AABB instances that are within the
///   given DynamicTree.
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

} // namespace playrho

#endif // PLAYRHO_COLLISION_DYNAMICTREE_HPP
