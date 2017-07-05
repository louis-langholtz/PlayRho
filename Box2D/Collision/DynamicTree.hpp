/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_DYNAMIC_TREE_H
#define B2_DYNAMIC_TREE_H

/// @file
/// Declaration of the DynamicTree class.

#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/RayCastInput.hpp>

#include <functional>

namespace box2d {

/// @brief A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
///
/// @details A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB.
///
/// @note Nodes are pooled and relocatable, so we use node indices rather than pointers.
/// @note This data structure is 24-bytes large (on at least one 64-bit platform).
///
class DynamicTree
{
public:

    using size_type = std::remove_const<decltype(MaxContacts)>::type;
    using QueryCallback = std::function<bool(size_type)>;
    using ForEachCallback = std::function<void(size_type)>;

    /// @brief Ray cast callback function.
    /// @note Return 0 to terminate raycasting, or > 0 to update the segment bounding box.
    using RayCastCallback = std::function<Real(const RayCastInput&, size_type)>;

    /// @brief Invalid index value.
    static constexpr size_type InvalidIndex = static_cast<size_type>(-1);

    static constexpr size_type GetDefaultInitialNodeCapacity() noexcept;

    /// @brief Constructing the tree initializes the node pool.
    DynamicTree(const size_type nodeCapacity = GetDefaultInitialNodeCapacity());

    /// @brief Destroys the tree, freeing the node pool.
    ~DynamicTree() noexcept;

    DynamicTree(const DynamicTree& copy);

    DynamicTree& operator=(const DynamicTree& copy);

    /// @brief Creates a new proxy.
    /// @details Creates a proxy for a tight fitting AABB and a userData pointer.
    /// @note The indices of proxies that have been destroyed get reused for new proxies.
    /// @return Index of the created proxy.
    size_type CreateProxy(const AABB aabb, void* userData);

    /// @brief Destroys a proxy.
    /// @warning Behavior is undefined if the given index is not valid.
    void DestroyProxy(const size_type index);

    /// @brief Updates a proxy with a new AABB value.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Proxy ID. Behavior is undefined if this is not a valid ID.
    /// @param aabb New axis aligned bounding box of the proxy.
    void UpdateProxy(const size_type index, const AABB aabb);

    /// @brief Gets the user data for the node identified by the given identifier.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Identifier of node to get the user data for.
    /// @return User data for the specified node.
    void* GetUserData(const size_type index) const noexcept;

    void SetUserData(const size_type index, void* value) noexcept;

    /// @brief Gets the AABB for a proxy.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param index Proxy ID. Must be a valid ID.
    AABB GetAABB(const size_type index) const noexcept;

    /// @brief Query an AABB for overlapping proxies.
    /// @note The callback instance is called for each proxy that overlaps the supplied AABB.
    void Query(const AABB aabb, QueryCallback callback) const;

    void ForEach(const AABB aabb, ForEachCallback callback) const;

    /// @brief Ray-cast against the proxies in the tree.
    ///
    /// @note This relies on the callback to perform an exact ray-cast in the case were the
    ///    proxy contains a shape.
    /// @note The callback also performs the any collision filtering.
    /// @note Performance is roughly k * log(n), where k is the number of collisions and n is the
    ///   number of proxies in the tree.
    ///
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback A callback instance function that's called for each proxy that is hit
    ///   by the ray. The callback should return 0 to terminate raycasting, or greater than 0
    ///   to update the segment bounding box. Values less than zero are ignored.
    ///
    void RayCast(const RayCastInput& input, RayCastCallback callback) const;

    /// @brief Validates this tree.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool Validate() const;

    /// @brief Validates the structure of this tree from the given index.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool ValidateStructure(const size_type index) const noexcept;

    /// @brief Validates the metrics of this tree from the given index.
    /// @note Meant for testing.
    /// @return <code>true</code> if valid, <code>false</code> otherwise.
    bool ValidateMetrics(size_type index) const noexcept;

    size_type GetRootIndex() const noexcept;

    /// @brief Gets the height of the binary tree.
    /// @return Height of the tree (as stored in the root node) or 0 if the root node is not valid.
    size_type GetHeight() const noexcept;

    /// @brief Gets the maximum balance.
    /// @details This gets the maximum balance of nodes in the tree.
    /// @note The balance is the difference in height of the two children of a node.
    size_type GetMaxBalance() const;

    /// @brief Gets the ratio of the sum of the perimeters of nodes to the root perimeter.
    /// @note Zero is returned if no proxies exist at the time of the call.
    /// @return Value of zero or more.
    Real GetAreaRatio() const noexcept;

    /// @brief Builds an optimal tree.
    /// @note This operation is very expensive.
    /// @note Meant for testing.
    void RebuildBottomUp();

    /// @brief Shifts the world origin.
    /// @note Useful for large worlds.
    /// @note The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const Length2D newOrigin);

    /// @brief Computes the height of the tree from a given node.
    /// @warning Behavior is undefined if the given index is not valid.
    /// @param nodeId ID of node to compute height from.
    size_type ComputeHeight(const size_type nodeId) const noexcept;

    /// @brief Computes the height of the tree from its root.
    /// @warning Behavior is undefined if the tree doesn't have a valid root.
    size_type ComputeHeight() const noexcept;

    /// @brief Gets the current node capacity of this tree.
    size_type GetNodeCapacity() const noexcept;
    
    void SetNodeCapacity(size_type value);

    /// @brief Gets the current node count.
    /// @return Count of existing proxies (count of nodes currently allocated).
    size_type GetNodeCount() const noexcept;

    /// @brief Finds the lowest cost node.
    /// @warning Behavior is undefined if the tree doesn't have a valid root.
    size_type FindLowestCostNode(const AABB leafAABB) const noexcept;

private:

    /// A node in the dynamic tree. The client does not interact with this directly.
    struct TreeNode
    {
        /// Whether or not this node is a leaf node.
        /// @note This has constant complexity.
        /// @return <code>true</code> if this is a leaf node, <code>false</code> otherwise.
        bool IsLeaf() const noexcept
        {
            return child1 == InvalidIndex;
        }
        
        /// Enlarged AABB
        AABB aabb;
        
        void* userData;
        
        union
        {
            size_type parent;
            size_type next;
        };
        
        size_type child1; ///< Index of child 1 in DynamicTree::m_nodes or InvalidIndex.
        size_type child2; ///< Index of child 2 in DynamicTree::m_nodes or InvalidIndex.
        
        size_type height; ///< Height - for tree balancing. 0 if leaf node. InvalidIndex if free node.
    };

    /// Allocates a new node.
    size_type AllocateNode();
    
    /// Frees the specified node.
    /// @warning Behavior is undefined if the given index is not valid.
    void FreeNode(const size_type node) noexcept;

    /// Inserts the specified node.
    /// Does a leaf insertion of the node with the given index.
    /// @warning Behavior is undefined if the given index is not valid.
    void InsertLeaf(const size_type index);

    /// Removes the specified node.
    /// Does a leaf removal of the node with the given index.
    /// @warning Behavior is undefined if the given index is not valid.
    void RemoveLeaf(const size_type index);

    /// Balances the tree from the given index.
    /// @warning Behavior is undefined if the given index is not valid.
    size_type Balance(const size_type index);

    TreeNode* m_nodes; ///< Nodes. @details Initialized on construction.

    size_type m_root = InvalidIndex; ///< Index of root element in m_nodes or InvalidIndex.

    size_type m_nodeCount = 0; ///< Node count. @details Count of currently allocated nodes.
    size_type m_nodeCapacity; ///< Node capacity. @details Size of buffer allocated for nodes.

    size_type m_freeListIndex = 0; ///< Free list. @details Index to free nodes.
};

constexpr DynamicTree::size_type DynamicTree::GetDefaultInitialNodeCapacity() noexcept
{
    return size_type{16};
}

inline DynamicTree::size_type DynamicTree::GetRootIndex() const noexcept
{
    return m_root;
}

inline DynamicTree::size_type DynamicTree::GetNodeCapacity() const noexcept
{
    return m_nodeCapacity;
}

inline DynamicTree::size_type DynamicTree::GetNodeCount() const noexcept
{
    return m_nodeCount;
}

inline void* DynamicTree::GetUserData(const size_type index) const noexcept
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    return m_nodes[index].userData;
}

inline void DynamicTree::SetUserData(const size_type index, void* value) noexcept
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    m_nodes[index].userData = value;
}

inline AABB DynamicTree::GetAABB(const size_type index) const noexcept
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    return m_nodes[index].aabb;
}

inline DynamicTree::size_type DynamicTree::GetHeight() const noexcept
{
    return (m_root != InvalidIndex)? m_nodes[m_root].height: 0;
}

inline DynamicTree::size_type DynamicTree::ComputeHeight() const noexcept
{
    return ComputeHeight(GetRootIndex());
}

inline bool TestOverlap(const DynamicTree& tree,
                        DynamicTree::size_type proxyIdA, DynamicTree::size_type proxyIdB)
{
    return TestOverlap(tree.GetAABB(proxyIdA), tree.GetAABB(proxyIdB));
}

} /* namespace box2d */

#endif
