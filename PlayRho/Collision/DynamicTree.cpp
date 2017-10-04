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

#include <PlayRho/Collision/DynamicTree.hpp>
#include <PlayRho/Common/GrowableStack.hpp>

#include <cstring>
#include <algorithm>

namespace playrho {

DynamicTree::DynamicTree(Size nodeCapacity):
    m_nodeCapacity{nodeCapacity},
    m_nodes{Alloc<TreeNode>(nodeCapacity)}
{
    if (nodeCapacity == Size{0})
    {
        throw InvalidArgument("DynamicTree: node capacity must be > 0");
    }

    // Build a linked list for the free list.
    const auto endCapacity = nodeCapacity - Size{1};
    for (auto i = decltype(nodeCapacity){0}; i < endCapacity; ++i)
    {
        new (m_nodes + i) TreeNode{i + 1};
    }
    new (m_nodes + endCapacity) TreeNode{};
}

DynamicTree::DynamicTree(const DynamicTree& other):
    m_nodes{Alloc<TreeNode>(other.m_nodeCapacity)},
    m_root{other.m_root},
    m_nodeCount{other.m_nodeCount},
    m_nodeCapacity{other.m_nodeCapacity},
    m_proxyCount{other.m_proxyCount},
    m_freeListIndex{other.m_freeListIndex}
{
    std::copy(other.m_nodes, other.m_nodes + other.m_nodeCapacity, m_nodes);
}

DynamicTree::DynamicTree(DynamicTree&& other) noexcept:
    m_nodes{other.m_nodes},
    m_root{other.m_root},
    m_nodeCount{other.m_nodeCount},
    m_nodeCapacity{other.m_nodeCapacity},
    m_proxyCount{other.m_proxyCount},
    m_freeListIndex{other.m_freeListIndex}
{
    other.m_nodes = nullptr;
    other.m_nodeCapacity = 0u;
    other.m_nodeCount = 0u;
}

DynamicTree& DynamicTree::operator=(const DynamicTree& other)
{
    if (&other != this)
    {
        Free(m_nodes);
        
        m_nodes = Alloc<TreeNode>(other.m_nodeCapacity);
        m_root = other.m_root;
        m_nodeCount = other.m_nodeCount;
        m_nodeCapacity = other.m_nodeCapacity;
        m_proxyCount = other.m_proxyCount;
        m_freeListIndex = other.m_freeListIndex;
        std::copy(other.m_nodes, other.m_nodes + other.m_nodeCapacity, m_nodes);
    }
    return *this;
}

DynamicTree& DynamicTree::operator=(DynamicTree&& other) noexcept
{
    if (&other != this)
    {
        m_nodes = other.m_nodes;
        m_root = other.m_root;
        m_nodeCount = other.m_nodeCount;
        m_nodeCapacity = other.m_nodeCapacity;
        m_proxyCount = other.m_proxyCount;
        m_freeListIndex = other.m_freeListIndex;
        other.m_nodes = nullptr;
        other.m_nodeCapacity = 0u;
        other.m_nodeCount = 0u;
    }
    return *this;
}

DynamicTree::~DynamicTree() noexcept
{
    // This frees the entire tree in one shot.
    Free(m_nodes);
}

void DynamicTree::SwapChild(Size index, Size oldChild, Size newChild) noexcept
{
    if (index == GetInvalidSize())
    {
        m_root = newChild;
    }
    else if (GetChild1(index) == oldChild)
    {
        SetChild1(index, newChild);
    }
    else
    {
        assert(GetChild2(index) == oldChild);
        SetChild2(index, newChild);
    }
}

void DynamicTree::SetChildren(Size index, Size child1, Size child2) noexcept
{
    SetChild1(index, child1);
    SetChild2(index, child2);
    SetAABB(index, GetEnclosingAABB(GetAABB(child1), GetAABB(child2)));
    SetHeight(index, 1 + std::max(GetHeight(child1), GetHeight(child2)));
}

void DynamicTree::SetNodeCapacity(Size value) noexcept
{
    assert(value > m_nodeCapacity);

    // The free list is empty. Rebuild a bigger pool.
    m_nodeCapacity = value;
    m_nodes = Realloc<TreeNode>(m_nodes, m_nodeCapacity);
    
    // Build a linked list for the free list. The parent
    // pointer becomes the "next" pointer.
    const auto endCapacity = m_nodeCapacity - 1;
    for (auto i = m_nodeCount; i < endCapacity; ++i)
    {
        new (m_nodes + i) TreeNode{i + 1};
    }
    new (m_nodes + endCapacity) TreeNode{};
    m_freeListIndex = m_nodeCount;
}

// Allocate a node from the pool. Grow the pool if necessary.
DynamicTree::Size DynamicTree::AllocateNode(const LeafData& node, AABB aabb) noexcept
{
    // Expand the node pool as needed.
    if (m_freeListIndex == GetInvalidSize())
    {
        assert(m_nodeCount == m_nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        SetNodeCapacity(m_nodeCapacity * 2);
    }

    // Peel a node off the free list.
    const auto index = m_freeListIndex;
    m_freeListIndex = GetNext(m_nodes[index]);
    m_nodes[index] = TreeNode{node, aabb};
    ++m_nodeCount;
    return index;
}

DynamicTree::Size DynamicTree::AllocateNode(const BranchData& node, AABB aabb, Height height,
                                            Size other) noexcept
{
    assert(height > 0);

    // Expand the node pool as needed.
    if (m_freeListIndex == GetInvalidSize())
    {
        assert(m_nodeCount == m_nodeCapacity);
        
        // The free list is empty. Rebuild a bigger pool.
        SetNodeCapacity(m_nodeCapacity * 2);
    }
    
    // Peel a node off the free list.
    const auto index = m_freeListIndex;
    m_freeListIndex = GetNext(m_nodes[index]);
    m_nodes[index] = TreeNode{node, aabb, height, other};
    ++m_nodeCount;
    return index;
}

// Return a node to the pool.
void DynamicTree::FreeNode(Size index) noexcept
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(m_nodeCount > 0); // index is not ncessarily less than m_nodeCount.
    assert(!IsUnused(m_nodes[index].GetHeight()));
    //assert(GetParent(index) == GetInvalidSize());
    assert(index != m_freeListIndex);
    //const auto found = FindReference(index);
    //assert(found == GetInvalidSize());
    m_nodes[index] = TreeNode{m_freeListIndex};
    m_freeListIndex = index;
    --m_nodeCount;
}

DynamicTree::Size DynamicTree::FindReference(Size index) const noexcept
{
    const auto it = std::find_if(m_nodes, m_nodes + m_nodeCapacity, [&](TreeNode& node) {
        if (node.GetOther() == index)
        {
            return true;
        }
        if (IsBranch(node.GetHeight()))
        {
            if (node.AsBranch().child1 == index || node.AsBranch().child2 == index)
            {
                return true;
            }
        }
        return false;
    });
    return (it != m_nodes + m_nodeCapacity)? static_cast<Size>(it - m_nodes): GetInvalidSize();
}

DynamicTree::Size DynamicTree::CreateLeaf(const AABB& aabb, LeafData leafData)
{
    assert(IsValid(aabb));
    const auto index = AllocateNode(leafData, aabb);
    InsertLeaf(index);
    m_proxyCount++;
    return index;
}

void DynamicTree::DestroyLeaf(Size index)
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index].GetHeight()));
    assert(m_proxyCount > 0);

    m_proxyCount--;
    RemoveLeaf(index);
    FreeNode(index);
}

void DynamicTree::UpdateLeaf(Size index, const AABB& aabb)
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index].GetHeight()));

    RemoveLeaf(index);
    m_nodes[index].SetAABB(aabb);
    InsertLeaf(index);
}

DynamicTree::Size DynamicTree::FindLowestCostNode(AABB leafAABB) const noexcept
{
    assert(m_root != GetInvalidSize());
    assert(IsValid(leafAABB));

    auto index = m_root;
    while (IsBranch(m_nodes[index].GetHeight()))
    {
        const auto child1 = m_nodes[index].AsBranch().child1;
        const auto child2 = m_nodes[index].AsBranch().child2;
        const auto aabb = m_nodes[index].GetAABB();
        
        const auto perimeter = GetPerimeter(aabb);
        const auto combinedPerimeter = GetPerimeter(GetEnclosingAABB(aabb, leafAABB));
        
        assert(combinedPerimeter >= perimeter);
        
        // Cost of creating a new parent for this node and the new leaf
        const auto cost = combinedPerimeter * Real{2};
        
        // Minimum cost of pushing the leaf further down the tree
        const auto inheritanceCost = (combinedPerimeter - perimeter) * Real{2};
        
        assert(child1 != GetInvalidSize());
        assert(child1 < m_nodeCapacity);
        
        // Cost function to calculate cost of descending into specified child
        auto costFunc = [&](Size child) {
            const auto childAabb = playrho::GetAABB(m_nodes[child]);
            const auto leafCost = GetPerimeter(GetEnclosingAABB(leafAABB, childAabb))
                + inheritanceCost;
            return (IsLeaf(m_nodes[child].GetHeight()))? leafCost: leafCost - GetPerimeter(childAabb);
        };

        const auto cost1 = costFunc(child1);
        const auto cost2 = costFunc(child2);
        
        if ((cost < cost1) && (cost < cost2))
        {
            // Cheaper to create a new parent for this node and the new leaf
            break;
        }
        
        // Descend into child with least cost.
        index = (cost1 < cost2)? child1: child2;
    }
    return index;
}

void DynamicTree::InsertLeaf(Size index)
{
    assert(index != GetInvalidSize());
    assert(index < m_nodeCapacity);
    assert(!IsUnused(m_nodes[index].GetHeight()));
    assert(IsLeaf(m_nodes[index].GetHeight()));

    if (m_root == GetInvalidSize())
    {
        m_root = index;
        SetParent(index, GetInvalidSize());
        return;
    }

    const auto leafAABB = m_nodes[index].GetAABB();
    
    // Find the best sibling for this node
    const auto sibling = FindLowestCostNode(leafAABB);

    // Create a new parent.
    const auto oldParent = GetParent(sibling);

    // Warning: the following may change value of m_nodes!
    // std::max of leaf height and sibling height + 1 = sibling height + 1
    const auto newParent = AllocateNode(BranchData{sibling, index},
                                        GetEnclosingAABB(leafAABB, GetAABB(sibling)),
                                        1 + GetHeight(sibling),
                                        oldParent);
    if (oldParent != GetInvalidSize())
    {
        // The sibling was not the root.
        SwapChild(oldParent, sibling, newParent);
    }
    else
    {
        // The sibling was the root.
        m_root = newParent;
    }
    SetParent(sibling, newParent);
    SetParent(index, newParent);

    // Walk back up the tree fixing heights and AABBs
    Rebalance(newParent);
}

void DynamicTree::RemoveLeaf(Size leaf)
{
    assert(leaf != GetInvalidSize());
    assert(leaf < m_nodeCapacity);
    
    if (leaf == m_root)
    {
        assert(GetParent(leaf) == GetInvalidSize());
        m_root = GetInvalidSize();
        return;
    }

    const auto parent = GetParent(leaf);
    const auto grandParent = GetParent(parent);
    const auto sibling = (GetChild1(parent) == leaf)? GetChild2(parent): GetChild1(parent);

    if (grandParent != GetInvalidSize())
    {
        // Destroy parent and connect sibling to grandParent.
        SwapChild(grandParent, parent, sibling);
        SetParent(sibling, grandParent);
        SetParent(parent, GetInvalidSize());

        // Adjust ancestor bounds.
        Rebalance(grandParent);
        SetParent(leaf, GetInvalidSize());
    }
    else
    {
        // grandParent == GetInvalidSize()
        SetParent(sibling, GetInvalidSize());
        m_root = sibling;
        SetParent(leaf, GetInvalidSize());
    }
    FreeNode(parent);
}

void DynamicTree::Rebalance(Size index)
{
    for (; index != GetInvalidSize(); index = GetParent(index))
    {
        assert(index != GetInvalidSize());
        assert(index < m_nodeCapacity);
        assert(!IsUnused(m_nodes[index].GetHeight()));
        
        do
        {
            if (GetHeight(index) < 2)
            {
                break;
            }
            
            //          o
            //          |
            //          i
            //         / \
            //      *-c1  c2-*
            //     /   |  |   \
            // c1c1 c1c2  c2c1 c2c2
            
            const auto parent = GetParent(index); // o
            const auto child1 = GetChild1(index); // c1
            const auto child2 = GetChild2(index); // c2
            
            if (GetHeight(child2) > (GetHeight(child1) + 1))
            {
                // child2 must be a branch, rotate it up.
                // change index's parent and child 2
                // Swap index and child2
                const auto child2child1 = GetChild1(child2);
                const auto child2child2 = GetChild2(child2);
                
                // index's old parent should point to child2
                SwapChild(parent, index, child2);
                
                // Rotate
                if (GetHeight(child2child1) > GetHeight(child2child2))
                {
                    SetParent(child2child2, index);
                    SetParent(index, child2);
                    SetChildren(index, child1, child2child2);
                    SetParent(child2, parent);
                    SetChildren(child2, index, child2child1);
                }
                else
                {
                    SetParent(child2child1, index);
                    SetParent(index, child2);
                    SetChildren(index, child1, child2child1);
                    SetParent(child2, parent);
                    SetChildren(child2, index, child2child2);
                }
                index = child2;
                break;
            }
            
            if (GetHeight(child1) > (GetHeight(child2) + 1))
            {
                // child1 must be a branch, rotate it up.
                // change index's parent and child 1
                // Swap index and child1
                const auto child1child1 = GetChild1(child1);
                const auto child1child2 = GetChild2(child1);
                
                // index's old parent should point to child1
                SwapChild(parent, index, child1);
                
                // Rotate
                if (GetHeight(child1child1) > GetHeight(child1child2))
                {
                    SetParent(child1child2, index);
                    SetParent(index, child1);
                    SetChildren(index, child1child2, child2);
                    SetParent(child1, parent);
                    SetChildren(child1, index, child1child1);
                }
                else
                {
                    SetParent(child1child1, index);
                    SetParent(index, child1);
                    SetChildren(index, child1child1, child2);
                    SetParent(child1, parent);
                    SetChildren(child1, index, child1child2);
                }
                index = child1;
                break;
            }
        }
        while (false);
        
        const auto child1 = GetChild1(index);
        const auto child2 = GetChild2(index);
        SetHeight(index, 1 + std::max(GetHeight(child1), GetHeight(child2)));
        SetAABB(index, GetEnclosingAABB(GetAABB(child1), GetAABB(child2)));
    }
}

Length DynamicTree::ComputeTotalPerimeter() const noexcept
{
    auto totalPerimeter = Length{0};
    if (m_root != GetInvalidSize())
    {
        for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
        {
            if (!IsUnused(m_nodes[i].GetHeight()))
            {
                totalPerimeter += GetPerimeter(m_nodes[i].GetAABB());
            }
        }
    }
    return totalPerimeter;
}

// Compute the height of a sub-tree.
DynamicTree::Height DynamicTree::ComputeHeight(Size index) const noexcept
{
    assert(index < m_nodeCapacity);
    if (IsBranch(m_nodes[index].GetHeight()))
    {
        const auto height1 = ComputeHeight(m_nodes[index].AsBranch().child1);
        const auto height2 = ComputeHeight(m_nodes[index].AsBranch().child2);
        return 1 + std::max(height1, height2);
    }
    return 0;
}

void DynamicTree::ForEach(const AABB& aabb, const ForEachCallback& callback) const
{
    GrowableStack<Size, 256> stack;
    stack.push(m_root);
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index != GetInvalidSize())
        {
            if (TestOverlap(m_nodes[index].GetAABB(), aabb))
            {
                if (IsLeaf(m_nodes[index].GetHeight()))
                {
                    callback(index);
                }
                else
                {
                    stack.push(m_nodes[index].AsBranch().child1);
                    stack.push(m_nodes[index].AsBranch().child2);
                }
            }
        }
    }
}

void DynamicTree::RayCast(const RayCastInput& input, const RayCastCallback& callback) const
{
    const auto p1 = input.p1;
    const auto p2 = input.p2;
    const auto delta = p2 - p1;
    
    // v is perpendicular to the segment.
    const auto v = GetRevPerpendicular(GetUnitVector(delta, UnitVec2::GetZero()));
    const auto abs_v = Abs(v);
    
    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)
    
    auto maxFraction = input.maxFraction;
    
    // Build a bounding box for the segment.
    auto segmentAABB = AABB{p1, p1 + maxFraction * delta};
    
    GrowableStack<Size, 256> stack;
    stack.push(m_root);
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index == GetInvalidSize())
        {
            continue;
        }
        
        if (!TestOverlap(m_nodes[index].GetAABB(), segmentAABB))
        {
            continue;
        }
        
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - ctr)| > dot(|v|, extents)
        const auto center = GetCenter(m_nodes[index].GetAABB());
        const auto extents = GetExtents(m_nodes[index].GetAABB());
        const auto separation = Abs(Dot(v, p1 - center)) - Dot(abs_v, extents);
        if (separation > Length{0})
        {
            continue;
        }
        
        if (IsLeaf(m_nodes[index].GetHeight()))
        {
            const auto subInput = RayCastInput{input.p1, input.p2, maxFraction};
            
            const auto value = callback(subInput, index);
            if (value == 0)
            {
                // The client has terminated the ray cast.
                return;
            }
            
            if (value > 0)
            {
                // Update segment bounding box.
                maxFraction = value;
                const auto t = p1 + maxFraction * (p2 - p1);
                segmentAABB = AABB{p1, t};
            }
        }
        else
        {
            stack.push(m_nodes[index].AsBranch().child1);
            stack.push(m_nodes[index].AsBranch().child2);
        }
    }
}

bool DynamicTree::ValidateStructure(Size index) const noexcept
{
    if (index == GetInvalidSize())
    {
        return true;
    }

    if (index == m_root)
    {
        if (GetParent(index) != GetInvalidSize())
        {
            return false;
        }
    }

    if (index >= m_nodeCapacity)
    {
        return false;
    }
    
    if (IsLeaf(m_nodes[index].GetHeight()))
    {
        return true;
    }
    
    const auto child1 = m_nodes[index].AsBranch().child1;
    const auto child2 = m_nodes[index].AsBranch().child2;

    if (child1 >= m_nodeCapacity)
    {
        return false;
    }
    if (child2 >= m_nodeCapacity)
    {
        return false;
    }

    if (m_nodes[child1].GetOther() != index)
    {
        return false;
    }
    if (m_nodes[child2].GetOther() != index)
    {
        return false;
    }

    if (!ValidateStructure(child1))
    {
        return false;
    }
    if (!ValidateStructure(child2))
    {
        return false;
    }
    
    return true;
}

bool DynamicTree::ValidateMetrics(Size index) const noexcept
{
    if (index == GetInvalidSize())
    {
        return true;
    }

    if (index >= m_nodeCapacity)
    {
        return false;
    }

    if (!IsBranch(m_nodes[index].GetHeight()))
    {
        return true;
    }
    
    const auto child1 = m_nodes[index].AsBranch().child1;
    const auto child2 = m_nodes[index].AsBranch().child2;

    if (child1 >= m_nodeCapacity)
    {
        return false;
    }
    if (child2 >= m_nodeCapacity)
    {
        return false;
    }

    const auto childNode1 = m_nodes + child1;
    const auto childNode2 = m_nodes + child2;

    {
        const auto height1 = childNode1->GetHeight();
        const auto height2 = childNode2->GetHeight();
        const auto height = 1 + std::max(height1, height2);
        if (m_nodes[index].GetHeight() != height)
        {
            return false;
        }
    }
    {
        const auto aabb = GetEnclosingAABB(childNode1->GetAABB(), childNode2->GetAABB());
        if (aabb != m_nodes[index].GetAABB())
        {
            return false;
        }
    }

    if (!ValidateMetrics(child1))
    {
        return false;
    }
    if (!ValidateMetrics(child2))
    {
        return false;
    }
    
    return true;
}

bool DynamicTree::Validate() const
{
    if (!ValidateStructure(m_root))
    {
        return false;
    }

    if (!ValidateMetrics(m_root))
    {
        return false;
    }

    auto freeCount = Size{0};
    auto freeIndex = m_freeListIndex;
    while (freeIndex != GetInvalidSize())
    {
        if (freeIndex >= GetNodeCapacity())
        {
            return false;
        }
        freeIndex = (m_nodes + freeIndex)->GetOther();
        ++freeCount;
    }

    if ((m_root != GetInvalidSize()) && (playrho::GetHeight(*this) != playrho::ComputeHeight(*this)))
    {
        return false;
    }
    if ((GetNodeCount() + freeCount) != GetNodeCapacity())
    {
        return false;
    }
    
    return true;
}

DynamicTree::Height DynamicTree::GetMaxBalance() const noexcept
{
    auto maxBalance = Height{0};
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        if (!IsBranch(m_nodes[i].GetHeight()))
        {
            continue;
        }

        const auto child1 = m_nodes[i].AsBranch().child1;
        assert(child1 != GetInvalidSize());
        assert(child1 < m_nodeCapacity);
        const auto child2 = m_nodes[i].AsBranch().child2;
        assert(child2 != GetInvalidSize());
        assert(child2 < m_nodeCapacity);
        const auto height1 = (m_nodes + child1)->GetHeight();
        const auto height2 = (m_nodes + child2)->GetHeight();
        assert(height1 != GetInvalidHeight());
        assert(height2 != GetInvalidHeight());
        const auto balance = (height2 >= height1)? height2 - height1: height1 - height2;
        maxBalance = std::max(maxBalance, balance);
    }

    return maxBalance;
}

void DynamicTree::RebuildBottomUp()
{
    const auto nodes = Alloc<Size>(m_nodeCount);
    auto count = Size{0};

    // Build array of leaves. Free the rest.
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        const auto height = m_nodes[i].GetHeight();
        if (IsLeaf(height))
        {
            m_nodes[i].SetOther(GetInvalidSize());
            nodes[count] = i;
            ++count;
        }
        else if (IsBranch(height))
        {
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        auto minCost = std::numeric_limits<Length>::infinity();
        auto iMin = GetInvalidSize();
        auto jMin = GetInvalidSize();
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            const auto& aabbi = m_nodes[nodes[i]].GetAABB();

            for (auto j = i + 1; j < count; ++j)
            {
                const auto& aabbj = m_nodes[nodes[j]].GetAABB();
                const auto b = GetEnclosingAABB(aabbi, aabbj);
                const auto cost = GetPerimeter(b);
                if (minCost > cost)
                {
                    iMin = i;
                    jMin = j;
                    minCost = cost;
                }
            }
        }

        assert((iMin < m_nodeCount) && (jMin < m_nodeCount));

        const auto index1 = nodes[iMin];
        const auto index2 = nodes[jMin];
        assert(!IsUnused(m_nodes[index1].GetHeight()));
        assert(!IsUnused(m_nodes[index2].GetHeight()));

        const auto aabb = GetEnclosingAABB(m_nodes[index1].GetAABB(), m_nodes[index2].GetAABB());
        const auto height = 1 + std::max(m_nodes[index1].GetHeight(), m_nodes[index2].GetHeight());
        
        // Warning: the following may change value of m_nodes!
        const auto parent = AllocateNode(BranchData{index1, index2}, aabb, height);
        m_nodes[index1].SetOther(parent);
        m_nodes[index2].SetOther(parent);

        nodes[jMin] = nodes[count-1];
        nodes[iMin] = parent;
        --count;
    }

    m_root = nodes[0];
    Free(nodes);

    Validate();
}

void DynamicTree::ShiftOrigin(Length2D newOrigin)
{
    // Build array of leaves. Free the rest.
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        m_nodes[i].SetAABB(GetMovedAABB(m_nodes[i].GetAABB(), -newOrigin));
    }
}

// Free functions...

void Query(const DynamicTree& tree, const AABB& aabb, const DynamicTree::QueryCallback& callback)
{
    GrowableStack<DynamicTree::Size, 256> stack;
    stack.push(tree.GetRootIndex());
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index != DynamicTree::GetInvalidSize())
        {
            if (TestOverlap(tree.GetAABB(index), aabb))
            {
                const auto height = tree.GetHeight(index);
                if (DynamicTree::IsBranch(height))
                {
                    const auto branchData = tree.GetBranchData(index);
                    stack.push(branchData.child1);
                    stack.push(branchData.child2);
                }
                else
                {
                    assert(IsLeaf(height));
                    if (!callback(index))
                    {
                        return;
                    }
                }
            }
        }
    }
}

} // namespace playrho
