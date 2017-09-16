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

namespace playrho {

DynamicTree::DynamicTree(size_type nodeCapacity):
    m_nodeCapacity{nodeCapacity},
    m_nodes{Alloc<TreeNode>(nodeCapacity)}
{
    // Build a linked list for the free list.
    for (auto i = decltype(nodeCapacity){0}; i < nodeCapacity - 1; ++i)
    {
        m_nodes[i].next = i + 1;
        m_nodes[i].height = InvalidIndex;
    }
    m_nodes[nodeCapacity - 1].next = InvalidIndex;
    m_nodes[nodeCapacity - 1].height = InvalidIndex;
}

DynamicTree::DynamicTree(const DynamicTree& other):
    m_nodes{Alloc<TreeNode>(other.m_nodeCapacity)},
    m_root{other.m_root},
    m_nodeCount{other.m_nodeCount},
    m_nodeCapacity{other.m_nodeCapacity},
    m_freeListIndex{other.m_freeListIndex}
{
    for (auto i = decltype(other.m_nodeCapacity){0}; i < other.m_nodeCapacity; ++i)
    {
        m_nodes[i] = other.m_nodes[i];
    }
}

DynamicTree::DynamicTree(DynamicTree&& other) noexcept:
    m_nodes{other.m_nodes},
    m_root{other.m_root},
    m_nodeCount{other.m_nodeCount},
    m_nodeCapacity{other.m_nodeCapacity},
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
        m_freeListIndex = other.m_freeListIndex;
        for (auto i = decltype(other.m_nodeCapacity){0}; i < other.m_nodeCapacity; ++i)
        {
            m_nodes[i] = other.m_nodes[i];
        }
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

void DynamicTree::SetNodeCapacity(size_type value)
{
    assert(value > m_nodeCapacity);
    
    // The free list is empty. Rebuild a bigger pool.
    m_nodeCapacity = value;
    m_nodes = Realloc<TreeNode>(m_nodes, m_nodeCapacity);
    
    // Build a linked list for the free list. The parent
    // pointer becomes the "next" pointer.
    for (auto i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
    {
        m_nodes[i].next = i + 1;
        m_nodes[i].height = InvalidIndex;
    }
    m_nodes[m_nodeCapacity - 1].next = InvalidIndex;
    m_nodes[m_nodeCapacity - 1].height = InvalidIndex;
    m_freeListIndex = m_nodeCount;
}

// Allocate a node from the pool. Grow the pool if necessary.
DynamicTree::size_type DynamicTree::AllocateNode()
{
    // Expand the node pool as needed.
    if (m_freeListIndex == InvalidIndex)
    {
        assert(m_nodeCount == m_nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        SetNodeCapacity(m_nodeCapacity * 2);
    }

    // Peel a node off the free list.
    const auto index = m_freeListIndex;
    m_freeListIndex = m_nodes[index].next;
    m_nodes[index].next = InvalidIndex;
    m_nodes[index].child1 = InvalidIndex;
    m_nodes[index].child2 = InvalidIndex;
    m_nodes[index].height = 0;
    m_nodes[index].userData = nullptr;
    ++m_nodeCount;
    return index;
}

// Return a node to the pool.
void DynamicTree::FreeNode(size_type index) noexcept
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    assert(m_nodeCount > 0); // index is not ncessarily less than m_nodeCount.
    m_nodes[index].next = m_freeListIndex;
    m_nodes[index].height = InvalidIndex;
    m_freeListIndex = index;
    --m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
DynamicTree::size_type DynamicTree::CreateProxy(const AABB& aabb, void* userData)
{
    assert(IsValid(aabb));
    const auto index = AllocateNode();
    m_nodes[index].aabb = aabb;
    m_nodes[index].userData = userData;
    m_nodes[index].height = 0;
    InsertLeaf(index);
    return index;
}

void DynamicTree::DestroyProxy(size_type index)
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index]));

    RemoveLeaf(index);
    FreeNode(index);
}

void DynamicTree::UpdateProxy(size_type index, const AABB& aabb)
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);
    assert(IsLeaf(m_nodes[index]));

    RemoveLeaf(index);
    m_nodes[index].aabb = aabb;
    InsertLeaf(index);
}

DynamicTree::size_type DynamicTree::FindLowestCostNode(AABB leafAABB) const noexcept
{
    assert(m_root != InvalidIndex);
    assert(IsValid(leafAABB));

    auto index = m_root;
    while (!IsLeaf(m_nodes[index]))
    {
        const auto child1 = m_nodes[index].child1;
        const auto child2 = m_nodes[index].child2;
        const auto aabb = m_nodes[index].aabb;
        
        const auto perimeter = GetPerimeter(aabb);
        const auto combinedPerimeter = GetPerimeter(GetEnclosingAABB(aabb, leafAABB));
        
        assert(combinedPerimeter >= perimeter);
        
        // Cost of creating a new parent for this node and the new leaf
        const auto cost = combinedPerimeter * Real{2};
        
        // Minimum cost of pushing the leaf further down the tree
        const auto inheritanceCost = (combinedPerimeter - perimeter) * Real{2};
        
        assert(child1 != InvalidIndex);
        assert(child1 < m_nodeCapacity);
        
        // Cost of descending into child1
        const auto cost1 = [&]() {
            const auto enclosingAABB = GetEnclosingAABB(leafAABB, m_nodes[child1].aabb);
            const auto perimeter = GetPerimeter(enclosingAABB);
            return (IsLeaf(m_nodes[child1]))?
                perimeter + inheritanceCost:
                perimeter - GetPerimeter(m_nodes[child1].aabb) + inheritanceCost;
        }();
        
        // Cost of descending into child2
        const auto cost2 = [&]() {
            const auto enclosingAABB = GetEnclosingAABB(leafAABB, m_nodes[child2].aabb);
            const auto perimeter = GetPerimeter(enclosingAABB);
            return (IsLeaf(m_nodes[child2]))?
                perimeter + inheritanceCost:
                perimeter - GetPerimeter(m_nodes[child2].aabb) + inheritanceCost;
        }();
        
        // Descend according to the minimum cost.
        if ((cost < cost1) && (cost < cost2))
        {
            break;
        }
        
        // Descend
        index = (cost1 < cost2)? child1: child2;
    }
    return index;
}

void DynamicTree::InsertLeaf(size_type index)
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);

    if (m_root == InvalidIndex)
    {
        m_root = index;
        m_nodes[index].next = InvalidIndex;
        return;
    }

    const auto leafAABB = m_nodes[index].aabb;
    
    // Find the best sibling for this node
    const auto sibling = FindLowestCostNode(leafAABB);

    // Create a new parent.
    const auto oldParent = m_nodes[sibling].next;
    const auto newParent = AllocateNode();
    m_nodes[newParent].next = oldParent;
    m_nodes[newParent].userData = nullptr;
    m_nodes[newParent].aabb = GetEnclosingAABB(leafAABB, m_nodes[sibling].aabb);
    assert(m_nodes[sibling].height != InvalidIndex);
    m_nodes[newParent].height = m_nodes[sibling].height + 1;

    if (oldParent != InvalidIndex)
    {
        // The sibling was not the root.
        if (m_nodes[oldParent].child1 == sibling)
        {
            m_nodes[oldParent].child1 = newParent;
        }
        else
        {
            m_nodes[oldParent].child2 = newParent;
        }

        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = index;
        m_nodes[sibling].next = newParent;
        m_nodes[index].next = newParent;
    }
    else
    {
        // The sibling was the root.
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = index;
        m_nodes[sibling].next = newParent;
        m_nodes[index].next = newParent;
        m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    auto parent = m_nodes[index].next;
    while (parent != InvalidIndex)
    {
        parent = Balance(parent);

        const auto child1 = m_nodes[parent].child1;
        const auto child2 = m_nodes[parent].child2;

        assert(child1 != InvalidIndex);
        assert(child2 != InvalidIndex);

        m_nodes[parent].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
        m_nodes[parent].aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);

        parent = m_nodes[parent].next;
    }

    //Validate();
}

void DynamicTree::RemoveLeaf(size_type leaf)
{
    assert(leaf != InvalidIndex);
    assert(leaf < m_nodeCapacity);

    if (leaf == m_root)
    {
        m_root = InvalidIndex;
        return;
    }

    const auto parent = m_nodes[leaf].next;

    assert(parent < m_nodeCapacity);
    const auto grandParent = m_nodes[parent].next;
    
    const auto sibling = (m_nodes[parent].child1 == leaf)? m_nodes[parent].child2: m_nodes[parent].child1;
    assert(sibling != InvalidIndex);
    assert(sibling < m_nodeCapacity);

    if (grandParent != InvalidIndex)
    {
        // Destroy parent and connect sibling to grandParent.
        if (m_nodes[grandParent].child1 == parent)
        {
            m_nodes[grandParent].child1 = sibling;
        }
        else
        {
            m_nodes[grandParent].child2 = sibling;
        }
        m_nodes[sibling].next = grandParent;
        FreeNode(parent);

        // Adjust ancestor bounds.
        auto index = grandParent;
        while (index != InvalidIndex)
        {
            index = Balance(index);

            const auto child1 = m_nodes[index].child1;
            const auto child2 = m_nodes[index].child2;

            assert(child1 != InvalidIndex);
            assert(child1 < m_nodeCapacity);
            assert(child2 != InvalidIndex);
            assert(child2 < m_nodeCapacity);

            m_nodes[index].aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);
            assert(m_nodes[child1].height != InvalidIndex);
            assert(m_nodes[child2].height != InvalidIndex);
            m_nodes[index].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);

            index = m_nodes[index].next;
        }
    }
    else
    {
        m_root = sibling;
        m_nodes[sibling].next = InvalidIndex;
        FreeNode(parent);
    }

    //Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
DynamicTree::size_type DynamicTree::Balance(size_type index)
{
    assert(index != InvalidIndex);
    assert(index < m_nodeCapacity);

    const auto A = m_nodes + index;
    if (IsLeaf(*A) || (A->height == InvalidIndex) || (A->height < 2))
    {
        return index;
    }

    const auto iB = A->child1;
    const auto iC = A->child2;
    assert(iB != InvalidIndex);
    assert(iB < m_nodeCapacity);
    assert(iC != InvalidIndex);
    assert(iC < m_nodeCapacity);

    const auto B = m_nodes + iB;
    const auto C = m_nodes + iC;

    assert(B->height != InvalidIndex);
    assert(C->height != InvalidIndex);

    // Rotate C up
    if (C->height > (B->height + 1))
    {
        const auto iF = C->child1;
        const auto iG = C->child2;
        assert((0 <= iF) && (iF < m_nodeCapacity));
        assert((0 <= iG) && (iG < m_nodeCapacity));

        const auto F = m_nodes + iF;
        const auto G = m_nodes + iG;
        
        // Swap A and C
        C->child1 = index;
        C->next = A->next;
        A->next = iC;

        // A's old parent should point to C
        if (C->next != InvalidIndex)
        {
            if (m_nodes[C->next].child1 == index)
            {
                m_nodes[C->next].child1 = iC;
            }
            else
            {
                assert(m_nodes[C->next].child2 == index);
                m_nodes[C->next].child2 = iC;
            }
        }
        else
        {
            m_root = iC;
        }

        // Rotate
        assert(F->height != InvalidIndex);
        assert(G->height != InvalidIndex);
        if (F->height > G->height)
        {
            C->child2 = iF;
            A->child2 = iG;
            G->next = index;
            A->aabb = GetEnclosingAABB(B->aabb, G->aabb);
            C->aabb = GetEnclosingAABB(A->aabb, F->aabb);
            A->height = 1 + std::max(B->height, G->height);
            C->height = 1 + std::max(A->height, F->height);
        }
        else
        {
            C->child2 = iG;
            A->child2 = iF;
            F->next = index;
            A->aabb = GetEnclosingAABB(B->aabb, F->aabb);
            C->aabb = GetEnclosingAABB(A->aabb, G->aabb);
            A->height = 1 + std::max(B->height, F->height);
            C->height = 1 + std::max(A->height, G->height);
        }

        return iC;
    }
    
    // Rotate B up
    if (B->height > (C->height + 1))
    {
        const auto iD = B->child1;
        const auto iE = B->child2;
        const auto D = m_nodes + iD;
        const auto E = m_nodes + iE;
        assert((0 <= iD) && (iD < m_nodeCapacity));
        assert((0 <= iE) && (iE < m_nodeCapacity));

        // Swap A and B
        B->child1 = index;
        B->next = A->next;
        A->next = iB;

        // A's old parent should point to B
        if (B->next != InvalidIndex)
        {
            if (m_nodes[B->next].child1 == index)
            {
                m_nodes[B->next].child1 = iB;
            }
            else
            {
                assert(m_nodes[B->next].child2 == index);
                m_nodes[B->next].child2 = iB;
            }
        }
        else
        {
            m_root = iB;
        }

        assert(D->height != InvalidIndex);
        assert(E->height != InvalidIndex);

        // Rotate
        if (D->height > E->height)
        {
            B->child2 = iD;
            A->child1 = iE;
            E->next = index;
            A->aabb = GetEnclosingAABB(C->aabb, E->aabb);
            B->aabb = GetEnclosingAABB(A->aabb, D->aabb);
            A->height = 1 + std::max(C->height, E->height);
            B->height = 1 + std::max(A->height, D->height);
        }
        else
        {
            B->child2 = iE;
            A->child1 = iD;
            D->next = index;
            A->aabb = GetEnclosingAABB(C->aabb, D->aabb);
            B->aabb = GetEnclosingAABB(A->aabb, E->aabb);
            A->height = 1 + std::max(C->height, D->height);
            B->height = 1 + std::max(A->height, E->height);
        }

        return iB;
    }

    return index;
}

Real DynamicTree::GetAreaRatio() const noexcept
{
    if (m_root == InvalidIndex)
    {
        return Real{0};
    }

    const auto root = m_nodes + m_root;
    const auto rootArea = GetPerimeter(root->aabb);

    auto totalArea = Length{0};
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        const auto node = m_nodes + i;
        if (node->height == InvalidIndex)
        {
            // Free node in pool
            continue;
        }

        totalArea += GetPerimeter(node->aabb);
    }

    return Real{totalArea / rootArea};
}

// Compute the height of a sub-tree.
DynamicTree::size_type DynamicTree::ComputeHeight(size_type index) const noexcept
{
    assert(index < m_nodeCapacity);
    const auto node = m_nodes + index;

    if (IsLeaf(*node))
    {
        return 0;
    }

    const auto height1 = ComputeHeight(node->child1);
    const auto height2 = ComputeHeight(node->child2);
    return 1 + std::max(height1, height2);
}

void DynamicTree::ForEach(const AABB& aabb, const ForEachCallback& callback) const
{
    GrowableStack<size_type, 256> stack;
    stack.push(m_root);
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index != InvalidIndex)
        {
            const auto node = m_nodes + index;
            if (TestOverlap(node->aabb, aabb))
            {
                if (IsLeaf(*node))
                {
                    callback(index);
                }
                else
                {
                    stack.push(node->child1);
                    stack.push(node->child2);
                }
            }
        }
    }
}

void DynamicTree::Query(const AABB& aabb, const QueryCallback& callback) const
{
    GrowableStack<size_type, 256> stack;
    stack.push(m_root);
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index != InvalidIndex)
        {
            const auto node = m_nodes + index;
            if (TestOverlap(node->aabb, aabb))
            {
                if (IsLeaf(*node))
                {
                    if (!callback(index))
                    {
                        return;
                    }
                }
                else
                {
                    stack.push(node->child1);
                    stack.push(node->child2);
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
    
    GrowableStack<size_type, 256> stack;
    stack.push(m_root);
    
    while (!stack.empty())
    {
        const auto index = stack.top();
        stack.pop();
        if (index == InvalidIndex)
        {
            continue;
        }
        
        const auto node = m_nodes + index;
        if (!TestOverlap(node->aabb, segmentAABB))
        {
            continue;
        }
        
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - ctr)| > dot(|v|, extents)
        const auto center = GetCenter(node->aabb);
        const auto extents = GetExtents(node->aabb);
        const auto separation = Abs(Dot(v, p1 - center)) - Dot(abs_v, extents);
        if (separation > Length{0})
        {
            continue;
        }
        
        if (IsLeaf(*node))
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
            stack.push(node->child1);
            stack.push(node->child2);
        }
    }
}

bool DynamicTree::ValidateStructure(size_type index) const noexcept
{
    if (index == InvalidIndex)
    {
        return true;
    }

    if (index == m_root)
    {
        if (m_nodes[index].next != InvalidIndex)
        {
            return false;
        }
    }

    if (index >= m_nodeCapacity)
    {
        return false;
    }

    const auto node = m_nodes + index;

    const auto child1 = node->child1;
    const auto child2 = node->child2;

    if (IsLeaf(*node))
    {
        if (child1 != InvalidIndex)
        {
            return false;
        }
        if (child2 != InvalidIndex)
        {
            return false;
        }
        if (node->height != 0)
        {
            return false;
        }
        return true;
    }

    if (child1 >= m_nodeCapacity)
    {
        return false;
    }
    if (child2 >= m_nodeCapacity)
    {
        return false;
    }

    if (m_nodes[child1].next != index)
    {
        return false;
    }
    if (m_nodes[child2].next != index)
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

bool DynamicTree::ValidateMetrics(size_type index) const noexcept
{
    if (index == InvalidIndex)
    {
        return true;
    }

    if (index >= m_nodeCapacity)
    {
        return false;
    }

    const auto node = m_nodes + index;

    const auto child1 = node->child1;
    const auto child2 = node->child2;

    if (IsLeaf(*node))
    {
        if (child1 != InvalidIndex)
        {
            return false;
        }
        if (child2 != InvalidIndex)
        {
            return false;
        }
        if (node->height != 0)
        {
            return false;
        }
        return true;
    }

    if (child1 >= m_nodeCapacity)
    {
        return false;
    }
    if (child2 >= m_nodeCapacity)
    {
        return false;
    }

    {
        const auto height1 = m_nodes[child1].height;
        const auto height2 = m_nodes[child2].height;
        const auto height = 1 + std::max(height1, height2);
        if (node->height != height)
        {
            return false;
        }
    }
    {
        const auto aabb = GetEnclosingAABB(m_nodes[child1].aabb, m_nodes[child2].aabb);
        if (aabb.GetLowerBound() != node->aabb.GetLowerBound())
        {
            return false;
        }
        if (aabb.GetUpperBound() != node->aabb.GetUpperBound())
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

    auto freeCount = size_type{0};
    auto freeIndex = m_freeListIndex;
    while (freeIndex != InvalidIndex)
    {
        if (freeIndex >= GetNodeCapacity())
        {
            return false;
        }
        freeIndex = m_nodes[freeIndex].next;
        ++freeCount;
    }

    if ((m_root != InvalidIndex) && (GetHeight() != ComputeHeight()))
    {
        return false;
    }
    if ((GetNodeCount() + freeCount) != GetNodeCapacity())
    {
        return false;
    }
    
    return true;
}

DynamicTree::size_type DynamicTree::GetMaxBalance() const
{
    auto maxBalance = size_type{0};
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        const auto node = m_nodes + i;
        
        if (node->height == InvalidIndex)
        {
            continue;
        }

        if (node->height <= 1)
        {
            continue;
        }

        assert(!IsLeaf(*node));

        const auto child1 = node->child1;
        assert(child1 != InvalidIndex);
        assert(child1 < m_nodeCapacity);
        const auto child2 = node->child2;
        assert(child2 != InvalidIndex);
        assert(child2 < m_nodeCapacity);
        const auto height1 = m_nodes[child1].height;
        const auto height2 = m_nodes[child2].height;
        assert(height1 != InvalidIndex);
        assert(height2 != InvalidIndex);
        const auto balance = (height2 >= height1)? height2 - height1: height1 - height2;
        maxBalance = std::max(maxBalance, balance);
    }

    return maxBalance;
}

void DynamicTree::RebuildBottomUp()
{
    const auto nodes = Alloc<size_type>(m_nodeCount);
    auto count = size_type{0};

    // Build array of leaves. Free the rest.
    for (auto i = decltype(m_nodeCapacity){0}; i < m_nodeCapacity; ++i)
    {
        if (m_nodes[i].height == InvalidIndex)
        {
            // free node in pool
            continue;
        }

        if (IsLeaf(m_nodes[i]))
        {
            m_nodes[i].next = InvalidIndex;
            nodes[count] = i;
            ++count;
        }
        else
        {
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        auto minCost = Length{std::numeric_limits<Real>::infinity() * Meter};
        auto iMin = InvalidIndex;
        auto jMin = InvalidIndex;
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            const auto& aabbi = m_nodes[nodes[i]].aabb;

            for (auto j = i + 1; j < count; ++j)
            {
                const auto& aabbj = m_nodes[nodes[j]].aabb;
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
        auto child1 = m_nodes + index1;
        auto child2 = m_nodes + index2;

        const auto parentIndex = AllocateNode();
        auto parent = m_nodes + parentIndex;
        parent->child1 = index1;
        parent->child2 = index2;
        assert(child1->height != InvalidIndex);
        assert(child2->height != InvalidIndex);
        parent->height = 1 + std::max(child1->height, child2->height);
        parent->aabb = GetEnclosingAABB(child1->aabb, child2->aabb);
        parent->next = InvalidIndex;

        child1->next = parentIndex;
        child2->next = parentIndex;

        nodes[jMin] = nodes[count-1];
        nodes[iMin] = parentIndex;
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
        m_nodes[i].aabb.Move(-newOrigin);
    }
}

} // namespace playrho
