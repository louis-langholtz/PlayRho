/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include <Box2D/Common/Settings.hpp>
#include <Box2D/Collision/DynamicTree.hpp>
#include <algorithm> // for std::sort
#include <functional>

namespace box2d {

/// Proxy ID pair.
/// @note This data structure is 8-bytes large (on at least one 64-bit platform).
struct ProxyIdPair
{
    using size_type = std::remove_const<decltype(MaxContacts)>::type;

    size_type proxyIdA;
    size_type proxyIdB;
};

constexpr inline bool operator == (ProxyIdPair lhs, ProxyIdPair rhs)
{
    return (lhs.proxyIdA == rhs.proxyIdA) && (lhs.proxyIdB == rhs.proxyIdB);
}

constexpr inline bool operator != (ProxyIdPair lhs, ProxyIdPair rhs)
{
    return !(lhs == rhs);
}

/// @brief Broad phase assistant.
/// @details
/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
/// @note This data structure is 56-bytes large (on at least one 64-bit platform).
class BroadPhase
{
public:

    using size_type = std::remove_const<decltype(MaxContacts)>::type;
    using QueryCallback = std::function<bool(size_type)>;
    using RayCastCallback = std::function<RealNum(const RayCastInput&, size_type)>;

    enum: size_type
    {
        /// Null proxy ID.
        e_nullProxy = static_cast<size_type>(-1)
    };

    struct Conf
    {
        size_type treeCapacity = 256;
        size_type moveCapacity = 16;
        size_type pairCapacity = 16;
    };

    static constexpr Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }

    BroadPhase(const Conf conf = GetDefaultConf());

    ~BroadPhase() noexcept;
    
    BroadPhase(const BroadPhase& copy) = delete;

    BroadPhase& operator=(const BroadPhase&) = delete;

    /// @brief Creates a proxy with an initial AABB.
    /// @note Pairs are not reported until UpdatePairs is called.
    size_type CreateProxy(const AABB& aabb, void* userData);

    /// @brief Destroys a proxy. It is up to the client to remove any pairs.
    void DestroyProxy(size_type proxyId);

    /// @brief Updates the proxy.
    /// @details
    /// Call this as many times as you like, then when you are done call UpdatePairs
    /// to finalize the proxy pairs (for your time step).
    /// @param proxyId Proxy ID. Behavior is undefined if this is the null proxy ID.
    /// @param aabb Axis aligned bounding box.
    /// @param displacement Displacement. Behavior is undefined if this is an invalid value.
    /// @param multiplier Multiplier to displacement amount for new AABB.
    ///   This is used to predict the future position based on the current displacement.
    ///   This is a dimensionless multiplier.
    /// @param extension Extension. Amount to extend the AABB by. This is used to fatten
    ///   AABBs in the dynamic tree.
    bool UpdateProxy(const size_type proxyId, const AABB& aabb, const Length2D displacement,
                     const RealNum multiplier = 1, const Length extension = Length{0});

    /// @brief Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
    void TouchProxy(size_type proxyId) noexcept;

    /// @brief Gets the fat AABB for a proxy.
    /// @warning Behavior is undefined if the given proxy ID is not a valid ID.
    AABB GetFatAABB(size_type proxyId) const;

    /// @brief Gets user data from a proxy.
    void* GetUserData(size_type proxyId) const;

    /// @brief Get the number of proxies.
    size_type GetProxyCount() const noexcept;

    /// @brief Updates the pairs.
    /// @details This results in pair callbacks. This can only add pairs.
    /// @param callback Callback that's called for AABB overlapping pairs.
    size_type UpdatePairs(std::function<bool(void*,void*)> callback);

    /// @brief Queries an AABB for overlapping proxies.
    /// @note The callback class is called for each proxy that overlaps the supplied AABB.
    void Query(const AABB aabb, QueryCallback callback) const;

    /// @brief Ray-cast against the proxies in the tree.
    ///
    /// @note This relies on the callback to perform an exact ray-cast in the case were the
    ///   proxy contains a shape.
    /// @note The callback also performs the any collision filtering.
    /// @note This has performance roughly equal to k * log(n), where k is the number of
    ///   collisions and n is the number of proxies in the tree.
    ///
    /// @param input the ray-cast input data.
    ///   The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback A callback instance that's called for each proxy that's hit by the ray.
    ///   The callback should return 0 to terminate raycasting, or greater than 0
    ///   to update the segment bounding box. Values less than zero are ignored.
    ///
    void RayCast(const RayCastInput& input, RayCastCallback callback) const;

    /// @brief Gets the height of the embedded tree.
    size_type GetTreeHeight() const noexcept;

    /// @brief Gets the balance of the embedded tree.
    size_type GetTreeBalance() const;

    /// @brief Gets the quality metric of the embedded tree.
    /// @return Value of zero or more.
    RealNum GetTreeQuality() const;

    /// @brief Shifts the world origin. Useful for large worlds.
    /// @note The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const Length2D newOrigin);

    /// @brief Gets the pair capacity.
    size_type GetPairCapacity() const noexcept;

    /// @brief Gets the move capacity.
    size_type GetMoveCapacity() const noexcept;
    
    /// @brief Gets the move count.
    size_type GetMoveCount() const noexcept;
    
    /// @brief Gets the pair count.
    size_type GetPairCount() const noexcept;

private:

    void EnqueueForOverlapProcessing(size_type proxyId) noexcept;
    void DequeueFromOverlapProcessing(size_type proxyId);

    DynamicTree m_tree;

    static constexpr size_type BufferGrowthRate = 2;
    
    size_type m_proxyCount = 0;

    size_type m_moveCapacity; ///< Move buffer capacity. The # of elements pointed to by move buffer. @sa m_moveBuffer.
    size_type m_moveCount = 0;

    size_type m_pairCapacity;

    // Initialized on construction
    size_type* m_moveBuffer; ///< Move buffer. @sa size_type. @sa <code>m_moveCapacity</code>. @sa <code>m_moveCount</code>.
    ProxyIdPair* m_pairBuffer;
};

inline void* BroadPhase::GetUserData(size_type proxyId) const
{
    return m_tree.GetUserData(proxyId);
}

inline AABB BroadPhase::GetFatAABB(size_type proxyId) const
{
    return m_tree.GetFatAABB(proxyId);
}

inline BroadPhase::size_type BroadPhase::GetPairCapacity() const noexcept
{
    return m_pairCapacity;
}

inline BroadPhase::size_type BroadPhase::GetMoveCapacity() const noexcept
{
    return m_moveCapacity;
}

inline BroadPhase::size_type BroadPhase::GetMoveCount() const noexcept
{
    return m_moveCount;
}

inline BroadPhase::size_type BroadPhase::GetProxyCount() const noexcept
{
    return m_proxyCount;
}

inline BroadPhase::size_type BroadPhase::GetTreeHeight() const noexcept
{
    return m_tree.GetHeight();
}

inline BroadPhase::size_type BroadPhase::GetTreeBalance() const
{
    return m_tree.GetMaxBalance();
}

inline RealNum BroadPhase::GetTreeQuality() const
{
    return m_tree.GetAreaRatio();
}

inline void BroadPhase::Query(const AABB aabb, QueryCallback callback) const
{
    m_tree.Query(aabb, callback);
}

inline void BroadPhase::RayCast(const RayCastInput& input, RayCastCallback callback) const
{
    m_tree.RayCast(input, callback);
}

inline void BroadPhase::ShiftOrigin(const Length2D newOrigin)
{
    m_tree.ShiftOrigin(newOrigin);
}

inline bool BroadPhase::UpdateProxy(const size_type proxyId,
                                    const AABB& aabb,
                                    const Length2D displacement,
                                    const RealNum multiplier,
                                    const Length extension)
{
    const auto updated = m_tree.UpdateProxy(proxyId, aabb, displacement, multiplier, extension);
    if (updated)
    {
        EnqueueForOverlapProcessing(proxyId);
    }
    return updated;
}

inline void BroadPhase::TouchProxy(size_type proxyId) noexcept
{
    EnqueueForOverlapProcessing(proxyId);
}

inline bool TestOverlap(const BroadPhase& bp,
                        BroadPhase::size_type proxyIdA, BroadPhase::size_type proxyIdB)
{
    return TestOverlap(bp.GetFatAABB(proxyIdA), bp.GetFatAABB(proxyIdB));
}

} // namespace box2d

namespace std
{
    template <>
    class hash<box2d::ProxyIdPair>
    {
    public:
        size_t operator()(const box2d::ProxyIdPair& pidpair) const
        {
            const auto a = size_t{pidpair.proxyIdA} * 2654435761u;
            const auto b = size_t{pidpair.proxyIdB} * 2654435761u;
            return a ^ b;
        }
    };
    
    template <>
    struct equal_to<box2d::ProxyIdPair>
    {
        constexpr bool operator()(const box2d::ProxyIdPair& lhs, const box2d::ProxyIdPair& rhs) const
        {
            return (lhs.proxyIdA == rhs.proxyIdA && lhs.proxyIdB == rhs.proxyIdB)
            || (lhs.proxyIdB == rhs.proxyIdA && lhs.proxyIdA == rhs.proxyIdB);
        }
    };
} // namespace std

#endif
