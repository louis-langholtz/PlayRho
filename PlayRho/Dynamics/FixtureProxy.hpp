/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_FIXTUREPROXY_HPP
#define PLAYRHO_DYNAMICS_FIXTUREPROXY_HPP

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Collision/AABB.hpp>

namespace playrho {

class Fixture;

/// @brief Fixture proxy.
/// @details This proxy is used internally to connect fixtures to the broad-phase.
/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
struct FixtureProxy
{
    
    /// @brief Size type.
    using size_type = std::remove_const<decltype(MaxContacts)>::type;
    
    FixtureProxy() = delete;
    
    /// @brief Copy constructor.
    FixtureProxy(const FixtureProxy& copy) = default;

    /// @brief Move constructor.
    FixtureProxy(FixtureProxy&& copy) = default;

    /// @brief Initializing constructor.
    FixtureProxy(const AABB& bb, size_type pid, Fixture* f, ChildCounter ci):
        aabb{bb}, fixture{f}, treeId{pid}, childIndex{ci} {}
    
    ~FixtureProxy() = default;
    
    // Deleted because some fields are marked <code>const</code>.
    FixtureProxy& operator= (const FixtureProxy& other) = delete;

    // Deleted because some fields are marked <code>const</code>.
    FixtureProxy& operator= (FixtureProxy&& other) = delete;

    AABB aabb; ///< Axis Aligned Bounding Box. 16-bytes.
    
    /// @brief Fixture that this proxy is for.
    /// @note 8-bytes.
    Fixture* const fixture;

    /// @brief Tree ID.
    /// @details This is the ID of the leaf node in the dynamic tree for this "proxy".
    /// @note 4-bytes.
    const size_type treeId;
 
    /// @brief Child index of the fixture's shape that this proxy is for.
    /// @note This could potentially be calculated via pointer arithmetic - i.e.
    ///    this - array, where "this" is the address of this class and "array" is the
    ///    address of the array that this class is within. While that would shrink
    ///    this structure's size, it may also cause some fixture proxies to straddle
    ///    any 64-byte wide cache lines (which would presumably not help performance).
    /// @note 4-bytes.
    const ChildCounter childIndex;
};

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_FIXTUREPROXY_HPP
