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

#ifndef PLAYRHO_DYNAMICS_WORLDCALLBACKS_HPP
#define PLAYRHO_DYNAMICS_WORLDCALLBACKS_HPP

#include <PlayRho/Common/Settings.hpp>
#include <algorithm>

namespace playrho {

class UnitVec2;
class Fixture;
class Joint;
class Contact;
class Manifold;

/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
class DestructionListener
{
public:
    virtual ~DestructionListener() noexcept = default;

    /// Called when any joint is about to be destroyed due
    /// to the destruction of one of its attached bodies.
    virtual void SayGoodbye(Joint& joint) = 0;

    /// Called when any fixture is about to be destroyed due
    /// to the destruction of its parent body.
    virtual void SayGoodbye(Fixture& fixture) = 0;
};

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
/// @note This data structure is 8-bytes large (on at least one 64-bit platform).
class ContactFilter
{
public:
    virtual ~ContactFilter() = default;

    /// @brief Whether contact calculations should be performed between these two shapes.
    /// @warning for performance reasons this is only called when the AABBs begin to overlap.
    /// @note If you implement your own collision filter you may want to build from the
    ///   implementation of this method.
    /// @return <code>true</code> if contact calculations should be performed between these
    ///   two shapes; <code>false</code> otherwise.
    virtual bool ShouldCollide(const Fixture* fixtureA, const Fixture* fixtureB);
};

/// Contact Impulse.
/// @details
/// Used for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in Manifold.
class ContactImpulsesList
{
public:
    
    /// @brief Counter type.
    using Counter = std::remove_const<decltype(MaxManifoldPoints)>::type;

    /// @brief Gets the count.
    Counter GetCount() const noexcept { return count; }

    /// @brief Gets the given indexed entry normal.
    Momentum GetEntryNormal(Counter index) const noexcept { return normalImpulses[index]; }

    /// @brief Gets the given indexed entry tangent.
    Momentum GetEntryTanget(Counter index) const noexcept { return tangentImpulses[index]; }
    
    /// @brief Adds an entry of the given data.
    void AddEntry(Momentum normal, Momentum tangent) noexcept
    {
        assert(count < MaxManifoldPoints);
        normalImpulses[count] = normal;
        tangentImpulses[count] = tangent;
        ++count;
    }

private:
    Momentum normalImpulses[MaxManifoldPoints]; ///< Normal impulses.
    Momentum tangentImpulses[MaxManifoldPoints]; ///< Tangent impulses.
    Counter count = 0; ///< Count of entries added.
};

/// @brief Gets the maximum normal impulse from the given contact impulses list.
inline Momentum GetMaxNormalImpulse(const ContactImpulsesList& impulses) noexcept
{
    auto maxImpulse = 0_Ns;
    const auto count = impulses.GetCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        maxImpulse = std::max(maxImpulse, impulses.GetEntryNormal(i));
    }
    return maxImpulse;
}

/// @brief A pure-virtual interface for "listeners" for contacts.
///
/// @details Implement this class to get contact information. You can use these results
///   for things like sounds and game logic. You can also get contact results by
///   traversing the contact lists after the time step. However, you might miss
///   some contacts because continuous physics leads to sub-stepping.
///   Additionally you may receive multiple callbacks for the same contact in a
///   single time step.
///   You should strive to make your callbacks efficient because there may be
///   many callbacks per time step.
///
/// @warning You cannot create/destroy PlayRho entities inside these callbacks.
///
class ContactListener
{
public:
    
    /// @brief Iteration type.
    using iteration_type = unsigned;

    virtual ~ContactListener() = default;

    /// @brief Called when two fixtures begin to touch.
    virtual void BeginContact(Contact& contact) = 0;

    /// @brief End contact callback.
    ///
    /// @details Called when the contact's "touching" property becomes false, or just before
    ///   the contact is destroyed.
    ///
    /// @note This contact persists until the broad phase determines there's no overlap anymore
    ///   between the two fixtures.
    /// @note If the contact's "touching" property becomes true again, <code>BeginContact</code>
    ///   will be called again for this contact.
    ///
    /// @param contact Contact that's about to be destroyed or whose "touching" property has become
    ///   false.
    ///
    /// @sa Contact::IsTouching().
    ///
    virtual void EndContact(Contact& contact) = 0;
    
    /// @brief Pre-solve callback.
    ///
    /// @details This is called after a contact is updated. This allows you to inspect
    ///   a contact before it goes to the solver. If you are careful, you can modify the
    ///   contact manifold (e.g. disable contact). A copy of the old manifold is provided
    ///   so that you can detect changes.
    ///
    /// @note This is called only for awake bodies.
    /// @note This is called even when the number of contact points is zero.
    /// @note This is not called for sensors.
    /// @note If you set the number of contact points to zero, you will not get an
    ///   <code>EndContact</code> callback. However, you may get a <code>BeginContact</code>
    ///   callback the next step.
    ///
    virtual void PreSolve(Contact& contact, const Manifold& oldManifold) = 0;

    /// @brief Post-solve callback.
    ///
    /// @details This lets you inspect a contact after the solver is finished. This is useful
    ///   for inspecting impulses.
    ///
    /// @note The contact manifold does not include time of impact impulses, which can be
    ///   arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    ///   in a separate data structure.
    /// @note This is only called for contacts that are touching, solid, and awake.
    ///
    virtual void PostSolve(Contact& contact, const ContactImpulsesList& impulses,
                           iteration_type solved) = 0;
};

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDCALLBACKS_HPP
