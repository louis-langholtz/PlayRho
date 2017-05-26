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

#ifndef B2_WORLD_CALLBACKS_H
#define B2_WORLD_CALLBACKS_H

#include <Box2D/Common/Vec2.hpp>

namespace box2d {

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
    virtual ~DestructionListener() noexcept {}

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
    virtual ~ContactFilter() {}

    /// Return true if contact calculations should be performed between these two shapes.
    /// @warning for performance reasons this is only called when the AABBs begin to overlap.
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
    using count_t = std::remove_const<decltype(MaxManifoldPoints)>::type;

    count_t GetCount() const noexcept { return count; }

    Momentum GetEntryNormal(count_t index) const noexcept { return normalImpulses[index]; }
    Momentum GetEntryTanget(count_t index) const noexcept { return tangentImpulses[index]; }
    
    void AddEntry(Momentum normal, Momentum tangent) noexcept
    {
        assert(count < MaxManifoldPoints);
        normalImpulses[count] = normal;
        tangentImpulses[count] = tangent;
        ++count;
    }

private:
    Momentum normalImpulses[MaxManifoldPoints];
    Momentum tangentImpulses[MaxManifoldPoints];
    count_t count = 0;
};

/// An interface for "listeners" for contacts.
///
/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
///
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
///
class ContactListener
{
public:
    using iteration_type = unsigned;

    virtual ~ContactListener() {}

    /// Called when two fixtures begin to touch.
    virtual void BeginContact(Contact& contact) { NOT_USED(contact); }

    /// End contact callback.
    /// @details
    /// Called when the contact's "touching" property becomes false, or just before the contact
    /// is destroyed.
    /// @note This contact persists until the broadphase determines there's no overlap anymore
    ///   between the two fixtures.
    /// @note If the contact's "touching" property becomes true again, BeginContact will be called
    ///   again for this contact.
    /// @sa Contact::IsTouching().
    /// @param contact Contact that's about to be destroyed or whose "touching" property has become
    ///   false.
    virtual void EndContact(Contact& contact) { NOT_USED(contact); }

    /// Pre-solve callback.
    /// @details
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. disable contact).
    /// A copy of the old manifold is provided so that you can detect changes.
    /// @note This is called only for awake bodies.
    /// @note This is called even when the number of contact points is zero.
    /// @note This is not called for sensors.
    /// @note If you set the number of contact points to zero, you will not
    /// get an EndContact callback. However, you may get a BeginContact callback
    /// the next step.
    virtual void PreSolve(Contact& contact, const Manifold& oldManifold)
    {
        NOT_USED(contact);
        NOT_USED(oldManifold);
    }

    /// Post-solve callback.
    /// @details
    /// This lets you inspect a contact after the solver is finished. This is useful
    /// for inspecting impulses.
    /// @note The contact manifold does not include time of impact impulses, which can be
    /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    /// in a separate data structure.
    /// @note This is only called for contacts that are touching, solid, and awake.
    virtual void PostSolve(Contact& contact, const ContactImpulsesList& impulse, iteration_type solved)
    {
        NOT_USED(contact);
        NOT_USED(impulse);
        NOT_USED(solved);
    }
};

/// Callback class for AABB queries.
/// See World::Query
class QueryFixtureReporter
{
public:
    virtual ~QueryFixtureReporter() {}

    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    virtual bool ReportFixture(Fixture* fixture) = 0;
};

/// Callback class for ray casts.
/// See World::RayCast
class RayCastFixtureReporter
{
public:
    enum class Opcode {
        Terminate,
        IgnoreFixture,
        ClipRay,
        ResetRay
    };

    virtual ~RayCastFixtureReporter() {}

    /// Reports fixture.
    ///
    /// @details Called for each fixture found in the query. You control how the ray cast
    /// proceeds by the return value:
    /// return -1: ignore this fixture and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    ///
    /// @param fixture the fixture hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection.
    ///
    /// @return Operational code.
    ///
    virtual Opcode ReportFixture(Fixture* fixture, const Length2D& point,
                                 const UnitVec2& normal) = 0;
};

} // namespace box2d

#endif
