/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_CONTACT_H
#define B2_CONTACT_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/TimeOfImpact.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>

namespace box2d {

class Body;
class Fixture;
class ContactListener;
struct ToiConf;
class StepConf;

/// @brief Mixes friction.
/// @details Friction mixing formula. The idea is to allow either fixture to drive the
///   resulting friction to zero. For example, anything slides on ice.
///
/// @warning Behavior is undefined if either friction values is less than zero.
///
/// @param friction1 A zero or greater value.
/// @param friction2 A zero or greater value.
///
inline Real MixFriction(Real friction1, Real friction2)
{
    assert(friction1 >= Real(0) && friction2 >= Real(0));
    return Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline Real MixRestitution(Real restitution1, Real restitution2) noexcept
{
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

/// A potential contact between the chidren of two Fixture objects.
/// @details
/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
/// @note This data structure is 104-bytes large (on at least one 64-bit platform).
class Contact
{
public:
    using substep_type = ts_iters_t;

    struct UpdateConf
    {
        DistanceConf distance;
        Manifold::Conf manifold;
    };
    
    static UpdateConf GetUpdateConf(const StepConf& conf) noexcept;

    /// @brief Initializing constructor.
    ///
    /// @param fixtureA Fixture A. A non-null pointer to fixture A that must have a shape
    ///   and may not be the same fixture or have the same body as the other fixture.
    /// @param indexA Index of child A (from fixture A).
    /// @param fixtureB Fixture B. A non-null pointer to fixture B that must have a shape
    ///   and may not be the same fixture or have the same body as the other fixture.
    /// @param indexB Index of child B (from fixture B).
    ///
    /// @warning Behavior is undefined if <code>fixtureA</code> is null.
    /// @warning Behavior is undefined if <code>fixtureB</code> is null.
    /// @warning Behavior is undefined if <code>fixtureA == fixtureB</code>.
    /// @warning Behavior is undefined if <code>fixtureA</code> has no associated shape.
    /// @warning Behavior is undefined if <code>fixtureB</code> has no associated shape.
    /// @warning Behavior is undefined if both fixtures have the same body.
    ///
    Contact(Fixture* fixtureA, ChildCounter indexA, Fixture* fixtureB, ChildCounter indexB);
    
    Contact() = delete;
    
    Contact(const Contact& copy) = default;
    
    /// Gets the contact manifold.
    const Manifold& GetManifold() const noexcept;

    /// Is this contact touching?
    /// @details
    /// Touching is defined as either:
    ///   1. This contact's manifold has more than 0 contact points, or
    ///   2. This contact has sensors and the two shapes of this contact are found to be
    ///      overlapping.
    /// @return true if this contact is said to be touching, false otherwise.
    bool IsTouching() const noexcept;

    /// Enable/disable this contact. This can be used inside the pre-solve
    /// contact listener. The contact is only disabled for the current
    /// time step (or sub-step in continuous collisions).
    [[deprecated]] void SetEnabled(bool flag) noexcept;

    /// Enables this contact.
    void SetEnabled() noexcept;

    /// Disables this contact.
    void UnsetEnabled() noexcept;

    /// Has this contact been disabled?
    bool IsEnabled() const noexcept;

    /// Gets fixture A in this contact.
    Fixture* GetFixtureA() const noexcept;

    /// Get the child primitive index for fixture A.
    ChildCounter GetChildIndexA() const noexcept;

    /// Gets fixture B in this contact.
    Fixture* GetFixtureB() const noexcept;

    /// Get the child primitive index for fixture B.
    ChildCounter GetChildIndexB() const noexcept;

    /// @brief Sets the friction value for this contact.
    /// @details Override the default friction mixture.
    /// @note You can call this in ContactListener::PreSolve.
    /// @note This value persists until set or reset.
    /// @warning Behavior is undefined if given a negative friction value.
    /// @param friction Co-efficient of friction value of zero or greater.
    void SetFriction(Real friction) noexcept;

    /// @brief Gets the coefficient of friction.
    /// @details Gets the combined friction of the two fixtures associated with this contact.
    /// @return Value of 0 or higher.
    /// @sa MixFriction.
    Real GetFriction() const noexcept;

    /// Override the default restitution mixture. You can call this in ContactListener::PreSolve.
    /// The value persists until you set or reset.
    void SetRestitution(Real restitution) noexcept;

    /// Get the restitution.
    Real GetRestitution() const noexcept;

    /// Set the desired tangent speed for a conveyor belt behavior.
    void SetTangentSpeed(LinearVelocity speed) noexcept;

    /// Gets the desired tangent speed.
    LinearVelocity GetTangentSpeed() const noexcept;

    substep_type GetToiCount() const noexcept;

    /// Gets whether a TOI is set.
    /// @return true if this object has a TOI set for it, false otherwise.
    bool HasValidToi() const noexcept;

    /// Gets the time of impact (TOI) as a fraction.
    /// @note This is only valid if a TOI has been set.
    /// @sa void SetToi(Real toi).
    /// @return Time of impact fraction in the range of 0 to 1 if set (where 1
    ///   means no actual impact in current time slot), otheriwse undefined.
    Real GetToi() const;

    void FlagForFiltering() noexcept;
    bool NeedsFiltering() const noexcept;

    void FlagForUpdating() noexcept;
    bool NeedsUpdating() const noexcept;

private:

    friend class ContactAtty;

    /// Flags type data type.
    using FlagsType = std::uint8_t;

    // Flags stored in m_flags
    enum: FlagsType
    {
        // Set when the shapes are touching.
        e_touchingFlag = 0x0001,

        // This contact can be disabled (by user)
        e_enabledFlag = 0x0002,

        // This contact needs filtering because a fixture filter was changed.
        e_filterFlag = 0x0004,

        // This contact has a valid TOI in m_toi
        e_toiFlag = 0x0008,
        
        // This contacts needs its touching state updated.
        e_dirtyFlag = 0x0010
    };
    
    /// Flag this contact for filtering. Filtering will occur the next time step.
    void UnflagForFiltering() noexcept;

    void UnflagForUpdating() noexcept;

    /// @brief Updates the touching related state and notifies listener (if one given).
    ///
    /// @note Ideally this method is only called when a dependent change has occurred.
    /// @note Touching related state depends on the following data:
    ///   - The fixtures' sensor states.
    ///   - The fixtures bodies' transformations.
    ///   - The "maxCirclesRatio" per-step configuration state *OR* the "maxDistanceIters"
    ///     per-step configuration state.
    ///
    /// @param conf Per-step configuration information.
    /// @param listener Listener that if non-null is called with status information.
    ///
    /// @sa GetManifold.
    /// @sa IsTouching.
    ///
    void Update(const UpdateConf& conf, ContactListener* listener = nullptr);

    /// Sets the time of impact (TOI).
    /// @details After returning, this object will have a TOI that is set as indicated by <code>HasValidToi()</code>.
    /// @note Behavior is undefined if the value assigned is less than 0 or greater than 1.
    /// @sa Real GetToi() const.
    /// @sa HasValidToi.
    /// @param toi Time of impact as a fraction between 0 and 1 where 1 indicates no actual impact in the current time slot.
    void SetToi(Real toi) noexcept;

    void UnsetToi() noexcept;

    void SetToiCount(substep_type value) noexcept;

    /// Sets the touching flag state.
    /// @note This should only be called if either:
    ///   1. The contact's manifold has more than 0 contact points, or
    ///   2. The contact has sensors and the two shapes of this contact are found to be overlapping.
    /// @sa IsTouching().
    void SetTouching() noexcept;

    void UnsetTouching() noexcept;

    /// @brief Gets the writable manifold.
    /// @note This is intentionally not a public method.
    /// @warning Do not modify the manifold unless you understand the internals of Box2D.
    Manifold& GetMutableManifold() noexcept;
    
    // Member variables...

    Fixture* const m_fixtureA; ///< Fixture A. @details Non-null pointer to fixture A.
    Fixture* const m_fixtureB; ///< Fixture B. @details Non-null pointer to fixture B.

    ChildCounter const m_indexA;
    ChildCounter const m_indexB;

    Manifold mutable m_manifold; ///< Manifold of the contact. 60-bytes. @sa Update.

    substep_type m_toiCount = 0; ///< Count of TOI calculations contact has gone through since last reset.

    FlagsType m_flags = e_enabledFlag|e_dirtyFlag;

    LinearVelocity m_tangentSpeed = 0;
    
    /// Time of impact.
    /// @note This is a unit interval of time (a value between 0 and 1).
    /// @note Only valid if m_flags & e_toiFlag
    Real m_toi;

    // initialized on construction (construction-time depedent)
    Real m_friction; ///< Mix of frictions of the associated fixtures. @sa MixFriction.
    Real m_restitution; ///< Mix of restitutions of the associated fixtures. @sa MixRestitution.
};

inline const Manifold& Contact::GetManifold() const noexcept
{
    // XXX: What to do if needs-updating?
    //assert(!NeedsUpdating());
    return m_manifold;
}

inline Manifold& Contact::GetMutableManifold() noexcept
{
    return m_manifold;
}

inline void Contact::SetEnabled(bool flag) noexcept
{
    if (flag)
        SetEnabled();
    else
        UnsetEnabled();
}

inline void Contact::SetEnabled() noexcept
{
    m_flags |= Contact::e_enabledFlag;
}

inline void Contact::UnsetEnabled() noexcept
{
    m_flags &= ~Contact::e_enabledFlag;
}

inline bool Contact::IsEnabled() const noexcept
{
    return (m_flags & e_enabledFlag) != 0;
}

inline bool Contact::IsTouching() const noexcept
{
    // XXX: What to do if needs-updating?
    // assert(!NeedsUpdating());
    return (m_flags & e_touchingFlag) != 0;
}

inline void Contact::SetTouching() noexcept
{
    m_flags |= e_touchingFlag;
}

inline void Contact::UnsetTouching() noexcept
{
    m_flags &= ~e_touchingFlag;
}

inline Fixture* Contact::GetFixtureA() const noexcept
{
    return m_fixtureA;
}

inline ChildCounter Contact::GetChildIndexA() const noexcept
{
    return m_indexA;
}

inline Fixture* Contact::GetFixtureB() const noexcept
{
    return m_fixtureB;
}

inline ChildCounter Contact::GetChildIndexB() const noexcept
{
    return m_indexB;
}

inline void Contact::FlagForFiltering() noexcept
{
    m_flags |= e_filterFlag;
}

inline void Contact::UnflagForFiltering() noexcept
{
    m_flags &= ~Contact::e_filterFlag;
}

inline bool Contact::NeedsFiltering() const noexcept
{
    return m_flags & Contact::e_filterFlag;
}

inline void Contact::FlagForUpdating() noexcept
{
    m_flags |= e_dirtyFlag;
}

inline void Contact::UnflagForUpdating() noexcept
{
    m_flags &= ~Contact::e_dirtyFlag;
}

inline bool Contact::NeedsUpdating() const noexcept
{
    return m_flags & Contact::e_dirtyFlag;
}

inline void Contact::SetFriction(Real friction) noexcept
{
    assert(friction >= 0);
    m_friction = friction;
}

inline Real Contact::GetFriction() const noexcept
{
    return m_friction;
}

inline void Contact::SetRestitution(Real restitution) noexcept
{
    m_restitution = restitution;
}

inline Real Contact::GetRestitution() const noexcept
{
    return m_restitution;
}

inline void Contact::SetTangentSpeed(LinearVelocity speed) noexcept
{
    m_tangentSpeed = speed;
}

inline LinearVelocity Contact::GetTangentSpeed() const noexcept
{
    return m_tangentSpeed;
}

inline bool Contact::HasValidToi() const noexcept
{
    return (m_flags & Contact::e_toiFlag) != 0;
}

inline Real Contact::GetToi() const
{
    assert(HasValidToi());
    return m_toi;
}

inline void Contact::SetToi(Real toi) noexcept
{
    assert(toi >= 0 && toi <= 1);
    m_toi = toi;
    m_flags |= Contact::e_toiFlag;
}

inline void Contact::UnsetToi() noexcept
{
    m_flags &= ~Contact::e_toiFlag;
}

inline void Contact::SetToiCount(substep_type value) noexcept
{
    m_toiCount = value;
}

inline Contact::substep_type Contact::GetToiCount() const noexcept
{
    return m_toiCount;
}

// Free functions...
    
using ContactPtr = Contact*;

bool HasSensor(const Contact& contact) noexcept;

bool IsImpenetrable(const Contact& contact) noexcept;

void SetAwake(const Contact& c) noexcept;

/// Resets the friction mixture to the default value.
void ResetFriction(Contact& contact);

/// Reset the restitution to the default value.
void ResetRestitution(Contact& contact) noexcept;

TOIOutput CalcToi(const Contact& contact, const ToiConf conf);

} // namespace box2d

#endif
