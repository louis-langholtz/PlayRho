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

#ifndef B2_CONTACT_H
#define B2_CONTACT_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>

namespace box2d {

class Body;
class Contact;
class Fixture;
class ContactListener;
struct ToiConf;
	
/// Friction mixing law. The idea is to allow either fixture to drive the resulting friction to zero.
/// For example, anything slides on ice.
inline RealNum MixFriction(RealNum friction1, RealNum friction2) noexcept(noexcept(Sqrt(friction1 * friction2)))
{
	return Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline RealNum MixRestitution(RealNum restitution1, RealNum restitution2) noexcept
{
	return (restitution1 > restitution2) ? restitution1 : restitution2;
}

/// Contact.
/// @detail
/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
/// @note This data structure is 112-bytes large (on at least one 64-bit platform).
class Contact
{
public:
	using substep_type = ts_iters_t;
	
	using ManifoldCalcFunc = Manifold (*)(const Fixture* fixtureA, child_count_t indexA,
										  const Fixture* fixtureB, child_count_t indexB);
	
	Contact(Fixture* fixtureA, child_count_t indexA, Fixture* fixtureB, child_count_t indexB,
			ManifoldCalcFunc mcf);
	
	Contact() = delete;
	Contact(const Contact& copy) = delete;
	
	/// Gets the contact manifold.
	/// @warning Do not modify the manifold unless you understand the internals of Box2D.
	Manifold& GetManifold() noexcept;
	const Manifold& GetManifold() const noexcept;

	/// Is this contact touching?
	/// @detail
	/// Touching is defined as either:
	///   1. This contact's manifold has more than 0 contact points, or
	///   2. This contact has sensors and the two shapes of this contact are found to be overlapping.
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
	Fixture* GetFixtureA() noexcept;
	
	/// Gets fixture A in this contact.
	const Fixture* GetFixtureA() const noexcept;

	/// Get the child primitive index for fixture A.
	child_count_t GetChildIndexA() const noexcept;

	/// Gets fixture B in this contact.
	Fixture* GetFixtureB() noexcept;
	
	/// Gets fixture B in this contact.
	const Fixture* GetFixtureB() const noexcept;

	/// Get the child primitive index for fixture B.
	child_count_t GetChildIndexB() const noexcept;

	/// Override the default friction mixture. You can call this in ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(RealNum friction) noexcept;

	/// Gets the combined friction of the two fixtures associated with this contact.
	/// @sa MixFriction.
	RealNum GetFriction() const noexcept;

	/// Override the default restitution mixture. You can call this in ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(RealNum restitution) noexcept;

	/// Get the restitution.
	RealNum GetRestitution() const noexcept;

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(RealNum speed) noexcept;

	/// Gets the desired tangent speed. In meters per second.
	RealNum GetTangentSpeed() const noexcept;

	/// Calculates this contact's collision manifold.
	/// @return Contact manifold with one or more points
	///   if the shapes are considered touching (collided).
	Manifold Evaluate() const;

	substep_type GetToiCount() const noexcept;
	
	/// Gets whether a TOI is set.
	/// @return true if this object has a TOI set for it, false otherwise.
	bool HasValidToi() const noexcept;
	
	/// Gets the time of impact (TOI) as a fraction.
	/// @note This is only valid if a TOI has been set.
	/// @sa void SetToi(RealNum toi).
	/// @return Time of impact fraction in the range of 0 to 1 if set (where 1
	///   means no actual impact in current time slot), otheriwse undefined.
	RealNum GetToi() const;

	void FlagForFiltering() noexcept;
	bool NeedsFiltering() const noexcept;
	
protected:

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void UnflagForFiltering() noexcept;

	static Contact* Create(Fixture& fixtureA, child_count_t indexA,
						   Fixture& fixtureB, child_count_t indexB);

	//static void Destroy(Contact* contact, Shape::Type typeA, Shape::Type typeB);
	
	/// Destroys the given contact.
	/// @note This awakens the associated fixtures of a non-sensor touching contact.
	/// @note This calls the contact's destructor.
	static void Destroy(Contact* contact);

	/// Updates the contact manifold and touching status and notifies listener (if one given).
	/// @param listener Listener that if non-null is called with status information.
	/// @sa GetManifold.
	/// @sa IsTouching.
	void Update(ContactListener* listener = nullptr);

	/// Sets the time of impact (TOI).
	/// @detail After returning, this object will have a TOI that is set as indicated by <code>HasValidToi()</code>.
	/// @note Behavior is undefined if the value assigned is less than 0 or greater than 1.
	/// @sa RealNum GetToi() const.
	/// @sa HasValidToi.
	/// @param toi Time of impact as a fraction between 0 and 1 where 1 indicates no actual impact in the current time slot.
	void SetToi(RealNum toi) noexcept;
	
	void UnsetToi() noexcept;
	
	void ResetToiCount() noexcept;
	
private:

	friend class ContactAtty;
	
	/// Flags type data type.
	using FlagsType = uint8;

	// Flags stored in m_flags
	enum: FlagsType
	{
		// Set when the shapes are touching.
		e_touchingFlag		= 0x0002,
		
		// This contact can be disabled (by user)
		e_enabledFlag		= 0x0004,
		
		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag		= 0x0008,
		
		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0010
	};
	
	/// Sets the touching flag state.
	/// @note This should only be called if either:
	///   1. The contact's manifold has more than 0 contact points, or
	///   2. The contact has sensors and the two shapes of this contact are found to be overlapping.
	/// @sa IsTouching().
	void SetTouching() noexcept;
	
	void UnsetTouching() noexcept;
	
	// Member variables...
	
	/// Manifold calculating function.
	/// @note This is a use of the strategy pattern via a function pointer rather than
	///   using a virtual method and subclassing this class.
	ManifoldCalcFunc const m_manifoldCalcFunc;

	Fixture* const m_fixtureA; ///< Fixture A. @detail Non-null pointer to fixture A.
	Fixture* const m_fixtureB; ///< Fixture B. @detail Non-null pointer to fixture B.

	child_count_t const m_indexA;
	child_count_t const m_indexB;

	Manifold m_manifold; ///< Manifold of the contact. 60-bytes. @sa Update.

	substep_type m_toiCount = 0; ///< Count of TOI calculations contact has gone through since last reset.
	
	FlagsType m_flags = e_enabledFlag;
	
	RealNum m_tangentSpeed = 0;

	
	/// Time of impact.
	/// @note This is a unit interval of time (a value between 0 and 1).
	/// @note Only valid if m_flags & e_toiFlag
	RealNum m_toi;

	// initialized on construction (construction-time depedent)
	RealNum m_friction; ///< Mix of frictions of the associated fixtures. @sa MixFriction.
	RealNum m_restitution; ///< Mix of restitutions of the associated fixtures. @sa MixRestitution.
};

inline Manifold& Contact::GetManifold() noexcept
{
	return m_manifold;
}

inline const Manifold& Contact::GetManifold() const noexcept
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

inline Fixture* Contact::GetFixtureA() noexcept
{
	return m_fixtureA;
}

inline const Fixture* Contact::GetFixtureA() const noexcept
{
	return m_fixtureA;
}

inline Fixture* Contact::GetFixtureB() noexcept
{
	return m_fixtureB;
}

inline child_count_t Contact::GetChildIndexA() const noexcept
{
	return m_indexA;
}

inline const Fixture* Contact::GetFixtureB() const noexcept
{
	return m_fixtureB;
}

inline child_count_t Contact::GetChildIndexB() const noexcept
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

inline void Contact::SetFriction(RealNum friction) noexcept
{
	m_friction = friction;
}

inline RealNum Contact::GetFriction() const noexcept
{
	return m_friction;
}

inline void Contact::SetRestitution(RealNum restitution) noexcept
{
	m_restitution = restitution;
}

inline RealNum Contact::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void Contact::SetTangentSpeed(RealNum speed) noexcept
{
	m_tangentSpeed = speed;
}

inline RealNum Contact::GetTangentSpeed() const noexcept
{
	return m_tangentSpeed;
}

inline bool Contact::HasValidToi() const noexcept
{
	return (m_flags & Contact::e_toiFlag) != 0;
}

inline RealNum Contact::GetToi() const
{
	assert(HasValidToi());
	return m_toi;
}

inline void Contact::SetToi(RealNum toi) noexcept
{
	assert(toi >= 0 && toi <= 1);
	m_toi = toi;
	m_flags |= Contact::e_toiFlag;
}
	
inline void Contact::UnsetToi() noexcept
{
	m_flags &= ~Contact::e_toiFlag;
}

inline void Contact::ResetToiCount() noexcept
{
	m_toiCount = 0;
}

inline Contact::substep_type Contact::GetToiCount() const noexcept
{
	return m_toiCount;
}

inline Manifold Contact::Evaluate() const
{
	return m_manifoldCalcFunc(m_fixtureA, m_indexA, m_fixtureB, m_indexB);
}

// Free functions...

bool HasSensor(const Contact& contact) noexcept;

bool IsImpenetrable(const Contact& contact) noexcept;
	
void SetAwake(Contact& c) noexcept;

/// Resets the friction mixture to the default value.
void ResetFriction(Contact& contact);

/// Reset the restitution to the default value.
void ResetRestitution(Contact& contact) noexcept;

} // namespace box2d

#endif
