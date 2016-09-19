/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/Manifold.hpp>
#include <Box2D/Collision/Shapes/Shape.h>
#include <Box2D/Dynamics/Fixture.h>

#include <type_traits>

namespace box2d {

class Body;
class Contact;
class Fixture;
class World;
class BlockAllocator;
class StackAllocator;
class ContactListener;

/// Friction mixing law. The idea is to allow either fixture to drive the resulting friction to zero.
/// For example, anything slides on ice.
inline float_t MixFriction(float_t friction1, float_t friction2) noexcept(noexcept(Sqrt(friction1 * friction2)))
{
	return Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline float_t MixRestitution(float_t restitution1, float_t restitution2) noexcept
{
	return (restitution1 > restitution2) ? restitution1 : restitution2;
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct ContactEdge
{
	Body* other;			///< provides quick access to the other body attached.
	Contact* contact;		///< the contact
	ContactEdge* prev;	///< the previous contact edge in the body's contact list
	ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class Contact
{
public:
	using substep_type = std::remove_const<decltype(MaxSubSteps)>::type;

	using toi_max_type = std::remove_const<decltype(MaxTOIIterations)>::type;
	using dist_max_type = std::remove_const<decltype(MaxDistanceIterations)>::type;
	using root_max_type = std::remove_const<decltype(MaxTOIRootIterCount)>::type;
	
	using toi_sum_type = std::conditional<sizeof(toi_max_type) < sizeof(uint16), uint16, uint32>::type;
	using dist_sum_type = std::conditional<sizeof(dist_max_type) < sizeof(uint16), uint16, uint32>::type;
	using root_sum_type = std::conditional<sizeof(root_max_type) < sizeof(uint16), uint16, uint32>::type;
	
	Contact() = delete;

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

	/// Get fixture A in this contact.
	Fixture* GetFixtureA() noexcept;
	const Fixture* GetFixtureA() const noexcept;

	/// Get the child primitive index for fixture A.
	child_count_t GetChildIndexA() const noexcept;

	/// Get fixture B in this contact.
	Fixture* GetFixtureB() noexcept;
	const Fixture* GetFixtureB() const noexcept;

	/// Get the child primitive index for fixture B.
	child_count_t GetChildIndexB() const noexcept;

	/// Override the default friction mixture. You can call this in ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float_t friction) noexcept;

	/// Get the friction.
	float_t GetFriction() const noexcept;

	/// Override the default restitution mixture. You can call this in ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float_t restitution) noexcept;

	/// Get the restitution.
	float_t GetRestitution() const noexcept;

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float_t speed) noexcept;

	/// Gets the desired tangent speed. In meters per second.
	float_t GetTangentSpeed() const noexcept;

	/// Calculates this contact's collision manifold for the given transforms.
	/// @param xfA Transformation for the contact's fixture A shape.
	/// @param xfB Transformation for the contact's fixture B shape.
	/// @return Contact manifold for the given transforms with one or more points
	///   if the shapes are considered touching (collided).
	virtual Manifold Evaluate(const Transformation& xfA, const Transformation& xfB) const = 0;

	substep_type GetToiCount() const noexcept;

	unsigned GetToiCalls() const noexcept;

	toi_sum_type GetToiItersTotal() const noexcept;
	dist_sum_type GetDistItersTotal() const noexcept;
	root_sum_type GetRootItersTotal() const noexcept;
	
	toi_max_type GetToiItersMax() const noexcept;
	dist_max_type GetDistItersMax() const noexcept;
	root_max_type GetRootItersMax() const noexcept;

	/// Gets whether a TOI is set.
	/// @return true if this object has a TOI set for it, false otherwise.
	bool HasValidToi() const noexcept;
	
	/// Gets the time of impact (TOI) as a fraction.
	/// @note This is only valid if a TOI has been set.
	/// @sa void SetToi(float_t toi).
	/// @return Time of impact fraction in the range of 0 to 1 if set (where 1
	///   means no actual impact in current time slot), otheriwse undefined.
	float_t GetToi() const;
	
protected:
	friend class ContactManager;
	friend class World;
	friend class Body;
	friend class Fixture;
	friend class ContactList;
	friend class ContactIterator;

	// Flags stored in m_flags
	enum: uint32
	{
		// Used when crawling contact graph when forming islands.
		e_islandFlag		= 0x0001,

        // Set when the shapes are touching.
		e_touchingFlag		= 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag		= 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag		= 0x0008,

		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0010
	};

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering() noexcept;
	void UnflagForFiltering() noexcept;
	bool NeedsFiltering() const noexcept;

	static Contact* Create(Fixture& fixtureA, child_count_t indexA, Fixture& fixtureB, child_count_t indexB,
						   BlockAllocator* allocator);

	static void Destroy(Contact* contact, Shape::Type typeA, Shape::Type typeB, BlockAllocator* allocator);
	
	static void Destroy(Contact* contact, BlockAllocator* allocator);

	Contact(Fixture* fixtureA, child_count_t indexA, Fixture* fixtureB, child_count_t indexB);
	virtual ~Contact() = default;

	/// Updates the contact manifold and touching status and notifies listener (if one given).
	/// @param listener Listener that if non-null is called with status information.
	/// @sa GetManifold.
	/// @sa IsTouching.
	void Update(ContactListener* listener = nullptr);

	/// Sets the time of impact (TOI).
	/// @detail After returning, this object will have a TOI that is set as indicated by <code>HasValidToi()</code>.
	/// @note Behavior is undefined if the value assigned is less than 0 or greater than 1.
	/// @sa float_t GetToi() const.
	/// @sa HasValidToi.
	/// @param toi Time of impact as a fraction between 0 and 1 where 1 indicates no actual impact in the current time slot.
	void SetToi(float_t toi) noexcept;
	
	void UnsetToi() noexcept;

	/// Updates the time of impact information.
	/// @detail This:
	///   Ensures both bodies's sweeps are on the max alpha0 of the two (by advancing the sweep of the lesser body).
	///   Calculates whether there's an impact and if so when.
	///   Sets the new time of impact or sets it to 1.
	bool UpdateTOI();

	bool IsInIsland() const noexcept;
	void SetInIsland() noexcept;
	void UnsetInIsland() noexcept;

	/// Sets the touching flag state.
	/// @note This should only be called if either:
 	///   1. The contact's manifold has more than 0 contact points, or
	///   2. The contact has sensors and the two shapes of this contact are found to be overlapping.
	/// @sa IsTouching().
	void SetTouching() noexcept;

	void UnsetTouching() noexcept;
	
	void SetTouching(bool value) noexcept;

	uint32 m_flags = e_enabledFlag;

	// World pool and list pointers.
	Contact* m_prev = nullptr;
	Contact* m_next = nullptr;

	// Nodes for connecting bodies.
	ContactEdge m_nodeA = { nullptr, nullptr, nullptr, nullptr}; ///< Node A's contact edge.
	ContactEdge m_nodeB = { nullptr, nullptr, nullptr, nullptr}; ///< Node B's contact edge.

	Fixture* const m_fixtureA; ///< Fixture A. @detail Non-null pointer to fixture A.
	Fixture* const m_fixtureB; ///< Fixture B. @detail Non-null pointer to fixture B.

	child_count_t const m_indexA;
	child_count_t const m_indexB;

	float_t m_tangentSpeed = float_t{0};

	Manifold m_manifold; ///< Manifold of the contact.

	substep_type m_toiCount = 0; ///< Count of TOI substeps contact has gone through [0,MaxSubSteps].
	substep_type m_toiCalls = 0;

	toi_sum_type m_toiItersTotal = 0;
	toi_max_type m_max_toi_iters = 0;
	
	dist_sum_type m_distItersTotal = 0;
	root_sum_type m_rootItersTotal = 0;
	dist_max_type m_max_dist_iters = 0;
	root_max_type m_max_root_iters = 0;
	
	float_t m_toi; // only valid if m_flags & e_toiFlag

	// initialized on construction (construction-time depedent)
	float_t m_friction; ///< Mix of frictions of the associated fixtures. @sa MixFriction.
	float_t m_restitution; ///< Mix of restitutions of the associated fixtures. @sa MixRestitution.
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

inline void Contact::SetTouching(bool value) noexcept
{
	if (value)
	{
		SetTouching();
	}
	else
	{
		UnsetTouching();
	}
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

inline void Contact::SetFriction(float_t friction) noexcept
{
	m_friction = friction;
}

inline float_t Contact::GetFriction() const noexcept
{
	return m_friction;
}

inline void Contact::SetRestitution(float_t restitution) noexcept
{
	m_restitution = restitution;
}

inline float_t Contact::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void Contact::SetTangentSpeed(float_t speed) noexcept
{
	m_tangentSpeed = speed;
}

inline float_t Contact::GetTangentSpeed() const noexcept
{
	return m_tangentSpeed;
}

inline bool Contact::HasValidToi() const noexcept
{
	return (m_flags & Contact::e_toiFlag) != 0;
}

inline float_t Contact::GetToi() const
{
	assert(HasValidToi());
	return m_toi;
}

inline void Contact::SetToi(float_t toi) noexcept
{
	assert(toi >= 0 && toi <= 1);
	m_toi = toi;
	m_flags |= Contact::e_toiFlag;
}

inline void Contact::UnsetToi() noexcept
{
	m_flags &= ~Contact::e_toiFlag;
}

inline bool Contact::IsInIsland() const noexcept
{
	return m_flags & Contact::e_islandFlag;
}

inline void Contact::SetInIsland() noexcept
{
	m_flags |= Contact::e_islandFlag;
}

inline void Contact::UnsetInIsland() noexcept
{	
	m_flags &= ~Contact::e_islandFlag;
}

inline Contact::substep_type Contact::GetToiCount() const noexcept
{
	return m_toiCount;
}

inline unsigned Contact::GetToiCalls() const noexcept
{
	return m_toiCalls;
}

inline Contact::toi_sum_type Contact::GetToiItersTotal() const noexcept
{
	return m_toiItersTotal;
}

inline Contact::dist_sum_type Contact::GetDistItersTotal() const noexcept
{
	return m_distItersTotal;
}
	
inline Contact::root_sum_type Contact::GetRootItersTotal() const noexcept
{
	return m_rootItersTotal;
}

inline Contact::toi_max_type Contact::GetToiItersMax() const noexcept
{
	return m_max_toi_iters;
}

inline Contact::dist_max_type Contact::GetDistItersMax() const noexcept
{
	return m_max_dist_iters;
}

inline Contact::root_max_type Contact::GetRootItersMax() const noexcept
{
	return m_max_root_iters;
}

inline bool HasSensor(const Contact& contact) noexcept
{
	return contact.GetFixtureA()->IsSensor() || contact.GetFixtureB()->IsSensor();
}

inline void SetAwake(Contact& c) noexcept
{
	SetAwake(*c.GetFixtureA());
	SetAwake(*c.GetFixtureB());
}

/// Resets the friction mixture to the default value.
inline void ResetFriction(Contact& contact)
{
	contact.SetFriction(MixFriction(contact.GetFixtureA()->GetFriction(), contact.GetFixtureB()->GetFriction()));
}

/// Reset the restitution to the default value.
inline void ResetRestitution(Contact& contact) noexcept
{
	contact.SetRestitution(MixRestitution(contact.GetFixtureA()->GetRestitution(), contact.GetFixtureB()->GetRestitution()));
}

} // namespace box2d

#endif
