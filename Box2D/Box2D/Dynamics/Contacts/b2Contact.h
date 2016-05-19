/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Dynamics/b2Fixture.h>

#include <type_traits>

namespace box2d {

class Body;
class b2Contact;
class Fixture;
class b2World;
class b2BlockAllocator;
class b2StackAllocator;
class b2ContactListener;

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
inline float_t b2MixFriction(float_t friction1, float_t friction2)
{
	return Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline float_t b2MixRestitution(float_t restitution1, float_t restitution2) noexcept
{
	return (restitution1 > restitution2) ? restitution1 : restitution2;
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct b2ContactEdge
{
	Body* other;			///< provides quick access to the other body attached.
	b2Contact* contact;		///< the contact
	b2ContactEdge* prev;	///< the previous contact edge in the body's contact list
	b2ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class b2Contact
{
public:
	b2Contact() = delete;

	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	b2Manifold* GetManifold() noexcept;
	const b2Manifold* GetManifold() const noexcept;

	/// Get the world manifold.
	b2WorldManifold GetWorldManifold() const;

	/// Is this contact touching?
	bool IsTouching() const noexcept;

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	[[deprecated]] void SetEnabled(bool flag) noexcept;
	void SetEnabled() noexcept;
	void UnsetEnabled() noexcept;

	/// Has this contact been disabled?
	bool IsEnabled() const noexcept;

	/// Get the next contact in the world's contact list.
	b2Contact* GetNext() noexcept;
	const b2Contact* GetNext() const noexcept;

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

	/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float_t friction) noexcept;

	/// Get the friction.
	float_t GetFriction() const noexcept;

	/// Reset the friction mixture to the default value.
	void ResetFriction();

	/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float_t restitution) noexcept;

	/// Get the restitution.
	float_t GetRestitution() const noexcept;

	/// Reset the restitution to the default value.
	void ResetRestitution() noexcept;

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float_t speed) noexcept;

	/// Get the desired tangent speed. In meters per second.
	float_t GetTangentSpeed() const noexcept;

	/// Gets this contact's collision manifold for the given transforms.
	/// @param xfA Transform for the contact's fixture A shape.
	/// @param xfB Transform for the contact's fixture B shape.
	/// @return Contact manifold for the given transforms with one or more points
	///   if the shapes are considered touching (collided).
	virtual b2Manifold Evaluate(const Transform& xfA, const Transform& xfB) = 0;

protected:
	friend class b2ContactManager;
	friend class b2World;
	friend class b2ContactSolver;
	friend class Body;
	friend class Fixture;

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

		// This bullet contact had a TOI event
		e_bulletHitFlag		= 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0020
	};

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering() noexcept;
	void UnflagForFiltering() noexcept;
	bool NeedsFiltering() const noexcept;

	static b2Contact* Create(Fixture* fixtureA, child_count_t indexA,
							 Fixture* fixtureB, child_count_t indexB,
							 b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2Shape::Type typeA, b2Shape::Type typeB, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

	b2Contact(Fixture* fixtureA, child_count_t indexA, Fixture* fixtureB, child_count_t indexB);
	virtual ~b2Contact() = default;

	void Update(b2ContactListener* listener);

	/// Gets whether a TOI is set.
	/// @return true if this object has a TOI set for it, false otherwise.
	bool HasValidToi() const noexcept;

	/// Gets the time of impact (TOI) as a fraction.
	/// @note This is only valid if a TOI has been set.
	/// @sa void SetToi(float_t toi).
	/// @return Time of impact fraction in the range of 0 to 1 if set, otheriwse undefined.
	float_t GetToi() const;

	/// Sets the time of impact (TOI).
	/// @detail After returning, this object will have a TOI that is set as indicated by <code>HasValidToi()</code>.
	/// @note Behavior is undefined if the value assigned is less than 0 or greater than 1.
	/// @sa float_t GetToi() const.
	/// @sa HasValidToi.
	/// @param toi Time of impact as a fraction between 0 and 1.
	void SetToi(float_t toi) noexcept;
	
	void UnsetToi() noexcept;

	bool UpdateTOI();

	bool IsInIsland() const noexcept;
	void SetInIsland() noexcept;
	void UnsetInIsland() noexcept;

	void SetTouching() noexcept;
	void UnsetTouching() noexcept;

	uint32 m_flags = e_enabledFlag;

	// World pool and list pointers.
	b2Contact* m_prev = nullptr;
	b2Contact* m_next = nullptr;

	// Nodes for connecting bodies.
	b2ContactEdge m_nodeA = { nullptr, nullptr, nullptr, nullptr};
	b2ContactEdge m_nodeB = { nullptr, nullptr, nullptr, nullptr};

	Fixture* m_fixtureA = nullptr;
	Fixture* m_fixtureB = nullptr;

	child_count_t m_indexA = 0;
	child_count_t m_indexB = 0;

	float_t m_tangentSpeed = float_t{0};

	b2Manifold m_manifold;

	std::remove_cv<decltype(MaxSubSteps)>::type m_toiCount = 0;
	float_t m_toi; // only valid if m_flags & e_toiFlag

	// initialized on construction (construction-time depedent)
	float_t m_friction;
	float_t m_restitution;

};

inline b2Manifold* b2Contact::GetManifold() noexcept
{
	return &m_manifold;
}

inline const b2Manifold* b2Contact::GetManifold() const noexcept
{
	return &m_manifold;
}

inline b2WorldManifold b2Contact::GetWorldManifold() const
{
	const auto bodyA = m_fixtureA->GetBody();
	const auto bodyB = m_fixtureB->GetBody();
	const auto shapeA = m_fixtureA->GetShape();
	const auto shapeB = m_fixtureB->GetShape();
	return b2WorldManifold{m_manifold, bodyA->GetTransform(), shapeA->GetRadius(), bodyB->GetTransform(), shapeB->GetRadius()};
}

inline void b2Contact::SetEnabled(bool flag) noexcept
{
	if (flag)
		SetEnabled();
	else
		UnsetEnabled();
}

inline void b2Contact::SetEnabled() noexcept
{
	m_flags |= b2Contact::e_enabledFlag;
}

inline void b2Contact::UnsetEnabled() noexcept
{
	m_flags &= ~b2Contact::e_enabledFlag;
}

inline bool b2Contact::IsEnabled() const noexcept
{
	return (m_flags & e_enabledFlag) != 0;
}

inline bool b2Contact::IsTouching() const noexcept
{
	return (m_flags & e_touchingFlag) != 0;
}

inline void b2Contact::SetTouching() noexcept
{
	m_flags |= e_touchingFlag;
}

inline void b2Contact::UnsetTouching() noexcept
{
	m_flags &= ~e_touchingFlag;
}

inline b2Contact* b2Contact::GetNext() noexcept
{
	return m_next;
}

inline const b2Contact* b2Contact::GetNext() const noexcept
{
	return m_next;
}

inline Fixture* b2Contact::GetFixtureA() noexcept
{
	return m_fixtureA;
}

inline const Fixture* b2Contact::GetFixtureA() const noexcept
{
	return m_fixtureA;
}

inline Fixture* b2Contact::GetFixtureB() noexcept
{
	return m_fixtureB;
}

inline child_count_t b2Contact::GetChildIndexA() const noexcept
{
	return m_indexA;
}

inline const Fixture* b2Contact::GetFixtureB() const noexcept
{
	return m_fixtureB;
}

inline child_count_t b2Contact::GetChildIndexB() const noexcept
{
	return m_indexB;
}

inline void b2Contact::FlagForFiltering() noexcept
{
	m_flags |= e_filterFlag;
}

inline void b2Contact::UnflagForFiltering() noexcept
{
	m_flags &= ~b2Contact::e_filterFlag;
}

inline bool b2Contact::NeedsFiltering() const noexcept
{
	return m_flags & b2Contact::e_filterFlag;
}

inline void b2Contact::SetFriction(float_t friction) noexcept
{
	m_friction = friction;
}

inline float_t b2Contact::GetFriction() const noexcept
{
	return m_friction;
}

inline void b2Contact::ResetFriction()
{
	m_friction = b2MixFriction(m_fixtureA->GetFriction(), m_fixtureB->GetFriction());
}

inline void b2Contact::SetRestitution(float_t restitution) noexcept
{
	m_restitution = restitution;
}

inline float_t b2Contact::GetRestitution() const noexcept
{
	return m_restitution;
}

inline void b2Contact::ResetRestitution() noexcept
{
	m_restitution = b2MixRestitution(m_fixtureA->GetRestitution(), m_fixtureB->GetRestitution());
}

inline void b2Contact::SetTangentSpeed(float_t speed) noexcept
{
	m_tangentSpeed = speed;
}

inline float_t b2Contact::GetTangentSpeed() const noexcept
{
	return m_tangentSpeed;
}

inline bool b2Contact::HasValidToi() const noexcept
{
	return (m_flags & b2Contact::e_toiFlag) != 0;
}

inline float_t b2Contact::GetToi() const
{
	assert(HasValidToi());
	return m_toi;
}

inline void b2Contact::SetToi(float_t toi) noexcept
{
	assert(toi >= 0 && toi <= 1);
	m_toi = toi;
	m_flags |= b2Contact::e_toiFlag;
}

inline void b2Contact::UnsetToi() noexcept
{
	m_flags &= ~b2Contact::e_toiFlag;
}

inline bool b2Contact::IsInIsland() const noexcept
{
	return m_flags & b2Contact::e_islandFlag;
}

inline void b2Contact::SetInIsland() noexcept
{
	m_flags |= b2Contact::e_islandFlag;
}

inline void b2Contact::UnsetInIsland() noexcept
{	
	m_flags &= ~b2Contact::e_islandFlag;
}
	
} // namespace box2d

#endif
