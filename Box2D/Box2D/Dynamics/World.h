/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include <Box2D/Common/Math.h>
#include <Box2D/Common/BlockAllocator.h>
#include <Box2D/Common/StackAllocator.h>
#include <Box2D/Dynamics/ContactManager.h>
#include <Box2D/Dynamics/WorldCallbacks.h>
#include <Box2D/Dynamics/TimeStep.h>
#include <Box2D/Dynamics/BodyList.hpp>
#include <Box2D/Dynamics/JointList.hpp>
#include <Box2D/Dynamics/ContactList.hpp>

namespace box2d {

struct AABB;
struct BodyDef;
struct Color;
struct JointDef;
class Body;
class Draw;
class Fixture;
class Joint;
class Island;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class World
{
public:
	using size_type = size_t;

	/// Construct a world object.
	/// @param gravity the world gravity vector.
	World(const Vec2& gravity);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~World();

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	void SetDestructionListener(DestructionListener* listener) noexcept;

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used. The listener is
	/// owned by you and must remain in scope. 
	void SetContactFilter(ContactFilter* filter) noexcept;

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	void SetContactListener(ContactListener* listener) noexcept;

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	void SetDebugDraw(Draw* debugDraw) noexcept;

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	Body* CreateBody(const BodyDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void DestroyBody(Body* body);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	Joint* CreateJoint(const JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void DestroyJoint(Joint* joint);

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solving.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	void Step(float_t timeStep, unsigned velocityIterations, unsigned positionIterations);

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// ClearForces after all sub-steps are complete in one pass of your game loop.
	/// @see SetAutoClearForces
	void ClearForces() noexcept;

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	void DrawDebugData();

	/// Queries the world for all fixtures that potentially overlap the provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(QueryFixtureReporter* callback, const AABB& aabb) const;

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(RayCastFixtureReporter* callback, const Vec2& point1, const Vec2& point2) const;

	/// Gets the world body list.
	/// @return Body list that can be iterated over using its begin and end methods or using ranged-based for-loops.
	BodyList& GetBodies() noexcept;

	/// Gets the world body list for this constant world.
	/// @return Body list that can be iterated over using its begin and end methods or using ranged-based for-loops.
	const BodyList& GetBodies() const noexcept;

	/// Gets the world joint list.
	/// @return World joint list.
	JointList& GetJoints() noexcept;

	/// Gets the world joint list.
	/// @return World joint list.
	const JointList& GetJoints() const noexcept;

	/// Gets the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use ContactListener to avoid missing contacts.
	/// @return the head of the world contact list.
	ContactList& GetContacts() noexcept;

	/// Gets the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use ContactListener to avoid missing contacts.
	/// @return the head of the world contact list.
	const ContactList& GetContacts() const noexcept;

	/// Enable/disable sleep.
	void SetAllowSleeping(bool flag) noexcept;
	bool GetAllowSleeping() const noexcept { return m_allowSleep; }

	/// Enable/disable warm starting. For testing.
	void SetWarmStarting(bool flag) noexcept { m_warmStarting = flag; }
	bool GetWarmStarting() const noexcept { return m_warmStarting; }

	/// Enable/disable continuous physics. For testing.
	void SetContinuousPhysics(bool flag) noexcept { m_continuousPhysics = flag; }
	bool GetContinuousPhysics() const noexcept { return m_continuousPhysics; }

	/// Enable/disable single stepped continuous physics. For testing.
	void SetSubStepping(bool flag) noexcept { m_subStepping = flag; }
	bool GetSubStepping() const noexcept { return m_subStepping; }

	/// Get the number of broad-phase proxies.
	size_type GetProxyCount() const noexcept;

	/// Get the number of bodies.
	body_count_t GetBodyCount() const noexcept;

	/// Get the number of joints.
	size_type GetJointCount() const noexcept;

	/// Get the number of contacts (each may have 0 or more contact points).
	contact_count_t GetContactCount() const noexcept;

	/// Get the height of the dynamic tree.
	size_type GetTreeHeight() const noexcept;

	/// Get the balance of the dynamic tree.
	size_type GetTreeBalance() const;

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	float_t GetTreeQuality() const;

	/// Change the global gravity vector.
	void SetGravity(const Vec2& gravity) noexcept;
	
	/// Get the global gravity vector.
	Vec2 GetGravity() const noexcept;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const noexcept;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag) noexcept;

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const noexcept;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const Vec2& newOrigin);

	/// Get the contact manager for testing.
	const ContactManager& GetContactManager() const noexcept;

	/// Get the current profile.
	const Profile& GetProfile() const noexcept;

	/// Dump the world into the log file.
	/// @warning this should be called outside of a time step.
	void Dump();

private:

	// m_flags
	enum: uint32
	{
		e_newFixture	= 0x0001,
		e_locked		= 0x0002,
		e_clearForces	= 0x0004
	};

	friend class Body;
	friend class Fixture;
	friend class ContactManager;

	/// Finds islands, integrates and solves constraints, solves position constraints.
	void Solve(const TimeStep& step);

	/// Solves Time Of Impact Events.
	/// @detail Used for continuous physics.
	/// @param step Time step value to use.
	void SolveTOI(const TimeStep& step);

	/// "Solves" collisions for the given time of impact.
	/// @param step Time step to solve for.
	/// @param contact Contact.
	/// @param toi Time of impact to solve for.
	/// @note Precondition 1: there is no contact having a lower TOI in this time step that has not already been solved for.
	/// @note Precondition 2: there is not a lower TOI in the time step for which collisions have not already been processed.
	void SolveTOI(const TimeStep& step, Contact& contact, float_t toi);

	void ResetBodiesForSolveTOI();
	void ResetContactsForSolveTOI();

	/// Processes the contacts of a given body for TOI handling.
	/// @detail This does the following:
	///   1. Advances the appropriate associated other bodies to the given TOI (advancing
	///      their sweeps and synchronizing their transforms to their new sweeps).
	///   2. Updates the contact manifolds and touching statuses and notifies listener (if one given) of
	///      the appropriate contacts of the body.
	///   3. Adds those contacts that are still enabled and still touching to the given island
	///      (or resets the other bodies advancement).
 	///   4. Adds to the island, those other bodies that haven't already been added of the contacts that got added.
	/// @note Precondition: there should be no lower TOI for which contacts have not already been processed.
	/// @param island Island.
	/// @param body A dynamic/accelerable body.
	/// @param toi Time of impact (TOI). Value between 0 and 1.
	/// @param listener Pointer to listener that will be called, or nullptr.
	static void ProcessContactsForTOI(Island& island, Body& body, float_t toi, ContactListener* listener);
	
	bool Add(Body& b);
	bool Add(Joint& j);

	bool Remove(Body& b);
	bool Remove(Joint& j);

	struct ContactToiPair
	{
		Contact* contact; ///< Contact for which the time of impact is relavant.
		float_t toi; ///< Time of impact (TOI) as a fractional value between 0 and 1.
	};

	/// Updates the contact times of impact.
	/// @detail While checking contacts and setting their time of impact values this also
	///   finds the contact with the lowest (soonest) time of impact.
	/// @return Contact with the least time of impact and its time of impact, or null contact.
	ContactToiPair UpdateContactTOIs();

	void DrawJoint(Joint* joint);
	void DrawShape(const Fixture* shape, const Transformation& xf, const Color& color);

	BlockAllocator m_blockAllocator;
	StackAllocator m_stackAllocator;
	ContactFilter m_defaultFilter;
	ContactListener m_defaultListener;

	uint32 m_flags = e_clearForces;
	
	bool HasNewFixtures() const noexcept { return (m_flags & e_newFixture) != 0; }
	void SetNewFixtures() noexcept { m_flags |= World::e_newFixture; }
	void UnsetNewFixtures() noexcept { m_flags &= ~e_newFixture; }

	ContactManager m_contactMgr{&m_blockAllocator, &m_defaultFilter, &m_defaultListener};

	BodyList m_bodies; ///< Body collection.
	JointList m_joints; ///< Joint collection.

	Vec2 m_gravity;
	bool m_allowSleep = true;

	DestructionListener* m_destructionListener = nullptr;
	Draw* g_debugDraw = nullptr;

	/// Inverse delta-t from previous step. Used to compute time step ratio to support a variable time step.
	/// @sa Step.
	float_t m_inv_dt0 = float_t{0};

	// These are for debugging the solver.
	bool m_warmStarting = true;
	bool m_continuousPhysics = true;
	bool m_subStepping = false;

	bool m_stepComplete = true;

	Profile m_profile;
};

inline BodyList& World::GetBodies() noexcept
{
	return m_bodies;
}

inline const BodyList& World::GetBodies() const noexcept
{
	return m_bodies;
}

inline JointList& World::GetJoints() noexcept
{
	return m_joints;
}

inline const JointList& World::GetJoints() const noexcept
{
	return m_joints;
}

inline ContactList& World::GetContacts() noexcept
{
	return m_contactMgr.GetContacts();
}

inline const ContactList& World::GetContacts() const noexcept
{
	return m_contactMgr.GetContacts();
}

inline body_count_t World::GetBodyCount() const noexcept
{
	return static_cast<body_count_t>(m_bodies.size());
}

inline World::size_type World::GetJointCount() const noexcept
{
	return m_joints.size();
}

inline contact_count_t World::GetContactCount() const noexcept
{
	return m_contactMgr.GetContactCount();
}

inline void World::SetGravity(const Vec2& gravity) noexcept
{
	m_gravity = gravity;
}

inline Vec2 World::GetGravity() const noexcept
{
	return m_gravity;
}

inline bool World::IsLocked() const noexcept
{
	return (m_flags & e_locked) == e_locked;
}

inline void World::SetAutoClearForces(bool flag) noexcept
{
	if (flag)
	{
		m_flags |= e_clearForces;
	}
	else
	{
		m_flags &= ~e_clearForces;
	}
}

/// Get the flag that controls automatic clearing of forces after each time step.
inline bool World::GetAutoClearForces() const noexcept
{
	return (m_flags & e_clearForces) != 0;
}

inline const ContactManager& World::GetContactManager() const noexcept
{
	return m_contactMgr;
}

inline const Profile& World::GetProfile() const noexcept
{
	return m_profile;
}
	
} // namespace box2d

#endif
