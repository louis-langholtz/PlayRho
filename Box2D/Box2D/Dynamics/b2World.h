/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2StackAllocator.h>
#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Dynamics/b2BodyList.hpp>
#include <Box2D/Dynamics/b2ConstBodyList.hpp>

namespace box2d {

struct b2AABB;
struct b2BodyDef;
struct b2Color;
struct b2JointDef;
class b2Body;
class b2Draw;
class b2Fixture;
class b2Joint;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class b2World
{
public:
	using size_type = b2_size_t;

	/// Construct a world object.
	/// @param gravity the world gravity vector.
	b2World(const b2Vec2& gravity);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b2World();

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	void SetDestructionListener(b2DestructionListener* listener) noexcept;

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b2_defaultFilter). The listener is
	/// owned by you and must remain in scope. 
	void SetContactFilter(b2ContactFilter* filter) noexcept;

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	void SetContactListener(b2ContactListener* listener) noexcept;

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with b2World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	void SetDebugDraw(b2Draw* debugDraw) noexcept;

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	b2Body* CreateBody(const b2BodyDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void DestroyBody(b2Body* body);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	b2Joint* CreateJoint(const b2JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void DestroyJoint(b2Joint* joint);

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	void Step(float_t timeStep, int32 velocityIterations, int32 positionIterations);

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

	/// Query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const;

	/// Get the world body list. With the returned body, use b2Body::GetNext to get
	/// the next body in the world list. A nullptr body indicates the end of the list.
	/// @return the head of the world body list.
	b2Body* GetBodyList() noexcept;
	const b2Body* GetBodyList() const noexcept;

	b2BodyList GetBodies() noexcept;
	b2ConstBodyList GetBodies() const noexcept;

	/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	/// the next joint in the world list. A nullptr joint indicates the end of the list.
	/// @return the head of the world joint list.
	b2Joint* GetJointList() noexcept;
	const b2Joint* GetJointList() const noexcept;

	/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	/// the next contact in the world list. A nullptr contact indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use b2ContactListener to avoid missing contacts.
	b2Contact* GetContactList() noexcept;
	const b2Contact* GetContactList() const noexcept;

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
	size_type GetBodyCount() const noexcept;

	/// Get the number of joints.
	size_type GetJointCount() const noexcept;

	/// Get the number of contacts (each may have 0 or more contact points).
	size_type GetContactCount() const noexcept;

	/// Get the height of the dynamic tree.
	size_type GetTreeHeight() const noexcept;

	/// Get the balance of the dynamic tree.
	size_type GetTreeBalance() const;

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	float_t GetTreeQuality() const;

	/// Change the global gravity vector.
	void SetGravity(const b2Vec2& gravity) noexcept;
	
	/// Get the global gravity vector.
	b2Vec2 GetGravity() const noexcept;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const noexcept;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag) noexcept;

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const noexcept;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

	/// Get the contact manager for testing.
	const b2ContactManager& GetContactManager() const noexcept;

	/// Get the current profile.
	const b2Profile& GetProfile() const noexcept;

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

	friend class b2Body;
	friend class b2Fixture;
	friend class b2ContactManager;
	friend class b2Controller;

	void Solve(const b2TimeStep& step);
	void SolveTOI(const b2TimeStep& step);

	void DrawJoint(b2Joint* joint);
	void DrawShape(const b2Fixture* shape, const b2Transform& xf, const b2Color& color);

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;
	b2ContactFilter m_defaultFilter;
	b2ContactListener m_defaultListener;

	uint32 m_flags = e_clearForces;
	
	bool HasNewFixtures() const noexcept { return (m_flags & e_newFixture) != 0; }
	void SetNewFixtures() noexcept { m_flags |= b2World::e_newFixture; }
	void UnsetNewFixtures() noexcept { m_flags &= ~e_newFixture; }

	b2ContactManager m_contactManager{&m_blockAllocator, &m_defaultFilter, &m_defaultListener};

	b2Body* m_bodyList = nullptr;
	b2Joint* m_jointList = nullptr;

	size_type m_bodyCount = 0;
	size_type m_jointCount = 0;

	b2Vec2 m_gravity;
	bool m_allowSleep = true;

	b2DestructionListener* m_destructionListener = nullptr;
	b2Draw* g_debugDraw = nullptr;

	/// Used to compute the time step ratio to support a variable time step.
	float_t m_inv_dt0 = float_t{0};

	// These are for debugging the solver.
	bool m_warmStarting = true;
	bool m_continuousPhysics = true;
	bool m_subStepping = false;

	bool m_stepComplete = true;

	b2Profile m_profile;
};

inline b2Body* b2World::GetBodyList() noexcept
{
	return m_bodyList;
}

inline const b2Body* b2World::GetBodyList() const noexcept
{
	return m_bodyList;
}

inline b2BodyList b2World::GetBodies() noexcept
{
	return b2BodyList(m_bodyList);
}

inline b2ConstBodyList b2World::GetBodies() const noexcept
{
	return b2ConstBodyList(m_bodyList);
}

inline b2Joint* b2World::GetJointList() noexcept
{
	return m_jointList;
}

inline const b2Joint* b2World::GetJointList() const noexcept
{
	return m_jointList;
}

inline b2Contact* b2World::GetContactList() noexcept
{
	return m_contactManager.GetContactList();
}

inline const b2Contact* b2World::GetContactList() const noexcept
{
	return m_contactManager.GetContactList();
}

inline b2World::size_type b2World::GetBodyCount() const noexcept
{
	return m_bodyCount;
}

inline b2World::size_type b2World::GetJointCount() const noexcept
{
	return m_jointCount;
}

inline b2World::size_type b2World::GetContactCount() const noexcept
{
	return m_contactManager.GetContactCount();
}

inline void b2World::SetGravity(const b2Vec2& gravity) noexcept
{
	m_gravity = gravity;
}

inline b2Vec2 b2World::GetGravity() const noexcept
{
	return m_gravity;
}

inline bool b2World::IsLocked() const noexcept
{
	return (m_flags & e_locked) == e_locked;
}

inline void b2World::SetAutoClearForces(bool flag) noexcept
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
inline bool b2World::GetAutoClearForces() const noexcept
{
	return (m_flags & e_clearForces) != 0;
}

inline const b2ContactManager& b2World::GetContactManager() const noexcept
{
	return m_contactManager;
}

inline const b2Profile& b2World::GetProfile() const noexcept
{
	return m_profile;
}
	
} // namespace box2d

#endif
