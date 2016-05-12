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

#ifndef B2_BODY_H
#define B2_BODY_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Dynamics/b2FixtureList.hpp>
#include <Box2D/Dynamics/b2ConstFixtureList.hpp>
#include <memory>

class b2Fixture;
class b2Joint;
class b2Contact;
class b2Controller;
class b2World;
struct b2FixtureDef;
struct b2JointEdge;
struct b2ContactEdge;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody,
	b2_dynamicBody

	// TODO_ERIN
	//b2_bulletBody,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b2BodyDef
{
	/// This constructor sets the body definition default values.
	constexpr b2BodyDef() = default;

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	b2BodyType type = b2_staticBody;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec2 position = b2Vec2_zero;

	/// The world angle of the body in radians.
	b2Float angle = b2Float{0};

	/// The linear velocity of the body's origin in world co-ordinates.
	b2Vec2 linearVelocity = b2Vec2_zero;

	/// The angular velocity of the body.
	b2Float angularVelocity = b2Float{0};

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	b2Float linearDamping = b2Float{0};

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	b2Float angularDamping = b2Float{0};

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool allowSleep = true;

	/// Is this body initially awake or sleeping?
	bool awake = true;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation = false;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	bool bullet = false;

	/// Does this body start out active?
	bool active = true;

	/// Use this to store application specific body data.
	void* userData = nullptr;

	/// Scale the gravity applied to this body.
	b2Float gravityScale = b2Float(1);
};

/// A rigid body. These are created via b2World::CreateBody.
class b2Body
{
public:
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	b2Fixture* CreateFixture(const b2FixtureDef* def);

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use b2FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	b2Fixture* CreateFixture(const b2Shape* shape, b2Float density);

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(b2Fixture* fixture);

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b2World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(const b2Vec2& position, b2Float angle);

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	b2Transform GetTransform() const noexcept;

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	b2Vec2 GetPosition() const noexcept;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	b2Float GetAngle() const noexcept;

	/// Get the world position of the center of mass.
	b2Vec2 GetWorldCenter() const noexcept;

	/// Get the local position of the center of mass.
	b2Vec2 GetLocalCenter() const noexcept;

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const b2Vec2& v) noexcept;

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	b2Vec2 GetLinearVelocity() const noexcept;

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(b2Float omega) noexcept;

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	b2Float GetAngularVelocity() const noexcept;

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake) noexcept;

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const b2Vec2& force, bool wake) noexcept;

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(b2Float torque, bool wake) noexcept;

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake) noexcept;

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(b2Float impulse, bool wake) noexcept;

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	b2Float GetMass() const noexcept;

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	b2Float GetInertia() const noexcept;

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	b2MassData GetMassData() const noexcept;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const b2MassData* data);

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData();

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const noexcept;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	b2Vec2 GetWorldVector(const b2Vec2& localVector) const noexcept;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const noexcept;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	b2Vec2 GetLocalVector(const b2Vec2& worldVector) const noexcept;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const noexcept;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const noexcept;

	/// Get the linear damping of the body.
	b2Float GetLinearDamping() const noexcept;

	/// Set the linear damping of the body.
	void SetLinearDamping(b2Float linearDamping) noexcept;

	/// Get the angular damping of the body.
	b2Float GetAngularDamping() const noexcept;

	/// Set the angular damping of the body.
	void SetAngularDamping(b2Float angularDamping) noexcept;

	/// Get the gravity scale of the body.
	b2Float GetGravityScale() const noexcept;

	/// Set the gravity scale of the body.
	void SetGravityScale(b2Float scale) noexcept;

	/// Set the type of this body. This may alter the mass and velocity.
	void SetType(b2BodyType type);

	/// Get the type of this body.
	b2BodyType GetType() const noexcept;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag) noexcept;

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const noexcept;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag) noexcept;

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const noexcept;

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	/// @deprecated use b2Body::SetAwake() or b2Body::UnsetAwake() instead.
	[[deprecated]] void SetAwake(bool flag) noexcept;

	/// Set the sleep state of the body to awake.
	void SetAwake() noexcept;

	/// Set the sleep state of the body to sleep.
	void UnsetAwake() noexcept;

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	bool IsAwake() const noexcept;

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b2World object and remains
	/// in the body list.
	void SetActive(bool flag);

	/// Get the active state of the body.
	bool IsActive() const noexcept;

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(bool flag);

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const noexcept;

	/// Get the list of all fixtures attached to this body.
	b2Fixture* GetFixtureList() noexcept;
	const b2Fixture* GetFixtureList() const noexcept;
	b2FixtureList GetFixtures() noexcept;
	b2ConstFixtureList GetFixtures() const noexcept;

	/// Get the list of all joints attached to this body.
	b2JointEdge* GetJointList() noexcept;
	const b2JointEdge* GetJointList() const noexcept;

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use b2ContactListener.
	b2ContactEdge* GetContactList() noexcept;
	const b2ContactEdge* GetContactList() const noexcept;

	/// Get the next body in the world's body list.
	b2Body* GetNext() noexcept;
	const b2Body* GetNext() const noexcept;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Get the parent world of this body.
	b2World* GetWorld() noexcept;
	const b2World* GetWorld() const noexcept;

	/// Dump this body to a log file
	void Dump();

private:

	friend class b2World;
	friend class b2Island;
	friend class b2ContactManager;
	friend class b2ContactSolver;
	friend class b2Contact;
	
	friend class b2DistanceJoint;
	friend class b2FrictionJoint;
	friend class b2GearJoint;
	friend class b2MotorJoint;
	friend class b2MouseJoint;
	friend class b2PrismaticJoint;
	friend class b2PulleyJoint;
	friend class b2RevoluteJoint;
	friend class b2RopeJoint;
	friend class b2WeldJoint;
	friend class b2WheelJoint;

	// m_flags
	enum : uint16
	{
		e_islandFlag		= 0x0001,
		e_awakeFlag			= 0x0002,
		e_autoSleepFlag		= 0x0004,
		e_bulletFlag		= 0x0008,
		e_fixedRotationFlag	= 0x0010,
		e_activeFlag		= 0x0020,
		e_toiFlag			= 0x0040
	};
	
	static uint16 GetFlags(const b2BodyDef& bd) noexcept;

	b2Body(const b2BodyDef* bd, b2World* world);
	~b2Body();

	void SynchronizeFixtures();
	
	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	bool ShouldCollide(const b2Body* other) const;

	void Advance(b2Float t);

	void DestroyContacts();

	bool IsInIsland() const noexcept;
	void SetInIsland(bool value) noexcept;
	void SetInIsland() noexcept;
	void UnsetInIsland() noexcept;

	b2MassData CalculateMassData() const noexcept;

	b2BodyType m_type;

	uint16 m_flags = 0;

	static constexpr auto InvalidIslandIndex = static_cast<island_count_t>(-1);

	island_count_t m_islandIndex = InvalidIslandIndex;
	
	bool IsValidIslandIndex() const noexcept;

	b2Transform m_xf; ///< Transform for body origin.
	b2Sweep m_sweep; ///< Sweep motion for CCD

	b2Vec2 m_linearVelocity;
	b2Float m_angularVelocity;

	b2Vec2 m_force = b2Vec2_zero;
	b2Float m_torque = b2Float{0};

	b2World* const m_world;
	b2Body* m_prev = nullptr;
	b2Body* m_next = nullptr;

	b2Fixture* m_fixtureList = nullptr;
	b2_size_t m_fixtureCount = 0;

	b2JointEdge* m_jointList = nullptr;
	b2ContactEdge* m_contactList = nullptr;

	b2Float m_mass; ///< Mass of the body. This is the sum total mass of all associated fixtures.
	b2Float m_invMass; ///< Inverse of m_mass or 0 if m_mass == 0. @see m_mass.

	b2Float m_I = b2Float{0}; ///< Rotational inertia about the center of mass.
	b2Float m_invI = b2Float{0}; ///< Inverse of m_I or 0 if m_I == 0. @see m_I.

	b2Float m_linearDamping;
	b2Float m_angularDamping;
	b2Float m_gravityScale;

	b2Float m_sleepTime = b2Float{0};

	void* m_userData;
};

inline b2BodyType b2Body::GetType() const noexcept
{
	return m_type;
}

inline b2Transform b2Body::GetTransform() const noexcept
{
	return m_xf;
}

inline b2Vec2 b2Body::GetPosition() const noexcept
{
	return m_xf.p;
}

inline b2Float b2Body::GetAngle() const noexcept
{
	return m_sweep.a;
}

inline b2Vec2 b2Body::GetWorldCenter() const noexcept
{
	return m_sweep.c;
}

inline b2Vec2 b2Body::GetLocalCenter() const noexcept
{
	return m_sweep.localCenter;
}

inline void b2Body::SetLinearVelocity(const b2Vec2& v) noexcept
{
	if (m_type == b2_staticBody)
	{
		return;
	}

	if (v != b2Vec2_zero)
	{
		SetAwake();
	}

	m_linearVelocity = v;
}

inline b2Vec2 b2Body::GetLinearVelocity() const noexcept
{
	return m_linearVelocity;
}

inline void b2Body::SetAngularVelocity(b2Float w) noexcept
{
	if (m_type == b2_staticBody)
	{
		return;
	}

	if (w != b2Float{0})
	{
		SetAwake();
	}

	m_angularVelocity = w;
}

inline b2Float b2Body::GetAngularVelocity() const noexcept
{
	return m_angularVelocity;
}

inline b2Float b2Body::GetMass() const noexcept
{
	return m_mass;
}

inline b2Float b2Body::GetInertia() const noexcept
{
	return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
}

inline b2MassData b2Body::GetMassData() const noexcept
{
	return b2MassData{m_mass, m_sweep.localCenter, m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter)};
}

inline b2Vec2 b2Body::GetWorldPoint(const b2Vec2& localPoint) const noexcept
{
	return b2Mul(m_xf, localPoint);
}

inline b2Vec2 b2Body::GetWorldVector(const b2Vec2& localVector) const noexcept
{
	return b2Mul(m_xf.q, localVector);
}

inline b2Vec2 b2Body::GetLocalPoint(const b2Vec2& worldPoint) const noexcept
{
	return b2MulT(m_xf, worldPoint);
}

inline b2Vec2 b2Body::GetLocalVector(const b2Vec2& worldVector) const noexcept
{
	return b2MulT(m_xf.q, worldVector);
}

inline b2Vec2 b2Body::GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const noexcept
{
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline b2Vec2 b2Body::GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const noexcept
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline b2Float b2Body::GetLinearDamping() const noexcept
{
	return m_linearDamping;
}

inline void b2Body::SetLinearDamping(b2Float linearDamping) noexcept
{
	m_linearDamping = linearDamping;
}

inline b2Float b2Body::GetAngularDamping() const noexcept
{
	return m_angularDamping;
}

inline void b2Body::SetAngularDamping(b2Float angularDamping) noexcept
{
	m_angularDamping = angularDamping;
}

inline b2Float b2Body::GetGravityScale() const noexcept
{
	return m_gravityScale;
}

inline void b2Body::SetGravityScale(b2Float scale) noexcept
{
	m_gravityScale = scale;
}

inline void b2Body::SetBullet(bool flag) noexcept
{
	if (flag)
	{
		m_flags |= e_bulletFlag;
	}
	else
	{
		m_flags &= ~e_bulletFlag;
	}
}

inline bool b2Body::IsBullet() const noexcept
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void b2Body::SetAwake(bool flag) noexcept
{
	if (flag)
	{
		SetAwake();
	}
	else
	{
		UnsetAwake();
	}
}

inline void b2Body::SetAwake() noexcept
{
	if ((m_flags & e_awakeFlag) == 0)
	{
		m_flags |= e_awakeFlag;
		m_sleepTime = b2Float{0};
	}
}

inline void b2Body::UnsetAwake() noexcept
{
	m_flags &= ~e_awakeFlag;
	m_sleepTime = b2Float{0};
	m_linearVelocity = b2Vec2_zero;
	m_angularVelocity = b2Float{0};
	m_force = b2Vec2_zero;
	m_torque = b2Float{0};
}

inline bool b2Body::IsAwake() const noexcept
{
	return (m_flags & e_awakeFlag) != 0;
}

inline bool b2Body::IsActive() const noexcept
{
	return (m_flags & e_activeFlag) != 0;
}

inline bool b2Body::IsFixedRotation() const noexcept
{
	return (m_flags & e_fixedRotationFlag) != 0;
}

inline void b2Body::SetSleepingAllowed(bool flag) noexcept
{
	if (flag)
	{
		m_flags |= e_autoSleepFlag;
	}
	else
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake();
	}
}

inline bool b2Body::IsSleepingAllowed() const noexcept
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

inline b2Fixture* b2Body::GetFixtureList() noexcept
{
	return m_fixtureList;
}

inline const b2Fixture* b2Body::GetFixtureList() const noexcept
{
	return m_fixtureList;
}

inline b2FixtureList b2Body::GetFixtures() noexcept
{
	return b2FixtureList(m_fixtureList);
}

inline b2ConstFixtureList b2Body::GetFixtures() const noexcept
{
	return b2ConstFixtureList(m_fixtureList);
}

inline b2JointEdge* b2Body::GetJointList() noexcept
{
	return m_jointList;
}

inline const b2JointEdge* b2Body::GetJointList() const noexcept
{
	return m_jointList;
}

inline b2ContactEdge* b2Body::GetContactList() noexcept
{
	return m_contactList;
}

inline const b2ContactEdge* b2Body::GetContactList() const noexcept
{
	return m_contactList;
}

inline b2Body* b2Body::GetNext() noexcept
{
	return m_next;
}

inline const b2Body* b2Body::GetNext() const noexcept
{
	return m_next;
}

inline void b2Body::SetUserData(void* data) noexcept
{
	m_userData = data;
}

inline void* b2Body::GetUserData() const noexcept
{
	return m_userData;
}

inline void b2Body::ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake) noexcept
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake)
	{
		SetAwake();
	}

	// Don't accumulate a force if the body is sleeping.
	if (IsAwake())
	{
		m_force += force;
		m_torque += b2Cross(point - m_sweep.c, force);
	}
}

inline void b2Body::ApplyForceToCenter(const b2Vec2& force, bool wake) noexcept
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake)
	{
		SetAwake();
	}

	// Don't accumulate a force if the body is sleeping
	if (IsAwake())
	{
		m_force += force;
	}
}

inline void b2Body::ApplyTorque(b2Float torque, bool wake) noexcept
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake)
	{
		SetAwake();
	}

	// Don't accumulate a force if the body is sleeping
	if (IsAwake())
	{
		m_torque += torque;
	}
}

inline void b2Body::ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake) noexcept
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake)
	{
		SetAwake();
	}

	// Don't accumulate velocity if the body is sleeping
	if (IsAwake())
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
	}
}

inline void b2Body::ApplyAngularImpulse(b2Float impulse, bool wake) noexcept
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake)
	{
		SetAwake();
	}

	// Don't accumulate velocity if the body is sleeping
	if (IsAwake())
	{
		m_angularVelocity += m_invI * impulse;
	}
}

inline void b2Body::Advance(b2Float alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf = b2ComputeTransform(m_sweep);
}

inline b2World* b2Body::GetWorld() noexcept
{
	return m_world;
}

inline const b2World* b2Body::GetWorld() const noexcept
{
	return m_world;
}

inline bool b2Body::IsInIsland() const noexcept
{
	return m_flags & b2Body::e_islandFlag;
}

inline void b2Body::SetInIsland(bool value) noexcept
{
	if (value)
		SetInIsland();
	else
		UnsetInIsland();
}

inline void b2Body::SetInIsland() noexcept
{
	m_flags |= b2Body::e_islandFlag;
}

inline void b2Body::UnsetInIsland() noexcept
{
	m_flags &= ~b2Body::e_islandFlag;
}

inline bool b2Body::IsValidIslandIndex() const noexcept
{
	return IsInIsland() && (m_islandIndex != InvalidIslandIndex);
}

#endif
