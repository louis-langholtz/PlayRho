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

namespace box2d {

class Fixture;
class Joint;
class Contact;
class World;
struct FixtureDef;
struct JointEdge;
struct ContactEdge;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum BodyType
{
	StaticBody = 0,
	KinematicBody,
	DynamicBody

	// TODO_ERIN
	//BulletBody,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct BodyDef
{
	/// This constructor sets the body definition default values.
	constexpr BodyDef() = default;

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	BodyType type = StaticBody;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	Vec2 position = Vec2_zero;

	/// The world angle of the body in radians.
	float_t angle = float_t{0};

	/// The linear velocity of the body's origin in world co-ordinates.
	Vec2 linearVelocity = Vec2_zero;

	/// The angular velocity of the body.
	float_t angularVelocity = float_t{0};

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float_t linearDamping = float_t{0};

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float_t angularDamping = float_t{0};

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
	float_t gravityScale = float_t(1);
};

/// A rigid body. These are created via World::CreateBody.
class Body
{
public:
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	Fixture* CreateFixture(const FixtureDef* def);

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	Fixture* CreateFixture(const Shape* shape, float_t density);

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(Fixture* fixture);

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(const Vec2& position, float_t angle);

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	Transform GetTransform() const noexcept;

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	Vec2 GetPosition() const noexcept;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float_t GetAngle() const noexcept;

	/// Get the world position of the center of mass.
	Vec2 GetWorldCenter() const noexcept;

	/// Get the local position of the center of mass.
	Vec2 GetLocalCenter() const noexcept;

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const Vec2& v) noexcept;

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	Vec2 GetLinearVelocity() const noexcept;

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float_t omega) noexcept;

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	float_t GetAngularVelocity() const noexcept;

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const Vec2& force, const Vec2& point, bool wake) noexcept;

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const Vec2& force, bool wake) noexcept;

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(float_t torque, bool wake) noexcept;

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake) noexcept;

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(float_t impulse, bool wake) noexcept;

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	float_t GetMass() const noexcept;

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	float_t GetInertia() const noexcept;

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	MassData GetMassData() const noexcept;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const MassData* data);

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData();

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	Vec2 GetWorldPoint(const Vec2& localPoint) const noexcept;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	Vec2 GetWorldVector(const Vec2& localVector) const noexcept;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	Vec2 GetLocalPoint(const Vec2& worldPoint) const noexcept;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	Vec2 GetLocalVector(const Vec2& worldVector) const noexcept;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	Vec2 GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const noexcept;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	Vec2 GetLinearVelocityFromLocalPoint(const Vec2& localPoint) const noexcept;

	/// Get the linear damping of the body.
	float_t GetLinearDamping() const noexcept;

	/// Set the linear damping of the body.
	void SetLinearDamping(float_t linearDamping) noexcept;

	/// Get the angular damping of the body.
	float_t GetAngularDamping() const noexcept;

	/// Set the angular damping of the body.
	void SetAngularDamping(float_t angularDamping) noexcept;

	/// Get the gravity scale of the body.
	float_t GetGravityScale() const noexcept;

	/// Set the gravity scale of the body.
	void SetGravityScale(float_t scale) noexcept;

	/// Set the type of this body. This may alter the mass and velocity.
	void SetType(BodyType type);

	/// Get the type of this body.
	BodyType GetType() const noexcept;

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
	/// @deprecated use Body::SetAwake() or Body::UnsetAwake() instead.
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
	/// An inactive body is still owned by a World object and remains
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
	Fixture* GetFixtureList() noexcept;
	const Fixture* GetFixtureList() const noexcept;
	FixtureList GetFixtures() noexcept;
	ConstFixtureList GetFixtures() const noexcept;

	/// Get the list of all joints attached to this body.
	JointEdge* GetJointList() noexcept;
	const JointEdge* GetJointList() const noexcept;

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use ContactListener.
	ContactEdge* GetContactList() noexcept;
	const ContactEdge* GetContactList() const noexcept;

	/// Get the next body in the world's body list.
	Body* GetNext() noexcept;
	const Body* GetNext() const noexcept;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Get the parent world of this body.
	World* GetWorld() noexcept;
	const World* GetWorld() const noexcept;

	/// Dump this body to a log file
	void Dump();

private:

	friend class World;
	friend class Island;
	friend class ContactManager;
	friend class ContactSolver;
	friend class Contact;
	
	friend class DistanceJoint;
	friend class FrictionJoint;
	friend class GearJoint;
	friend class MotorJoint;
	friend class MouseJoint;
	friend class PrismaticJoint;
	friend class PulleyJoint;
	friend class RevoluteJoint;
	friend class RopeJoint;
	friend class WeldJoint;
	friend class WheelJoint;

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
	
	static uint16 GetFlags(const BodyDef& bd) noexcept;

	Body(const BodyDef* bd, World* world);
	~Body();

	void SynchronizeFixtures();
	
	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	bool ShouldCollide(const Body* other) const;

	/// Advances the body by a given time ratio.
	/// @detail This method:
	///    1. advances the body's sweep to the given time ratio;
	///    2. updates the body's sweep positions (linear and angular) to the advanced ones; and
	///    3. updates the body's transform to the new sweep one settings.
	/// @param t New time factor in [0,1) to advance the sweep to.
	void Advance(float_t t);

	void DestroyContacts();

	/// Checks if flagged as being in an island or not.
	/// @return true if flagged for being in an island, false otherwise.
	/// @sa Island.
	bool IsInIsland() const noexcept;

	void SetInIsland(bool value) noexcept;
	void SetInIsland() noexcept;
	void UnsetInIsland() noexcept;

	MassData CalculateMassData() const noexcept;

	BodyType m_type;

	uint16 m_flags = 0;

	static constexpr auto InvalidIslandIndex = static_cast<island_count_t>(-1);

	island_count_t m_islandIndex = InvalidIslandIndex;
	
	bool IsValidIslandIndex() const noexcept;

	Transform m_xf; ///< Transform for body origin.
	Sweep m_sweep; ///< Sweep motion for CCD

	Vec2 m_linearVelocity;
	float_t m_angularVelocity;

	Vec2 m_force = Vec2_zero;
	float_t m_torque = float_t{0};

	World* const m_world;
	Body* m_prev = nullptr;
	Body* m_next = nullptr;

	Fixture* m_fixtureList = nullptr;
	size_t m_fixtureCount = 0;

	JointEdge* m_jointList = nullptr;
	ContactEdge* m_contactList = nullptr;

	float_t m_mass; ///< Mass of the body. This is the sum total mass of all associated fixtures.
	float_t m_invMass; ///< Inverse of m_mass or 0 if m_mass == 0. @see m_mass.

	float_t m_I = float_t{0}; ///< Rotational inertia about the center of mass.
	float_t m_invI = float_t{0}; ///< Inverse of m_I or 0 if m_I == 0. @see m_I.

	float_t m_linearDamping;
	float_t m_angularDamping;
	float_t m_gravityScale;

	float_t m_sleepTime = float_t{0};

	void* m_userData;
};

inline BodyType Body::GetType() const noexcept
{
	return m_type;
}

inline Transform Body::GetTransform() const noexcept
{
	return m_xf;
}

inline Vec2 Body::GetPosition() const noexcept
{
	return m_xf.p;
}

inline float_t Body::GetAngle() const noexcept
{
	return m_sweep.a;
}

inline Vec2 Body::GetWorldCenter() const noexcept
{
	return m_sweep.c;
}

inline Vec2 Body::GetLocalCenter() const noexcept
{
	return m_sweep.localCenter;
}

inline void Body::SetLinearVelocity(const Vec2& v) noexcept
{
	if (m_type == StaticBody)
	{
		return;
	}

	if (v != Vec2_zero)
	{
		SetAwake();
	}

	m_linearVelocity = v;
}

inline Vec2 Body::GetLinearVelocity() const noexcept
{
	return m_linearVelocity;
}

inline void Body::SetAngularVelocity(float_t w) noexcept
{
	if (m_type == StaticBody)
	{
		return;
	}

	if (w != float_t{0})
	{
		SetAwake();
	}

	m_angularVelocity = w;
}

inline float_t Body::GetAngularVelocity() const noexcept
{
	return m_angularVelocity;
}

inline float_t Body::GetMass() const noexcept
{
	return m_mass;
}

inline float_t Body::GetInertia() const noexcept
{
	return m_I + m_mass * m_sweep.localCenter.LengthSquared();
}

inline MassData Body::GetMassData() const noexcept
{
	return MassData{m_mass, m_sweep.localCenter, m_I + m_mass * m_sweep.localCenter.LengthSquared()};
}

inline Vec2 Body::GetWorldPoint(const Vec2& localPoint) const noexcept
{
	return Mul(m_xf, localPoint);
}

inline Vec2 Body::GetWorldVector(const Vec2& localVector) const noexcept
{
	return Mul(m_xf.q, localVector);
}

inline Vec2 Body::GetLocalPoint(const Vec2& worldPoint) const noexcept
{
	return MulT(m_xf, worldPoint);
}

inline Vec2 Body::GetLocalVector(const Vec2& worldVector) const noexcept
{
	return MulT(m_xf.q, worldVector);
}

inline Vec2 Body::GetLinearVelocityFromWorldPoint(const Vec2& worldPoint) const noexcept
{
	return m_linearVelocity + Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline Vec2 Body::GetLinearVelocityFromLocalPoint(const Vec2& localPoint) const noexcept
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline float_t Body::GetLinearDamping() const noexcept
{
	return m_linearDamping;
}

inline void Body::SetLinearDamping(float_t linearDamping) noexcept
{
	m_linearDamping = linearDamping;
}

inline float_t Body::GetAngularDamping() const noexcept
{
	return m_angularDamping;
}

inline void Body::SetAngularDamping(float_t angularDamping) noexcept
{
	m_angularDamping = angularDamping;
}

inline float_t Body::GetGravityScale() const noexcept
{
	return m_gravityScale;
}

inline void Body::SetGravityScale(float_t scale) noexcept
{
	m_gravityScale = scale;
}

inline void Body::SetBullet(bool flag) noexcept
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

inline bool Body::IsBullet() const noexcept
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void Body::SetAwake(bool flag) noexcept
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

inline void Body::SetAwake() noexcept
{
	if ((m_flags & e_awakeFlag) == 0)
	{
		m_flags |= e_awakeFlag;
		m_sleepTime = float_t{0};
	}
}

inline void Body::UnsetAwake() noexcept
{
	m_flags &= ~e_awakeFlag;
	m_sleepTime = float_t{0};
	m_linearVelocity = Vec2_zero;
	m_angularVelocity = float_t{0};
	m_force = Vec2_zero;
	m_torque = float_t{0};
}

inline bool Body::IsAwake() const noexcept
{
	return (m_flags & e_awakeFlag) != 0;
}

inline bool Body::IsActive() const noexcept
{
	return (m_flags & e_activeFlag) != 0;
}

inline bool Body::IsFixedRotation() const noexcept
{
	return (m_flags & e_fixedRotationFlag) != 0;
}

inline void Body::SetSleepingAllowed(bool flag) noexcept
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

inline bool Body::IsSleepingAllowed() const noexcept
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

inline Fixture* Body::GetFixtureList() noexcept
{
	return m_fixtureList;
}

inline const Fixture* Body::GetFixtureList() const noexcept
{
	return m_fixtureList;
}

inline FixtureList Body::GetFixtures() noexcept
{
	return FixtureList(m_fixtureList);
}

inline ConstFixtureList Body::GetFixtures() const noexcept
{
	return ConstFixtureList(m_fixtureList);
}

inline JointEdge* Body::GetJointList() noexcept
{
	return m_jointList;
}

inline const JointEdge* Body::GetJointList() const noexcept
{
	return m_jointList;
}

inline ContactEdge* Body::GetContactList() noexcept
{
	return m_contactList;
}

inline const ContactEdge* Body::GetContactList() const noexcept
{
	return m_contactList;
}

inline Body* Body::GetNext() noexcept
{
	return m_next;
}

inline const Body* Body::GetNext() const noexcept
{
	return m_next;
}

inline void Body::SetUserData(void* data) noexcept
{
	m_userData = data;
}

inline void* Body::GetUserData() const noexcept
{
	return m_userData;
}

inline void Body::ApplyForce(const Vec2& force, const Vec2& point, bool wake) noexcept
{
	if (m_type != DynamicBody)
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
		m_torque += Cross(point - m_sweep.c, force);
	}
}

inline void Body::ApplyForceToCenter(const Vec2& force, bool wake) noexcept
{
	if (m_type != DynamicBody)
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

inline void Body::ApplyTorque(float_t torque, bool wake) noexcept
{
	if (m_type != DynamicBody)
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

inline void Body::ApplyLinearImpulse(const Vec2& impulse, const Vec2& point, bool wake) noexcept
{
	if (m_type != DynamicBody)
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
		m_angularVelocity += m_invI * Cross(point - m_sweep.c, impulse);
	}
}

inline void Body::ApplyAngularImpulse(float_t impulse, bool wake) noexcept
{
	if (m_type != DynamicBody)
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

inline void Body::Advance(float_t alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf = GetTransformOne(m_sweep);
}

inline World* Body::GetWorld() noexcept
{
	return m_world;
}

inline const World* Body::GetWorld() const noexcept
{
	return m_world;
}

inline bool Body::IsInIsland() const noexcept
{
	return m_flags & Body::e_islandFlag;
}

inline void Body::SetInIsland(bool value) noexcept
{
	if (value)
		SetInIsland();
	else
		UnsetInIsland();
}

inline void Body::SetInIsland() noexcept
{
	m_flags |= Body::e_islandFlag;
}

inline void Body::UnsetInIsland() noexcept
{
	m_flags &= ~Body::e_islandFlag;
}

inline bool Body::IsValidIslandIndex() const noexcept
{
	return IsInIsland() && (m_islandIndex != InvalidIslandIndex);
}

} // namespace box2d

#endif
