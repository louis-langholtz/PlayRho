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

#ifndef B2_BODY_H
#define B2_BODY_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/Shapes/Shape.h>
#include <Box2D/Dynamics/BodyList.hpp>
#include <Box2D/Dynamics/FixtureList.hpp>
#include <Box2D/Dynamics/Contacts/ContactEdgeList.hpp>
#include <Box2D/Dynamics/Joints/JointEdgeList.hpp>
#include <memory>

namespace box2d {

class Fixture;
class Joint;
class Contact;
class World;
struct FixtureDef;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum class BodyType
{
	/// Static body type.
	/// @detail
	/// Static bodies have no mass, have no forces applied to them, and aren't moved by physical processeses.
	/// They are impenetrable.
	/// Physics applied: none.
	Static = 0,
	
	/// Kinematic body type.
	/// @detail
	/// Kinematic bodies have no mass and have no forces applied to them, but can move at set velocities.
	/// They are impenetrable.
	/// Physics applied: velocity.
	Kinematic,

	/// Dynamic body type.
	/// @detail
	/// Dynamic bodies are fully simulated bodies.
	/// Dynamic bodies always have a positive non-zero mass.
	/// They may be penetrable.
	/// Physics applied: velocity, acceleration.
	Dynamic

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
	BodyType type = BodyType::Static;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	Vec2 position = Vec2_zero;

	/// The world angle of the body in radians.
	float_t angle = float_t{0};

	/// The linear velocity of the body's origin in world co-ordinates (in m/s).
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
	/// @note Use this flag sparingly since it increases processing time.
	bool bullet = false;

	/// Does this body start out active?
	bool active = true;

	/// Use this to store application specific body data.
	void* userData = nullptr;
};

/// A rigid body. These are created via World::CreateBody.
class Body
{
public:
	static constexpr auto InvalidIslandIndex = static_cast<body_count_t>(-1);
	
	/// Creates a fixture and attaches it to this body.
	/// @detail 
	/// Use this function if you need to set some fixture parameters, like friction.
	/// Otherwise you can create the fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	Fixture* CreateFixture(const FixtureDef& def);

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
	Transformation GetTransformation() const noexcept;

	/// Gets the world body origin position.
	/// @detail This is the position of the body's origin relative to its world.
	/// The position of the body after stepping the world's physics simulations is dependent on a number of factors:
	///   1. Position at the last time step.
	///   2. Forces acting on the body (gravity, applied force, applied impulse).
	///   3. The mass data of the body.
	///   4. Damping of the body.
	///   5. Restitution and friction values of the body's fixtures when they experience collisions.
	/// @return World position of the body's origin.
	Vec2 GetPosition() const noexcept;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float_t GetAngle() const noexcept;

	/// Get the world position of the center of mass.
	Vec2 GetWorldCenter() const noexcept;

	/// Get the local position of the center of mass.
	Vec2 GetLocalCenter() const noexcept;

	Velocity GetVelocity() const noexcept;

	void SetVelocity(const Velocity& v) noexcept;

	/// Sets the linear and rotational accelerations on this body.
	/// @note This has no effect on non-accelerable bodies.
	/// @param linear Linear acceleration.
	/// @param angular Angular acceleration.
	void SetAcceleration(const Vec2& linear, const float_t angular) noexcept;

	Vec2 GetLinearAcceleration() const noexcept;

	float_t GetAngularAcceleration() const noexcept;

	/// Gets the inverse total mass of the body.
	/// @detail This is the cached result of dividing 1 by the body's mass.
	/// Often floating division is much slower than multiplication.
	/// As such, it's likely faster to multiply values by this inverse value than to redivide
	/// them all the time by the mass.
	/// @return Value of zero or more representing the body's inverse mass (in 1/kg).
	/// @sa SetMassData.
	float_t GetInverseMass() const noexcept;
	
	/// Gets the inverse rotational inertia of the body.
	/// @detail This is the cached result of dividing 1 by the body's rotational inertia.
	/// Often floating division is much slower than multiplication.
	/// As such, it's likely faster to multiply values by this inverse value than to redivide
	/// them all the time by the rotational inertia.
	/// @return Inverse rotational intertia (in 1/kg-m^2).
	float_t GetInverseInertia() const noexcept;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param data the mass properties.
	void SetMassData(const MassData& data);

	/// Resets the mass data properties.
	/// @detail This resets the mass data to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData();

	/// Get the linear damping of the body.
	float_t GetLinearDamping() const noexcept;

	/// Set the linear damping of the body.
	void SetLinearDamping(float_t linearDamping) noexcept;

	/// Get the angular damping of the body.
	float_t GetAngularDamping() const noexcept;

	/// Set the angular damping of the body.
	void SetAngularDamping(float_t angularDamping) noexcept;

	/// Set the type of this body. This may alter the mass and velocity.
	void SetType(BodyType type);

	/// Get the type of this body.
	BodyType GetType() const noexcept;

	/// Is "speedable".
	/// @detail Is this body able to have a non-zero speed associated with it.
	/// Kinematic and Dynamic bodies are speedable. Static bodies are not.
	bool IsSpeedable() const noexcept;

	/// Is accelerable.
	/// @detail Indicates whether this body is accelerable, ie. whether it is effected by
	///   forces. Only Dynamic bodies are accelerable.
	/// @return true if the body is accelerable, false otherwise.
	bool IsAccelerable() const noexcept;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag) noexcept;

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsImpenetrable() const noexcept;

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

	/// Gets the list of all fixtures attached to this body.
	FixtureList& GetFixtures() noexcept;
	
	/// Gets the list of all fixtures attached to this body.
	const FixtureList& GetFixtures() const noexcept;

	/// Gets the list of all joints attached to this body.
	JointEdgeList& GetJoints() noexcept;
	
	/// Gets the list of all joints attached to this body.
	const JointEdgeList& GetJoints() const noexcept;

	/// Gets the list of all contacts attached to this body.
	/// @warning This list changes during the time step and you may
	/// miss some collisions if you don't use ContactListener.
	ContactEdgeList& GetContactEdges() noexcept;

	/// Gets the list of all contacts attached to this body.
	/// @warning This list changes during the time step and you may
	/// miss some collisions if you don't use ContactListener.
	const ContactEdgeList& GetContactEdges() const noexcept;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Gets the parent world of this body.
	World* GetWorld() noexcept;

	/// Gets the parent world of this body.
	const World* GetWorld() const noexcept;

	/// Checks if flagged as being in an island or not.
	/// @return true if flagged for being in an island, false otherwise.
	/// @sa Island.
	bool IsInIsland() const noexcept;
	
	body_count_t GetIslandIndex() const noexcept;

	/// Dump this body to a log file
	void Dump();

private:

	friend class World;
	friend class Island;
	friend class ContactManager;
	friend class Contact;
	friend class BodyIterator;
	friend class ConstBodyIterator;
	friend class BodyList;
	
	using flags_type = uint16;

	// m_flags
	enum Flag: flags_type
	{
		/// Island flag.
		e_islandFlag		= 0x0001,
		
		/// Awake flag.
		e_awakeFlag			= 0x0002,
		
		/// Auto sleep flag.
		e_autoSleepFlag		= 0x0004,

		/// Impenetrable flag.
		/// @detail Indicates whether CCD should be done for this body.
		/// All static and kinematic bodies have this flag enabled.
		e_impenetrableFlag	= 0x0008,
		
		/// Fixed rotation flag.
		e_fixedRotationFlag	= 0x0010,
		
		/// Active flag.
		e_activeFlag		= 0x0020,

		/// TOI valid flag.
		/// @detail Indicates whether the TOI field is valid.
		/// Enabled indicates TOI field is valid. It's otherwise invalid.
		e_toiFlag			= 0x0040,
		
		/// Velocity flag.
		/// @detail Set this to enable changes in position due to velocity.
		/// Bodies with this set are either kinematic or dynamic bodies.
		e_velocityFlag      = 0x0080,

		/// Acceleration flag.
		/// @detail Set this to enable changes in velocity due to physical properties (like forces).
		/// Bodies with this set are dynamic bodies.
		e_accelerationFlag  = 0x0100
	};
	
	static uint16 GetFlags(const BodyDef& bd) noexcept;

	Body(const BodyDef& bd, World* world);
	~Body();

	void SynchronizeFixtures();
	
	/// Determines whether this body should possibly be able to collide with the given other body.
	/// @return true if either body is dynamic and no joint prevents collision, false otherwise.
	bool ShouldCollide(const Body* other) const;

	/// Advances the body by a given time ratio.
	/// @detail This method:
	///    1. advances the body's sweep to the given time ratio;
	///    2. updates the body's sweep positions (linear and angular) to the advanced ones; and
	///    3. updates the body's transform to the new sweep one settings.
	/// @param t New time factor in [0,1) to advance the sweep to.
	void Advance(float_t t);

	void DestroyContacts();
	void DestroyJoints();
	void DestroyFixtures();

	[[deprecated]] void SetInIsland(bool value) noexcept;

	void SetInIsland() noexcept;

	/// Unsets the in-island flag.
	/// @detail Afterwards, IsInIsland returns false. This does the opposite of what SetInIsland() does.
	/// @sa SetInIsland().
	/// @sa IsInIsland().
	void UnsetInIsland() noexcept;

	/// Computes mass data.
 	/// @detail This basically accumulates the mass data over all fixtures.
	/// @note The center is the mass weighted sum of all fixture centers. Make sure to divide it by the mass to get the averaged center.
	/// @return accumalated mass data for all fixtures associated with this body.
	MassData ComputeMassData() const noexcept;

	//
	// Member variables. Try to keep total size small.
	//

	flags_type m_flags = 0; ///< Flags. 2-bytes.

	body_count_t m_islandIndex = InvalidIslandIndex; ///< Index of this body in its island (only valid when in an island). 2-bytes.
	
	Transformation m_xf; ///< Transformation for body origin. 16-bytes.
	Sweep m_sweep; ///< Sweep motion for CCD. 36-bytes.

	Velocity m_velocity; ///< Velocity (linear and angular). 12-bytes.

	Vec2 m_linearAcceleration = Vec2_zero; ///< Linear acceleration. 8-bytes.
	float_t m_angularAcceleration = float_t{0}; ///< Angular acceleration. 4-bytes.

	World* const m_world; ///< World to which this body belongs. 8-bytes.
	Body* m_prev = nullptr; ///< Previous body. 8-bytes.
	Body* m_next = nullptr; ///< Next body. 8-bytes.

	FixtureList m_fixtures; ///< Container of fixtures. 8-bytes.
	JointEdgeList m_joints; ///< Container of joint edges. 8-bytes.
	ContactEdgeList m_contacts; ///< Container of contact edges. 8-bytes.

	/// Inverse mass of the body.
	/// @detail A non-negative value (in units of 1/kg).
	/// Can only be zero for non-accelerable bodies.
	float_t m_invMass = float_t{0};	
	
	/// Inverse rotational inertia about the center of mass.
	/// @detail A non-negative value (in units of 1/(kg*m^2)).
	float_t m_invI = float_t{0};

	float_t m_linearDamping; ///< Linear damping.
	float_t m_angularDamping; ///< Angular damping.

	float_t m_sleepTime = float_t{0};

	void* m_userData; ///< User data. 8-bytes.
};

inline BodyType Body::GetType() const noexcept
{
	switch (m_flags & (e_accelerationFlag|e_velocityFlag))
	{
		case e_velocityFlag|e_accelerationFlag: return BodyType::Dynamic;
		case e_velocityFlag: return BodyType::Kinematic;
		default: break; // handle case 0 this way so compiler doesn't warn of no default handling.
	}
	return BodyType::Static;
}

inline Transformation Body::GetTransformation() const noexcept
{
	return m_xf;
}

inline Vec2 Body::GetPosition() const noexcept
{
	return m_xf.p;
}

inline float_t Body::GetAngle() const noexcept
{
	return m_sweep.pos1.a;
}

inline Vec2 Body::GetWorldCenter() const noexcept
{
	return m_sweep.pos1.c;
}

inline Vec2 Body::GetLocalCenter() const noexcept
{
	return m_sweep.GetLocalCenter();
}

inline Velocity Body::GetVelocity() const noexcept
{
	return m_velocity;
}
	
inline float_t Body::GetInverseMass() const noexcept
{
	return m_invMass;
}

inline float_t Body::GetInverseInertia() const noexcept
{
	return m_invI;
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

inline void Body::SetBullet(bool flag) noexcept
{
	if (flag)
	{
		m_flags |= e_impenetrableFlag;
	}
	else
	{
		m_flags &= ~e_impenetrableFlag;
	}
}

inline bool Body::IsImpenetrable() const noexcept
{
	return (m_flags & e_impenetrableFlag) != 0;
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
	m_velocity = Velocity{Vec2_zero, 0};
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

inline bool Body::IsSpeedable() const noexcept
{
	return (m_flags & e_velocityFlag) != 0;
}

inline bool Body::IsAccelerable() const noexcept
{
	return (m_flags & e_accelerationFlag) != 0;
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
	return (m_flags & e_autoSleepFlag) != 0;
}

inline FixtureList& Body::GetFixtures() noexcept
{
	return m_fixtures;
}

inline const FixtureList& Body::GetFixtures() const noexcept
{
	return m_fixtures;
}

inline JointEdgeList& Body::GetJoints() noexcept
{
	return m_joints;
}

inline const JointEdgeList& Body::GetJoints() const noexcept
{
	return m_joints;
}

inline ContactEdgeList& Body::GetContactEdges() noexcept
{
	return m_contacts;
}

inline const ContactEdgeList& Body::GetContactEdges() const noexcept
{
	return m_contacts;
}

inline void Body::SetUserData(void* data) noexcept
{
	m_userData = data;
}

inline void* Body::GetUserData() const noexcept
{
	return m_userData;
}

inline Vec2 Body::GetLinearAcceleration() const noexcept
{
	return m_linearAcceleration;
}

inline float_t Body::GetAngularAcceleration() const noexcept
{
	return m_angularAcceleration;
}

inline void Body::Advance(float_t alpha)
{
	//assert(m_sweep.GetAlpha0() <= alpha);

	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance0(alpha);
	m_sweep.pos1 = m_sweep.pos0;
	m_xf = GetTransform1(m_sweep);
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

inline body_count_t Body::GetIslandIndex() const noexcept
{
	return m_islandIndex;
}

inline bool IsValidIslandIndex(const Body& body) noexcept
{
	return body.IsInIsland() && (body.GetIslandIndex() != Body::InvalidIslandIndex);
}

/// Gets the total mass of the body.
/// @return Value of zero or more representing the body's mass (in kg).
/// @sa GetInverseMass.
/// @sa SetMassData.
inline float_t GetMass(const Body& body) noexcept
{
	return float_t{1} / body.GetInverseMass();
}

inline void ApplyLinearAcceleration(Body& body, const Vec2 amount)
{
	body.SetAcceleration(body.GetLinearAcceleration() + amount, body.GetAngularAcceleration());
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. Non-zero forces wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
inline void ApplyForce(Body& body, const Vec2& force, const Vec2& point) noexcept
{
	const auto linAccel = body.GetLinearAcceleration() + force * body.GetInverseMass();
	const auto angAccel = body.GetAngularAcceleration() + Cross(point - body.GetWorldCenter(), force) * body.GetInverseInertia();
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply a force to the center of mass. Non-zero forces wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
inline void ApplyForceToCenter(Body& body, const Vec2& force) noexcept
{
	const auto linAccel = body.GetLinearAcceleration() + force * body.GetInverseMass();
	const auto angAccel = body.GetAngularAcceleration();
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// Non-zero forces wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
inline void ApplyTorque(Body& body, float_t torque) noexcept
{
	const auto linAccel = body.GetLinearAcceleration();
	const auto angAccel = body.GetAngularAcceleration() + torque * body.GetInverseInertia();
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. Non-zero impulses wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
inline void ApplyLinearImpulse(Body& body, const Vec2& impulse, const Vec2& point) noexcept
{
	auto velocity = body.GetVelocity();
	velocity.v += body.GetInverseMass() * impulse;
	velocity.w += body.GetInverseInertia() * Cross(point - body.GetWorldCenter(), impulse);
	body.SetVelocity(velocity);
}

/// Apply an angular impulse.
/// @param body Body to apply the angular impulse to.
/// @param impulse the angular impulse in units of kg*m*m/s
inline void ApplyAngularImpulse(Body& body, float_t impulse) noexcept
{
	auto velocity = body.GetVelocity();
	velocity.w += body.GetInverseInertia() * impulse;
	body.SetVelocity(velocity);
}

/// Gets the rotational inertia of the body.
/// @return the rotational inertia, usually in kg-m^2.
inline float_t GetInertia(const Body& body) noexcept
{
	return float_t{1} / body.GetInverseInertia();
}

/// Gets the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
inline float_t GetLocalInertia(const Body& body) noexcept
{
	return GetInertia(body) + GetMass(body) * LengthSquared(body.GetLocalCenter());
}

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
inline MassData GetMassData(const Body& body) noexcept
{
	return MassData{GetMass(body), body.GetLocalCenter(), GetLocalInertia(body)};
}

/// Get the linear velocity of the center of mass.
/// @param body Body to get the linear velocity for.
/// @return the linear velocity of the center of mass.
inline Vec2 GetLinearVelocity(const Body& body) noexcept
{
	return body.GetVelocity().v;
}

/// Get the angular velocity.
/// @param body Body to get the angular velocity for.
/// @return the angular velocity in radians/second.
inline float_t GetAngularVelocity(const Body& body) noexcept
{
	return body.GetVelocity().w;
}

/// Set the linear velocity of the center of mass.
/// @param body Body to set the linear velocity of.
/// @param v the new linear velocity of the center of mass.
inline void SetLinearVelocity(Body& body, const Vec2& v) noexcept
{
	body.SetVelocity(Velocity{v, GetAngularVelocity(body)});
}

/// Set the angular velocity.
/// @param body Body to set the angular velocity of.
/// @param omega the new angular velocity in radians/second.
inline void SetAngularVelocity(Body& body, float_t omega) noexcept
{
	body.SetVelocity(Velocity{GetLinearVelocity(body), omega});
}

/// Gets the world coordinates of a point given in coordinates relative to the body's origin.
/// @param body Body that the given point is relative to.
/// @param localPoint a point measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
inline Vec2 GetWorldPoint(const Body& body, const Vec2& localPoint) noexcept
{
	return Transform(localPoint, body.GetTransformation());
}

/// Gets the world coordinates of a vector given the local coordinates.
/// @param body Body that the given vector is relative to.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
inline Vec2 GetWorldVector(const Body& body, const Vec2& localVector) noexcept
{
	return Rotate(localVector, body.GetTransformation().q);
}

/// Gets a local point relative to the body's origin given a world point.
/// @param body Body that the returned point should be relative to.
/// @param worldPoint point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
inline Vec2 GetLocalPoint(const Body& body, const Vec2& worldPoint) noexcept
{
	return InverseTransform(worldPoint, body.GetTransformation());
}

/// Gets a local vector given a world vector.
/// @param body Body that the returned vector should be relative to.
/// @param worldVector vector in world coordinates.
/// @return the corresponding local vector.
inline Vec2 GetLocalVector(const Body& body, const Vec2& worldVector) noexcept
{
	return InverseRotate(worldVector, body.GetTransformation().q);
}

/// Get the world linear velocity of a world point attached to this body.
/// @param worldPoint point in world coordinates.
/// @return the world velocity of a point.
inline Vec2 GetLinearVelocityFromWorldPoint(const Body& body, const Vec2& worldPoint) noexcept
{
	const auto velocity = body.GetVelocity();
	return velocity.v + GetReversePerpendicular(worldPoint - body.GetWorldCenter()) * velocity.w;
}

/// Get the world velocity of a local point.
/// @param localPoint point in local coordinates.
/// @return the world velocity of a point.
inline Vec2 GetLinearVelocityFromLocalPoint(const Body& body, const Vec2& localPoint) noexcept
{
	return GetLinearVelocityFromWorldPoint(body, GetWorldPoint(body, localPoint));
}

inline Vec2 GetForce(const Body& body) noexcept
{
	return body.GetLinearAcceleration() * GetMass(body);
}

inline float_t GetTorque(const Body& body) noexcept
{
	return body.GetAngularAcceleration() * GetInertia(body);;
}

/// Gets the velocity of this body after the given time with the given gravity.
Velocity GetVelocity(const Body& body, float_t h, Vec2 gravity) noexcept;
	
} // namespace box2d

#endif
