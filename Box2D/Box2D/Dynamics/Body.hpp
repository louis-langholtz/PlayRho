/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_BODY_H
#define B2_BODY_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Collision/MassData.hpp>
#include <Box2D/Dynamics/BodyType.hpp>

#include <forward_list>
#include <list>
#include <unordered_set>
#include <memory>
#include <cassert>

namespace box2d {

class Fixture;
class World;
struct FixtureDef;
class Shape;

const FixtureDef &GetDefaultFixtureDef() noexcept;

/// Body definition.
/// @detail
/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions.
struct BodyDef
{
	/// This constructor sets the body definition default values.
	constexpr BodyDef() = default;

	constexpr BodyDef& UseType(BodyType t) noexcept;
	constexpr BodyDef& UseLocation(Vec2 l) noexcept;
	constexpr BodyDef& UseAngle(Angle a) noexcept;
	constexpr BodyDef& UseLinearVelocity(Vec2 v) noexcept;
	constexpr BodyDef& UseAngularVelocity(AngularVelocity v) noexcept;
	constexpr BodyDef& UseLinearAcceleration(Vec2 v) noexcept;
	constexpr BodyDef& UseAngularAcceleration(Angle v) noexcept;
	constexpr BodyDef& UseLinearDamping(RealNum v) noexcept;
	constexpr BodyDef& UseAngularDamping(RealNum v) noexcept;
	constexpr BodyDef& UseUnderActiveTime(Time v) noexcept;
	constexpr BodyDef& UseAllowSleep(bool value) noexcept;
	constexpr BodyDef& UseAwake(bool value) noexcept;
	constexpr BodyDef& UseFixedRotation(bool value) noexcept;
	constexpr BodyDef& UseBullet(bool value) noexcept;
	constexpr BodyDef& UseEnabled(bool value) noexcept;
	constexpr BodyDef& UseUserData(void* value) noexcept;
	
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	BodyType type = BodyType::Static;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	Vec2 position = Vec2_zero;

	/// The world angle of the body in radians.
	Angle angle = Angle{0};

	/// The linear velocity of the body's origin in world co-ordinates (in m/s).
	Vec2 linearVelocity = Vec2_zero;

	/// The angular velocity of the body.
	AngularVelocity angularVelocity = AngularVelocity{0};

	Vec2 linearAcceleration = Vec2_zero;
	
	Angle angularAcceleration = Angle{0};
	
	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	RealNum linearDamping = 0;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	RealNum angularDamping = 0;

	/// Under-active time.
	/// @detail Set this to the value retrieved from Body::GetUnderActiveTime() or leave it as 0.
	Time underActiveTime = Second * RealNum{0};

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

	/// Does this body start out enabled?
	bool enabled = true;

	/// Use this to store application specific body data.
	void* userData = nullptr;
};

constexpr inline BodyDef& BodyDef::UseType(BodyType t) noexcept
{
	type = t;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseLocation(Vec2 l) noexcept
{
	position = l;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseAngle(Angle a) noexcept
{
	angle = a;
	return *this;
}

constexpr BodyDef& BodyDef::UseLinearVelocity(Vec2 v) noexcept
{
	linearVelocity = v;
	return *this;
}

constexpr BodyDef& BodyDef::UseLinearAcceleration(Vec2 v) noexcept
{
	linearAcceleration = v;
	return *this;
}

constexpr BodyDef& BodyDef::UseAngularVelocity(AngularVelocity v) noexcept
{
	angularVelocity = v;
	return *this;
}

constexpr BodyDef& BodyDef::UseAngularAcceleration(Angle v) noexcept
{
	angularAcceleration = v;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseLinearDamping(RealNum v) noexcept
{
	linearDamping = v;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseAngularDamping(RealNum v) noexcept
{
	angularDamping = v;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseUnderActiveTime(Time v) noexcept
{
	underActiveTime = v;
	return *this;
}
	
constexpr inline BodyDef& BodyDef::UseAllowSleep(bool value) noexcept
{
	allowSleep = value;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseAwake(bool value) noexcept
{
	awake = value;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseFixedRotation(bool value) noexcept
{
	fixedRotation = value;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseBullet(bool value) noexcept
{
	bullet = value;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseEnabled(bool value) noexcept
{
	enabled = value;
	return *this;
}

constexpr inline BodyDef& BodyDef::UseUserData(void* value) noexcept
{
	userData = value;
	return *this;
}

/// Body.
///
/// @detail A rigid body entity created or destroyed through a World instance.
///
/// @invariant Only bodies that allow sleeping, can be put to sleep.
/// @invariant Only "speedable" bodies can be awake.
/// @invariant Only "speedable" bodies can have non-zero velocities.
/// @invariant Only "accelerable" bodies can have non-zero accelerations.
/// @invariant Only "accelerable" bodies can have non-zero "under-active" times.
///
/// @note Create these using the World::Create method.
/// @note On a 64-bit architecture with 4-byte RealNum, this data structure is at least 192-bytes large.
///
class Body
{
public:
	using Fixtures = std::forward_list<Fixture*>;
	
	using Joints = std::unordered_set<Joint*>;
	
	using Contacts = std::list<Contact*>;
	
	static constexpr auto InvalidIslandIndex = static_cast<body_count_t>(-1);
	
	static bool IsValid(const Shape& shape);

	/// Creates a fixture and attaches it to this body.
	///
	/// @param shape Sharable shape definition.
	///   Its vertex radius must be less than the minimum or more than the maximum allowed by
	///   the body's world.
	/// @param def Initial fixture settings.
	///   Friction and density must be >= 0.
	///   Restitution must be > -infinity and < infinity.
	/// @param resetMassData Whether or not to reset the mass data of the body.
	///
	/// @note This function should not be called if the world is locked.
	/// @warning This function is locked during callbacks.
	///
	/// @return <code>nullptr</code> if the world is locked or a parameter is dissallowed.
	///   A pointer to the created fixture otherwise.
	///
	Fixture* CreateFixture(std::shared_ptr<const Shape> shape,
						   const FixtureDef& def = GetDefaultFixtureDef(),
						   bool resetMassData = true);
	
	/// Destroys a fixture.
	/// @detail This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @warning This function is locked during callbacks.
	/// @note Make sure to explicitly call ResetMassData after fixtures have been destroyed.
	/// @sa ResetMassData.
	/// @param fixture the fixture to be removed.
	bool DestroyFixture(Fixture* fixture, bool resetMassData = true);
	
	/// Sets the position of the body's origin and rotation.
	/// @detail This instantly adjusts the body to be at the new position and new orientation.
	/// @warning Manipulating a body's transform may cause non-physical behavior.
	/// @note Contacts are updated on the next call to World::Step.
	/// @param position Valid world position of the body's local origin. Behavior is undefined if value is invalid.
	/// @param angle Valid world rotation in radians. Behavior is undefined if value is invalid.
	void SetTransform(const Vec2 position, Angle angle);

	/// Gets the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	Transformation GetTransformation() const noexcept;

	/// Gets the world body origin location.
	/// @detail This is the location of the body's origin relative to its world.
	/// The location of the body after stepping the world's physics simulations is dependent on a number of factors:
	///   1. Location at the last time step.
	///   2. Forces acting on the body (gravity, applied force, applied impulse).
	///   3. The mass data of the body.
	///   4. Damping of the body.
	///   5. Restitution and friction values of the body's fixtures when they experience collisions.
	/// @return World location of the body's origin.
	Vec2 GetLocation() const noexcept;

	const Sweep& GetSweep() const noexcept;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	Angle GetAngle() const noexcept;

	/// Get the world position of the center of mass.
	Vec2 GetWorldCenter() const noexcept;

	/// Gets the local position of the center of mass.
	Vec2 GetLocalCenter() const noexcept;

	Velocity GetVelocity() const noexcept;

	/// Sets the body's velocity (linear and angular velocity).
	/// @note This method does nothing if this body is not speedable.
	/// @note A non-zero velocity will awaken this body.
	/// @sa SetAwake.
	/// @sa SetUnderActiveTime.
	void SetVelocity(const Velocity& v) noexcept;

	/// Sets the linear and rotational accelerations on this body.
	/// @note This has no effect on non-accelerable bodies.
	/// @note A non-zero acceleration will also awaken the body.
	/// @param linear Linear acceleration.
	/// @param angular Angular acceleration.
	void SetAcceleration(const Vec2 linear, const Angle angular) noexcept;

	Vec2 GetLinearAcceleration() const noexcept;

	Angle GetAngularAcceleration() const noexcept;

	/// Gets the inverse total mass of the body.
	/// @detail This is the cached result of dividing 1 by the body's mass.
	/// Often floating division is much slower than multiplication.
	/// As such, it's likely faster to multiply values by this inverse value than to redivide
	/// them all the time by the mass.
	/// @return Value of zero or more representing the body's inverse mass (in 1/kg).
	/// @sa SetMassData.
	InvMass GetInvMass() const noexcept;
	
	/// Gets the inverse rotational inertia of the body.
	/// @detail This is the cached result of dividing 1 by the body's rotational inertia.
	/// Often floating division is much slower than multiplication.
	/// As such, it's likely faster to multiply values by this inverse value than to redivide
	/// them all the time by the rotational inertia.
	/// @return Inverse rotational intertia (in 1/kg-m^2).
	InvRotInertia GetInvRotInertia() const noexcept;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// @note This changes the center of mass position.
	/// @note Creating or destroying fixtures can also alter the mass.
	/// @note This function has no effect if the body isn't dynamic.
	/// @param data the mass properties.
	void SetMassData(const MassData& data);

	/// Resets the mass data properties.
	/// @detail This resets the mass data to the sum of the mass properties of the fixtures.
	/// @note This method must be called after calling <code>CreateFixture</code> to update the
	///   body mass data properties unless <code>SetMassData</code> is used.
	/// @sa SetMassData.
	void ResetMassData();

	/// Get the linear damping of the body.
	RealNum GetLinearDamping() const noexcept;

	/// Set the linear damping of the body.
	void SetLinearDamping(RealNum linearDamping) noexcept;

	/// Get the angular damping of the body.
	RealNum GetAngularDamping() const noexcept;

	/// Set the angular damping of the body.
	void SetAngularDamping(RealNum angularDamping) noexcept;

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
	
	/// Sets this body to awake if it's a "speedable" body.
	/// @detail
	/// This sets the sleep state of the body to awake if it's a "speedable" body. It has
	/// no effect otherwise.
	/// @post If this body is a "speedable" body, then this body's IsAwake method returns true.
	void SetAwake() noexcept;

	/// Sets this body to asleep if sleeping is allowed.
	/// @detail
	/// If this body is allowed to sleep, this: sets the sleep state of the body to asleep,
	/// resets this body's under active time, and resets this body's velocity (linear and angular).
	/// @post This body's IsAwake method returns false.
	/// @post This body's GetUnderActiveTime method returns zero.
	/// @post This body's GetVelocity method returns zero linear and zero angular speed.
	void UnsetAwake() noexcept;
	
	/// Gets the awake/asleep state of this body.
	/// @warning Being awake may or may not imply being speedable.
	/// @return true if the body is awake.
	bool IsAwake() const noexcept;

	/// Gets this body's under-active time value.
	/// @return Zero or more time in seconds (of step time) that this body has been "under-active" for.
	Time GetUnderActiveTime() const noexcept;
	
	/// Sets the "under-active" time to the given value.
	///
	/// @detail Sets the "under-active" time to a value of zero or a non-zero value if the
	///   body is "accelerable". Otherwise it does nothing.
	///
	/// @warning Behavior is undefined for negative values.
	/// @note A non-zero time is only valid for an "accelerable" body.
	///
	void SetUnderActiveTime(Time value) noexcept;
	
	/// Set the enabled state of the body. A disabled body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on disabled bodies.
	/// Fixtures on a disabled body are implicitly disabled and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to a disabled body are implicitly disabled.
	/// A disabled body is still owned by a World object and remains
	/// in the body list.
	void SetEnabled(bool flag);

	/// Get the enabled/disabled state of the body.
	bool IsEnabled() const noexcept;

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(bool flag);

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const noexcept;
	
	/// Gets the container of all fixtures attached to this body.
	const Fixtures& GetFixtures() const noexcept;
	
	/// Gets the container of all joints attached to this body.
	const Joints& GetJoints() const noexcept;

	/// Gets the container of all contacts attached to this body.
	/// @warning This list changes during the time step and you may
	/// miss some collisions if you don't use ContactListener.
	const Contacts& GetContacts() const noexcept;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const noexcept;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) noexcept;

	/// Gets the parent world of this body.
	World* GetWorld() noexcept;

	/// Gets the parent world of this body.
	const World* GetWorld() const noexcept;
	
	bool IsMassDataDirty() const noexcept;
		
private:

	friend class BodyAtty;
	
	using FlagsType = uint16;

	// m_flags
	enum Flag: FlagsType
	{
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
		
		/// Enabled flag.
		e_enabledFlag		= 0x0020,
		
		/// Velocity flag.
		/// @detail Set this to enable changes in position due to velocity.
		/// Bodies with this set are "speedable" - either kinematic or dynamic bodies.
		e_velocityFlag      = 0x0080,

		/// Acceleration flag.
		/// @detail Set this to enable changes in velocity due to physical properties (like forces).
		/// Bodies with this set are "accelerable" - dynamic bodies.
		e_accelerationFlag  = 0x0100,
		
		/// Mass Data Dirty Flag.
		e_massDataDirtyFlag	= 0x0200,
	};
	
	static FlagsType GetFlags(const BodyType type) noexcept;
	static FlagsType GetFlags(const BodyDef& bd) noexcept;

	Body(const BodyDef& bd, World* world);
	~Body();

	void SetAwakeFlag() noexcept;
	void UnsetAwakeFlag() noexcept;

	/// Advances the body by a given time ratio.
	/// @detail This method:
	///    1. advances the body's sweep to the given time ratio;
	///    2. updates the body's sweep positions (linear and angular) to the advanced ones; and
	///    3. updates the body's transform to the new sweep one settings.
	/// @param t Valid new time factor in [0,1) to advance the sweep to.
	void Advance(RealNum t) noexcept;

	void SetMassDataDirty() noexcept;
	void UnsetMassDataDirty() noexcept;
	
	void SetEnabledFlag() noexcept;
	void UnsetEnabledFlag() noexcept;

	bool Insert(Contact* contact);
	bool Insert(Joint* joint);
	
	bool Erase(Contact* const contact);
	bool Erase(Joint* const joint);
	bool Erase(Fixture* const fixture);

	void SetTransformation(const Transformation value) noexcept;
		
	//
	// Member variables. Try to keep total size small.
	//

	/// Transformation for body origin.
	/// @detail
	/// This is essentially the cached result of <code>GetTransform1(m_sweep)</code>. 16-bytes.
	Transformation m_xf;

	Sweep m_sweep; ///< Sweep motion for CCD. 36-bytes.

	Velocity m_velocity; ///< Velocity (linear and angular). 12-bytes.
	FlagsType m_flags = 0; ///< Flags. 2-bytes.
	
	Vec2 m_linearAcceleration = Vec2_zero; ///< Linear acceleration. 8-bytes.

	World* const m_world; ///< World to which this body belongs. 8-bytes.
	void* m_userData; ///< User data. 8-bytes.
	
	Fixtures m_fixtures; ///< Container of fixtures. 8-bytes.
	Contacts m_contacts; ///< Container of contacts. 8-bytes.
	Joints m_joints; ///< Container of joints. 8-bytes.
	
	Angle m_angularAcceleration = Angle{0}; ///< Angular acceleration. 4-bytes.
	
	/// Inverse mass of the body.
	/// @detail A non-negative value (in units of 1/kg).
	/// Can only be zero for non-accelerable bodies.
	/// @note 4-bytes.
	InvMass m_invMass = 0;
	
	/// Inverse rotational inertia about the center of mass.
	/// @detail A non-negative value (in units of 1/(kg*m^2)).
	/// @note 4-bytes.
	InvRotInertia m_invRotI = 0;

	RealNum m_linearDamping; ///< Linear damping. 4-bytes.
	RealNum m_angularDamping; ///< Angular damping. 4-bytes.

	/// Under-active time.
	/// @detail A body under-active for enough time should have their awake flag unset.
	///   I.e. if a body is under-active for long enough, it should go to sleep.
	/// @note 4-bytes.
	Time m_underActiveTime = 0;
};

inline Body::FlagsType Body::GetFlags(const BodyType type) noexcept
{
	auto flags = FlagsType{0};
	switch (type)
	{
		case BodyType::Dynamic:   flags |= (e_velocityFlag|e_accelerationFlag); break;
		case BodyType::Kinematic: flags |= (e_impenetrableFlag|e_velocityFlag); break;
		case BodyType::Static:    flags |= (e_impenetrableFlag); break;
	}
	return flags;
}

inline bool Body::Insert(Contact* c)
{
#ifndef NDEBUG
	// Prevent the same contact from being added more than once...
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
	{
		assert(*iter != c);
		if (*iter == c)
		{
			return false;
		}
	}
#endif
	m_contacts.push_back(c);
	return true;
}

inline bool Body::Insert(Joint* j)
{
	const auto results = m_joints.insert(j);
	return results.second;
}

inline bool Body::Erase(Contact* const contact)
{
	for (auto iter = m_contacts.begin(); iter != m_contacts.end(); ++iter)
	{
		if (*iter == contact)
		{
			m_contacts.erase(iter);
			return true;
		}
	}
	return false;
}

inline bool Body::Erase(Joint* const joint)
{
	return m_joints.erase(joint) > 0;
}

inline bool Body::Erase(Fixture* const fixture)
{
	auto prev = m_fixtures.before_begin();
	for (auto iter = m_fixtures.begin(); iter != m_fixtures.end(); ++iter)
	{
		if (*iter == fixture)
		{
			m_fixtures.erase_after(prev);
			return true;
		}
		prev = iter;
	}
	return false;
}

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

inline Vec2 Body::GetLocation() const noexcept
{
	return GetTransformation().p;
}

inline const Sweep& Body::GetSweep() const noexcept
{
	return m_sweep;
}
	
inline Angle Body::GetAngle() const noexcept
{
	return GetSweep().pos1.angular;
}

inline Vec2 Body::GetWorldCenter() const noexcept
{
	return GetSweep().pos1.linear;
}

inline Vec2 Body::GetLocalCenter() const noexcept
{
	return GetSweep().GetLocalCenter();
}

inline Velocity Body::GetVelocity() const noexcept
{
	return m_velocity;
}
	
inline InvMass Body::GetInvMass() const noexcept
{
	return m_invMass;
}

inline InvRotInertia Body::GetInvRotInertia() const noexcept
{
	return m_invRotI;
}

inline RealNum Body::GetLinearDamping() const noexcept
{
	return m_linearDamping;
}

inline void Body::SetLinearDamping(RealNum linearDamping) noexcept
{
	m_linearDamping = linearDamping;
}

inline RealNum Body::GetAngularDamping() const noexcept
{
	return m_angularDamping;
}

inline void Body::SetAngularDamping(RealNum angularDamping) noexcept
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

inline void Body::SetAwakeFlag() noexcept
{
	assert(IsSpeedable());
	m_flags |= e_awakeFlag;
}

inline void Body::UnsetAwakeFlag() noexcept
{
	assert(!IsSpeedable() || IsSleepingAllowed());
	m_flags &= ~e_awakeFlag;
}

inline void Body::SetAwake() noexcept
{
	if (IsSpeedable())
	{
		// Note: DO NOT reset m_underActiveTime for a callers to this method.
		SetAwakeFlag();
	}
}

inline void Body::UnsetAwake() noexcept
{
	if (!IsSpeedable() || IsSleepingAllowed())
	{
		UnsetAwakeFlag();
		m_underActiveTime = 0;
		m_velocity = Velocity{Vec2_zero, AngularVelocity{0}};
	}
}

inline bool Body::IsAwake() const noexcept
{
	return (m_flags & e_awakeFlag) != 0;
}

inline Time Body::GetUnderActiveTime() const noexcept
{
	return m_underActiveTime;
}

inline void Body::SetUnderActiveTime(Time value) noexcept
{
	if ((value == Second * RealNum{0}) || IsAccelerable())
	{
		m_underActiveTime = value;
	}
}

inline bool Body::IsEnabled() const noexcept
{
	return (m_flags & e_enabledFlag) != 0;
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
	else if (IsSpeedable())
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake();
	}
}

inline bool Body::IsSleepingAllowed() const noexcept
{
	return (m_flags & e_autoSleepFlag) != 0;
}

inline const Body::Fixtures& Body::GetFixtures() const noexcept
{
	return m_fixtures;
}

inline const Body::Joints& Body::GetJoints() const noexcept
{
	return m_joints;
}

inline const Body::Contacts& Body::GetContacts() const noexcept
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

inline Angle Body::GetAngularAcceleration() const noexcept
{
	return m_angularAcceleration;
}

inline void Body::Advance(RealNum alpha) noexcept
{
	//assert(m_sweep.GetAlpha0() <= alpha);
	assert(IsSpeedable() || m_sweep.pos1 == m_sweep.pos0);

	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance0(alpha);
	m_sweep.pos1 = m_sweep.pos0;
	SetTransformation(GetTransform1(m_sweep));
}

inline World* Body::GetWorld() noexcept
{
	return m_world;
}

inline const World* Body::GetWorld() const noexcept
{
	return m_world;
}

inline void Body::SetMassDataDirty() noexcept
{
	m_flags |= e_massDataDirtyFlag;
}

inline void Body::UnsetMassDataDirty() noexcept
{
	m_flags &= ~e_massDataDirtyFlag;
}

inline bool Body::IsMassDataDirty() const noexcept
{
	return m_flags & e_massDataDirtyFlag;
}

inline void Body::SetTransformation(const Transformation value) noexcept
{
	m_xf = value;
}

inline void Body::SetEnabledFlag() noexcept
{
	m_flags |= e_enabledFlag;
}

inline void Body::UnsetEnabledFlag() noexcept
{
	m_flags &= ~e_enabledFlag;
}

// Free functions...

/// Awakens the body if it's asleep.
inline bool Awaken(Body& body) noexcept
{
	if (!body.IsAwake() && body.IsSpeedable())
	{
		body.SetAwake();
		return true;
	}
	return false;
}

/// Puts the body to sleep if it's awake.
inline bool Unawaken(Body& body) noexcept
{
	if (body.IsAwake() && body.IsSleepingAllowed())
	{
		body.UnsetAwake();
		return true;
	}
	return false;
}

/// Should collide.
/// @detail Determines whether a body should possibly be able to collide with the other body.
/// @return true if either body is dynamic and no joint prevents collision, false otherwise.
bool ShouldCollide(const Body& lhs, const Body& rhs) noexcept;

void DestroyFixtures(Body& body);
	
inline Position GetPosition1(const Body& body) noexcept
{
	return body.GetSweep().pos1;
}

/// Gets the total mass of the body.
/// @return Value of zero or more representing the body's mass (in kg).
/// @sa GetInvMass.
/// @sa SetMassData.
inline Mass GetMass(const Body& body) noexcept
{
	const auto invMass = body.GetInvMass();
	return (invMass != InvMass{0})? Mass{RealNum{1} / invMass}: Mass{0};
}

inline void ApplyLinearAcceleration(Body& body, const Vec2 amount)
{
	body.SetAcceleration(body.GetLinearAcceleration() + amount, body.GetAngularAcceleration());
}

inline void SetForce(Body& body, const Vec2 force, const Vec2 point) noexcept
{
	const auto linAccel = force * RealNum{body.GetInvMass() * Kilogram};
	const auto invRotI = body.GetInvRotInertia();
 	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	const auto angAccel = Radian * Cross(point - body.GetWorldCenter(), force) * intRotInertiaUnitless;
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply a force at a world point.
/// @note If the force is not applied at the center of mass, it will generate a torque and
///   affect the angular velocity.
/// @note Non-zero forces wakes up the body.
/// @param force World force vector, usually in Newtons (N).
/// @param point World position of the point of application.
inline void ApplyForce(Body& body, const Vec2 force, const Vec2 point) noexcept
{
	const auto linAccel = force * RealNum{body.GetInvMass() * Kilogram};
	const auto invRotI = body.GetInvRotInertia();
	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	const auto angAccel = Radian * Cross(point - body.GetWorldCenter(), force) * intRotInertiaUnitless;
	body.SetAcceleration(body.GetLinearAcceleration() + linAccel, body.GetAngularAcceleration() + angAccel);
}

/// Apply a force to the center of mass.
/// @note Non-zero forces wakes up the body.
/// @param force World force vector, usually in Newtons (N).
inline void ApplyForceToCenter(Body& body, const Vec2 force) noexcept
{
	const auto linAccel = body.GetLinearAcceleration() + force * RealNum{body.GetInvMass() * Kilogram};
	const auto angAccel = body.GetAngularAcceleration();
	body.SetAcceleration(linAccel, angAccel);
}

inline void SetTorque(Body& body, const Torque torque) noexcept
{
	const auto linAccel = body.GetLinearAcceleration();
	const auto invRotI = body.GetInvRotInertia();
	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	const auto angAccel = RealNum{torque / NewtonMeter} * intRotInertiaUnitless * Radian;
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply a torque.
/// @note This affects the angular velocity without affecting the linear velocity of the center of mass.
/// @note Non-zero forces wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
inline void ApplyTorque(Body& body, const Torque torque) noexcept
{
	const auto linAccel = body.GetLinearAcceleration();
	const auto invRotI = body.GetInvRotInertia();
	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	const auto angAccel = body.GetAngularAcceleration() + RealNum{torque / NewtonMeter} * intRotInertiaUnitless * Radian;
	body.SetAcceleration(linAccel, angAccel);
}

/// Apply an impulse at a point.
/// @note This immediately modifies the velocity.
/// @note This also modifies the angular velocity if the point of application
///   is not at the center of mass.
/// @note Non-zero impulses wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
inline void ApplyLinearImpulse(Body& body, const Vec2 impulse, const Vec2 point) noexcept
{
	auto velocity = body.GetVelocity();
	velocity.linear += RealNum{body.GetInvMass() * Kilogram} * impulse;
	const auto invRotI = body.GetInvRotInertia();
	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	velocity.angular += RadianPerSecond * intRotInertiaUnitless * Cross(point - body.GetWorldCenter(), impulse);
	body.SetVelocity(velocity);
}

/// Apply an angular impulse.
/// @param body Body to apply the angular impulse to.
/// @param impulse the angular impulse in units of kg*m*m/s
inline void ApplyAngularImpulse(Body& body, RealNum impulse) noexcept
{
	auto velocity = body.GetVelocity();
	const auto invRotI = body.GetInvRotInertia();
	const auto intRotInertiaUnitless = invRotI * (SquareMeter * Kilogram / SquareRadian);
	velocity.angular += RadianPerSecond * intRotInertiaUnitless * impulse;
	body.SetVelocity(velocity);
}

Vec2 GetCentripetalForce(const Body& body, const Vec2 axis);

/// Gets the rotational inertia of the body.
/// @return the rotational inertia, usually in kg-m^2.
inline RotInertia GetRotInertia(const Body& body) noexcept
{
	return RealNum{1} / body.GetInvRotInertia();
}

/// Gets the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
inline RotInertia GetLocalInertia(const Body& body) noexcept
{
	return GetRotInertia(body) + GetMass(body) * GetLengthSquared(body.GetLocalCenter()) * SquareMeter / SquareRadian;
}

/// Gets the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
inline MassData GetMassData(const Body& body) noexcept
{
	const auto I = GetLocalInertia(body);
	return MassData{GetMass(body), body.GetLocalCenter(), I};
}

/// Gets the linear velocity of the center of mass.
/// @param body Body to get the linear velocity for.
/// @return the linear velocity of the center of mass.
inline Vec2 GetLinearVelocity(const Body& body) noexcept
{
	return body.GetVelocity().linear;
}

/// Gets the angular velocity.
/// @param body Body to get the angular velocity for.
/// @return the angular velocity in radians/second.
inline AngularVelocity GetAngularVelocity(const Body& body) noexcept
{
	return body.GetVelocity().angular;
}

/// Sets the linear velocity of the center of mass.
/// @param body Body to set the linear velocity of.
/// @param v the new linear velocity of the center of mass.
inline void SetLinearVelocity(Body& body, const Vec2 v) noexcept
{
	body.SetVelocity(Velocity{v, GetAngularVelocity(body)});
}

/// Sets the angular velocity.
/// @param body Body to set the angular velocity of.
/// @param omega the new angular velocity in radians/second.
inline void SetAngularVelocity(Body& body, AngularVelocity omega) noexcept
{
	body.SetVelocity(Velocity{GetLinearVelocity(body), omega});
}

/// Gets the world coordinates of a point given in coordinates relative to the body's origin.
/// @param body Body that the given point is relative to.
/// @param localPoint a point measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
inline Vec2 GetWorldPoint(const Body& body, const Vec2 localPoint) noexcept
{
	return Transform(localPoint, body.GetTransformation());
}

/// Gets the world coordinates of a vector given the local coordinates.
/// @param body Body that the given vector is relative to.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
inline Vec2 GetWorldVector(const Body& body, const Vec2 localVector) noexcept
{
	return Rotate(localVector, body.GetTransformation().q);
}

/// Gets a local point relative to the body's origin given a world point.
/// @param body Body that the returned point should be relative to.
/// @param worldPoint point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
inline Vec2 GetLocalPoint(const Body& body, const Vec2 worldPoint) noexcept
{
	return InverseTransform(worldPoint, body.GetTransformation());
}

/// Gets a local vector given a world vector.
/// @param body Body that the returned vector should be relative to.
/// @param worldVector vector in world coordinates.
/// @return the corresponding local vector.
inline Vec2 GetLocalVector(const Body& body, const Vec2 worldVector) noexcept
{
	return InverseRotate(worldVector, body.GetTransformation().q);
}

/// Get the world linear velocity of a world point attached to this body.
/// @param worldPoint point in world coordinates.
/// @return the world velocity of a point.
inline Vec2 GetLinearVelocityFromWorldPoint(const Body& body, const Vec2 worldPoint) noexcept
{
	const auto velocity = body.GetVelocity();
	const auto worldCtr = body.GetWorldCenter();
	return velocity.linear + GetRevPerpendicular(worldPoint - worldCtr) * RealNum{velocity.angular / RadianPerSecond};
}

/// Get the world velocity of a local point.
/// @param localPoint point in local coordinates.
/// @return the world velocity of a point.
inline Vec2 GetLinearVelocityFromLocalPoint(const Body& body, const Vec2 localPoint) noexcept
{
	return GetLinearVelocityFromWorldPoint(body, GetWorldPoint(body, localPoint));
}

inline Vec2 GetForce(const Body& body) noexcept
{
	return body.GetLinearAcceleration() * (GetMass(body) / Kilogram);
}

inline Torque GetTorque(const Body& body) noexcept
{
	return body.GetAngularAcceleration() * GetRotInertia(body) / (Second * Second);
}

/// Gets the velocity of the body after the given time accounting for the body's acceleration.
/// @warning Behavior is undefined if the given elapsed time is an invalid value (like NaN).
/// @param body Body to get the velocity for.
/// @param h Time elapsed to get velocity for. Behavior is undefined if this value is invalid.
Velocity GetVelocity(const Body& body, Time h) noexcept;

size_t GetWorldIndex(const Body* body);

size_t GetFixtureCount(const Body& body);

/// Computes the body's mass data.
/// @detail This basically accumulates the mass data over all fixtures.
/// @note The center is the mass weighted sum of all fixture centers. Divide it by the
///   mass to get the averaged center.
/// @return accumalated mass data for all fixtures associated with the given body.
MassData ComputeMassData(const Body& body) noexcept;

/// Rotates a body a given amount around a point in world coordinates.
/// @detail This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param worldPoint Point in world coordinates.
void RotateAboutWorldPoint(Body& body, Angle amount, Vec2 worldPoint);

/// Rotates a body a given amount around a point in body local coordinates.
/// @detail This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @note This is a convenience function that translates the local point into world coordinates
///   and then calls the <code>RotateAboutWorldPoint</code> function.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param localPoint Point in local coordinates.
void RotateAboutLocalPoint(Body& body, Angle amount, Vec2 localPoint);

} // namespace box2d

#endif
