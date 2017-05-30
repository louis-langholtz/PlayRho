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

/// @file
/// Declarations of the Body class, and free functions associated with it.

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Range.hpp>
#include <Box2D/Common/BoundedValue.hpp>
#include <Box2D/Dynamics/BodyType.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/Contacts/ContactKey.hpp>
#include <Box2D/Dynamics/Joints/JointKey.hpp>

#include <list>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <cassert>
#include <utility>

namespace box2d {

class World;
struct FixtureDef;
class Shape;
struct BodyDef;
struct MassData;

const FixtureDef &GetDefaultFixtureDef() noexcept;

/// @brief A physical entity that exists within a World.
///
/// @details A rigid body entity created or destroyed through a World instance.
///
/// @invariant Only bodies that allow sleeping, can be put to sleep.
/// @invariant Only "speedable" bodies can be awake.
/// @invariant Only "speedable" bodies can have non-zero velocities.
/// @invariant Only "accelerable" bodies can have non-zero accelerations.
/// @invariant Only "accelerable" bodies can have non-zero "under-active" times.
///
/// @note Create these using the World::Create method.
/// @note From a memory management perspective, bodies own Fixture instances.
/// @note On a 64-bit architecture with 4-byte RealNum, this data structure is at least
///   192-bytes large.
///
class Body
{
public:
    
    /// @brief Container type for fixtures.
    using Fixtures = std::list<Fixture>;

    /// @brief Container type for joints.
    using Joints = std::vector<std::pair<Body*, Joint*>>;
    
    /// @brief Container type for contacts.
    using Contacts = std::vector<std::pair<ContactKey,Contact*>>;

    static constexpr auto InvalidIslandIndex = static_cast<body_count_t>(-1);

    /// @brief Initializing constructor.
    /// @note This is not meant to be called directly by users of the library API. Call
    ///   a world instance's <code>CreateBody</code> method instead.
    Body(const BodyDef& bd, World* world);

    ~Body();
    
    /// @brief Creates a fixture and attaches it to this body.
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

    /// @brief Destroys a fixture.
    ///
    /// @details This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    ///
    /// @warning This function is locked during callbacks.
    /// @note Make sure to explicitly call ResetMassData after fixtures have been destroyed.
    /// @sa ResetMassData.
    ///
    /// @param fixture the fixture to be removed.
    /// @param resetMassData Whether or not to reset the mass data.
    ///
    bool DestroyFixture(Fixture* fixture, bool resetMassData = true);
    
    void DestroyFixtures();
    
    /// @brief Sets the position of the body's origin and rotation.
    /// @details This instantly adjusts the body to be at the new position and new orientation.
    /// @warning Manipulating a body's transform can cause non-physical behavior!
    /// @note Contacts are updated on the next call to World::Step.
    /// @param position Valid world position of the body's local origin. Behavior is undefined
    ///   if value is invalid.
    /// @param angle Valid world rotation. Behavior is undefined if value is invalid.
    void SetTransform(const Length2D position, Angle angle);

    /// @brief Gets the body transform for the body's origin.
    /// @return the world transform of the body's origin.
    Transformation GetTransformation() const noexcept;

    /// @brief Gets the world body origin location.
    /// @details This is the location of the body's origin relative to its world.
    /// The location of the body after stepping the world's physics simulations is dependent on
    /// a number of factors:
    ///   1. Location at the last time step.
    ///   2. Forces acting on the body (gravity, applied force, applied impulse).
    ///   3. The mass data of the body.
    ///   4. Damping of the body.
    ///   5. Restitution and friction values of the body's fixtures when they experience collisions.
    /// @return World location of the body's origin.
    Length2D GetLocation() const noexcept;

    const Sweep& GetSweep() const noexcept;

    /// @brief Get the angle.
    /// @return the current world rotation angle.
    Angle GetAngle() const noexcept;

    /// @brief Get the world position of the center of mass.
    Length2D GetWorldCenter() const noexcept;

    /// @brief Gets the local position of the center of mass.
    Length2D GetLocalCenter() const noexcept;

    Velocity GetVelocity() const noexcept;

    /// @brief Sets the body's velocity (linear and angular velocity).
    /// @note This method does nothing if this body is not speedable.
    /// @note A non-zero velocity will awaken this body.
    /// @sa SetAwake.
    /// @sa SetUnderActiveTime.
    void SetVelocity(const Velocity& v) noexcept;

    /// @brief Sets the linear and rotational accelerations on this body.
    /// @note This has no effect on non-accelerable bodies.
    /// @note A non-zero acceleration will also awaken the body.
    /// @param linear Linear acceleration.
    /// @param angular Angular acceleration.
    void SetAcceleration(const LinearAcceleration2D linear,
                         const AngularAcceleration angular) noexcept;

    /// @brief Gets this body's linear acceleration.
    LinearAcceleration2D GetLinearAcceleration() const noexcept;

    /// @brief Gets this body's angular acceleration.
    AngularAcceleration GetAngularAcceleration() const noexcept;

    /// @brief Gets the inverse total mass of the body.
    /// @details This is the cached result of dividing 1 by the body's mass.
    /// Often floating division is much slower than multiplication.
    /// As such, it's likely faster to multiply values by this inverse value than to redivide
    /// them all the time by the mass.
    /// @return Value of zero or more representing the body's inverse mass (in 1/kg).
    /// @sa SetMassData.
    InvMass GetInvMass() const noexcept;

    /// @brief Gets the inverse rotational inertia of the body.
    /// @details This is the cached result of dividing 1 by the body's rotational inertia.
    /// Often floating division is much slower than multiplication.
    /// As such, it's likely faster to multiply values by this inverse value than to redivide
    /// them all the time by the rotational inertia.
    /// @return Inverse rotational intertia (in 1/kg-m^2).
    InvRotInertia GetInvRotInertia() const noexcept;

    /// @brief Set the mass properties to override the mass properties of the fixtures.
    /// @note This changes the center of mass position.
    /// @note Creating or destroying fixtures can also alter the mass.
    /// @note This function has no effect if the body isn't dynamic.
    /// @param data the mass properties.
    void SetMassData(const MassData& data);

    /// @brief Resets the mass data properties.
    /// @details This resets the mass data to the sum of the mass properties of the fixtures.
    /// @note This method must be called after calling <code>CreateFixture</code> to update the
    ///   body mass data properties unless <code>SetMassData</code> is used.
    /// @sa SetMassData.
    void ResetMassData();

    /// @brief Gets the linear damping of the body.
    Frequency GetLinearDamping() const noexcept;

    /// @brief Sets the linear damping of the body.
    void SetLinearDamping(NonNegative<Frequency> linearDamping) noexcept;

    /// @brief Gets the angular damping of the body.
    Frequency GetAngularDamping() const noexcept;

    /// @brief Sets the angular damping of the body.
    void SetAngularDamping(NonNegative<Frequency> angularDamping) noexcept;

    /// @brief Sets the type of this body.
    /// @note This may alter the mass and velocity.
    void SetType(BodyType type);

    /// @brief Gets the type of this body.
    BodyType GetType() const noexcept;

    /// @brief Is "speedable".
    /// @details Is this body able to have a non-zero speed associated with it.
    /// Kinematic and Dynamic bodies are speedable. Static bodies are not.
    bool IsSpeedable() const noexcept;

    /// @brief Is "accelerable".
    /// @details Indicates whether this body is accelerable, ie. whether it is effected by
    ///   forces. Only Dynamic bodies are accelerable.
    /// @return true if the body is accelerable, false otherwise.
    bool IsAccelerable() const noexcept;

    /// @brief Sets the bullet status of this body.
    /// @details Sets whether or not this body should be treated like a bullet for continuous
    ///   collision detection.
    void SetBullet(bool flag) noexcept;

    /// @brief Is this body treated like a bullet for continuous collision detection?
    bool IsImpenetrable() const noexcept;

    /// You can disable sleeping on this body. If you disable sleeping, the
    /// body will be woken.
    void SetSleepingAllowed(bool flag) noexcept;

    /// @brief Gets whether or not this body allowed to sleep
    bool IsSleepingAllowed() const noexcept;

    /// @brief Awakens this body.
    ///
    /// @details Sets this body to awake and resets its under-active time if it's a "speedable"
    ///   body. This method has no effect otherwise.
    ///
    /// @post If this body is a "speedable" body, then this body's IsAwake method returns true.
    /// @post If this body is a "speedable" body, then this body's GetUnderActiveTime method
    ///   returns zero.
    ///
    void SetAwake() noexcept;

    /// @brief Sets this body to asleep if sleeping is allowed.
    ///
    /// @details If this body is allowed to sleep, this: sets the sleep state of the body to
    /// asleep, resets this body's under active time, and resets this body's velocity (linear
    /// and angular).
    ///
    /// @post This body's IsAwake method returns false.
    /// @post This body's GetUnderActiveTime method returns zero.
    /// @post This body's GetVelocity method returns zero linear and zero angular speed.
    ///
    void UnsetAwake() noexcept;

    /// @brief Gets the awake/asleep state of this body.
    /// @warning Being awake may or may not imply being speedable.
    /// @return true if the body is awake.
    bool IsAwake() const noexcept;

    /// @brief Gets this body's under-active time value.
    /// @return Zero or more time in seconds (of step time) that this body has been
    ///   "under-active" for.
    Time GetUnderActiveTime() const noexcept;

    /// @brief Sets the "under-active" time to the given value.
    ///
    /// @details Sets the "under-active" time to a value of zero or a non-zero value if the
    ///   body is "accelerable". Otherwise it does nothing.
    ///
    /// @warning Behavior is undefined for negative values.
    /// @note A non-zero time is only valid for an "accelerable" body.
    ///
    void SetUnderActiveTime(Time value) noexcept;

    /// @brief Resets the under-active time for this body.
    /// @note This has performance degrading potential and is best not called unless the
    ///   caller is certain that it should be.
    void ResetUnderActiveTime() noexcept;

    /// @brief Sets the enabled state of the body.
    ///
    /// @details A disabled body is not simulated and cannot be collided with or woken up.
    ///   If you pass a flag of true, all fixtures will be added to the broad-phase.
    ///   If you pass a flag of false, all fixtures will be removed from the broad-phase
    ///   and all contacts will be destroyed. Fixtures and joints are otherwise unaffected.
    ///
    /// @note A disabled body is still owned by a World object and remains in the body list.
    /// @note You may continue to create/destroy fixtures and joints on disabled bodies.
    /// @note Fixtures on a disabled body are implicitly disabled and will not participate in
    ///   collisions, ray-casts, or queries.
    /// @note Joints connected to a disabled body are implicitly disabled.
    ///
    void SetEnabled(bool flag);

    /// @brief Gets the enabled/disabled state of the body.
    bool IsEnabled() const noexcept;

    /// @brief Sets this body to have fixed rotation.
    /// @note This causes the mass to be reset.
    void SetFixedRotation(bool flag);

    /// @brief Does this body have fixed rotation?
    bool IsFixedRotation() const noexcept;

    /// @brief Gets the range of all constant fixtures attached to this body.
    SizedRange<Fixtures::const_iterator> GetFixtures() const noexcept;

    /// @brief Gets the range of all fixtures attached to this body.
    SizedRange<Fixtures::iterator> GetFixtures() noexcept;
    
    /// @brief Gets the range of all joints attached to this body.
    SizedRange<Joints::const_iterator> GetJoints() const noexcept;
 
    /// @brief Gets the range of all joints attached to this body.
    SizedRange<Joints::iterator> GetJoints() noexcept;
    
    /// @brief Gets the container of all contacts attached to this body.
    /// @warning This list changes during the time step and you may
    ///   miss some collisions if you don't use ContactListener.
    SizedRange<Contacts::const_iterator> GetContacts() const noexcept;

    /// @brief Gets the user data pointer that was provided in the body definition.
    void* GetUserData() const noexcept;

    /// @brief Sets the user data. Use this to store your application specific data.
    void SetUserData(void* data) noexcept;

    /// @brief Gets the parent world of this body.
    World* GetWorld() noexcept;

    /// @brief Gets the parent world of this body.
    const World* GetWorld() const noexcept;

    /// @brief Gets whether the mass data for this body is "dirty".
    bool IsMassDataDirty() const noexcept;
    
private:

    friend class BodyAtty;

    using FlagsType = std::uint16_t;

    // m_flags
    enum Flag: FlagsType
    {
        /// Awake flag.
        e_awakeFlag = 0x0002,

        /// Auto sleep flag.
        e_autoSleepFlag = 0x0004,

        /// Impenetrable flag.
        /// @details Indicates whether CCD should be done for this body.
        /// All static and kinematic bodies have this flag enabled.
        e_impenetrableFlag = 0x0008,

        /// Fixed rotation flag.
        e_fixedRotationFlag = 0x0010,

        /// Enabled flag.
        e_enabledFlag = 0x0020,

        /// Velocity flag.
        /// @details Set this to enable changes in position due to velocity.
        /// Bodies with this set are "speedable" - either kinematic or dynamic bodies.
        e_velocityFlag = 0x0080,

        /// Acceleration flag.
        /// @details Set this to enable changes in velocity due to physical properties (like forces).
        /// Bodies with this set are "accelerable" - dynamic bodies.
        e_accelerationFlag = 0x0100,

        /// Mass Data Dirty Flag.
        e_massDataDirtyFlag = 0x0200,
    };

    static FlagsType GetFlags(const BodyType type) noexcept;
    static FlagsType GetFlags(const BodyDef& bd) noexcept;

    /// @brief Sets the body's awake flag.
    /// @details This is done unconditionally.
    /// @note This should **not** be called unless the body is "speedable".
    /// @warning Behavior is undefined if called for a body that is not "speedable".
    void SetAwakeFlag() noexcept;

    void UnsetAwakeFlag() noexcept;

    /// Advances the body by a given time ratio.
    /// @details This method:
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

    void SetTransformation(const Transformation value) noexcept;

    //
    // Member variables. Try to keep total size small.
    //

    /// Transformation for body origin.
    /// @details
    /// This is essentially the cached result of <code>GetTransform1(m_sweep)</code>. 16-bytes.
    Transformation m_xf;

    Sweep m_sweep; ///< Sweep motion for CCD. 36-bytes.

    Velocity m_velocity; ///< Velocity (linear and angular). 12-bytes.
    FlagsType m_flags = 0; ///< Flags. 2-bytes.

    /// @brief Linear acceleration.
    /// @note 8-bytes.
    LinearAcceleration2D m_linearAcceleration = Vec2_zero * MeterPerSquareSecond;

    World* const m_world; ///< World to which this body belongs. 8-bytes.
    void* m_userData; ///< User data. 8-bytes.
    
    Fixtures m_fixtures; ///< Container of fixtures. 8-bytes.
    Contacts m_contacts; ///< Container of contacts (owned by world). 8-bytes.
    Joints m_joints; ///< Container of joints (owned by wolrd). 8-bytes.

    /// @brief Angular acceleration.
    /// @note 4-bytes.
    AngularAcceleration m_angularAcceleration = AngularAcceleration{0};

    /// Inverse mass of the body.
    /// @details A non-negative value.
    /// Can only be zero for non-accelerable bodies.
    /// @note 4-bytes.
    InvMass m_invMass = 0;

    /// Inverse rotational inertia about the center of mass.
    /// @details A non-negative value.
    /// @note 4-bytes.
    InvRotInertia m_invRotI = 0;

    NonNegative<Frequency> m_linearDamping; ///< Linear damping. 4-bytes.
    NonNegative<Frequency> m_angularDamping; ///< Angular damping. 4-bytes.

    /// Under-active time.
    /// @details A body under-active for enough time should have their awake flag unset.
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

inline Length2D Body::GetLocation() const noexcept
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

inline Length2D Body::GetWorldCenter() const noexcept
{
    return GetSweep().pos1.linear;
}

inline Length2D Body::GetLocalCenter() const noexcept
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

inline Frequency Body::GetLinearDamping() const noexcept
{
    return m_linearDamping;
}

inline void Body::SetLinearDamping(NonNegative<Frequency> linearDamping) noexcept
{
    m_linearDamping = linearDamping;
}

inline Frequency Body::GetAngularDamping() const noexcept
{
    return m_angularDamping;
}

inline void Body::SetAngularDamping(NonNegative<Frequency> angularDamping) noexcept
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
    // Protect the body's invariant that only "speedable" bodies can be awake.
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
    // Ignore this request unless this body is speedable so as to maintain the body's invariant
    // that only "speedable" bodies can be awake.
    if (IsSpeedable())
    {
    	SetAwakeFlag();
        ResetUnderActiveTime();
    }
}

inline void Body::UnsetAwake() noexcept
{
    if (!IsSpeedable() || IsSleepingAllowed())
    {
        UnsetAwakeFlag();
        m_underActiveTime = 0;
        m_velocity = Velocity{Vec2_zero * MeterPerSecond, AngularVelocity{0}};
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

inline void Body::ResetUnderActiveTime() noexcept
{
    m_underActiveTime = Second * RealNum(0);
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
        SetAwakeFlag();
        ResetUnderActiveTime();
    }
}

inline bool Body::IsSleepingAllowed() const noexcept
{
    return (m_flags & e_autoSleepFlag) != 0;
}

inline SizedRange<Body::Fixtures::const_iterator> Body::GetFixtures() const noexcept
{
    return SizedRange<Body::Fixtures::const_iterator>(m_fixtures.begin(), m_fixtures.end(),
                                                      m_fixtures.size());
}

inline SizedRange<Body::Fixtures::iterator> Body::GetFixtures() noexcept
{
    return SizedRange<Body::Fixtures::iterator>(m_fixtures.begin(), m_fixtures.end(),
                                                m_fixtures.size());
}

inline SizedRange<Body::Joints::const_iterator> Body::GetJoints() const noexcept
{
    return SizedRange<Body::Joints::const_iterator>(m_joints.begin(), m_joints.end(),
                                                    m_joints.size());
}

inline SizedRange<Body::Joints::iterator> Body::GetJoints() noexcept
{
    return SizedRange<Body::Joints::iterator>(m_joints.begin(), m_joints.end(), m_joints.size());
}

inline SizedRange<Body::Contacts::const_iterator> Body::GetContacts() const noexcept
{
    return SizedRange<Body::Contacts::const_iterator>(m_contacts.begin(), m_contacts.end(),
                                                      m_contacts.size());
}

inline void Body::SetUserData(void* data) noexcept
{
    m_userData = data;
}

inline void* Body::GetUserData() const noexcept
{
    return m_userData;
}

inline LinearAcceleration2D Body::GetLinearAcceleration() const noexcept
{
    return m_linearAcceleration;
}

inline AngularAcceleration Body::GetAngularAcceleration() const noexcept
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
/// @details Determines whether a body should possibly be able to collide with the other body.
/// @return true if either body is dynamic and no joint prevents collision, false otherwise.
bool ShouldCollide(const Body& lhs, const Body& rhs) noexcept;

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

inline void ApplyLinearAcceleration(Body& body, const LinearAcceleration2D amount)
{
    body.SetAcceleration(body.GetLinearAcceleration() + amount, body.GetAngularAcceleration());
}

inline void SetForce(Body& body, const Force2D force, const Length2D point) noexcept
{
    const auto linAccel = LinearAcceleration2D{force * body.GetInvMass()};
    const auto invRotI = body.GetInvRotInertia();
    const auto dp = point - body.GetWorldCenter();
    const auto cp = Torque{Cross(dp, force) / Radian};
    const auto angAccel = AngularAcceleration{cp * invRotI};
    body.SetAcceleration(linAccel, angAccel);
}

/// Apply a force at a world point.
/// @note If the force is not applied at the center of mass, it will generate a torque and
///   affect the angular velocity.
/// @note Non-zero forces wakes up the body.
/// @param body Body to apply the force to.
/// @param force World force vector.
/// @param point World position of the point of application.
inline void ApplyForce(Body& body, const Force2D force, const Length2D point) noexcept
{
    // Torque is L^2 M T^-2 QP^-1.
    const auto linAccel = LinearAcceleration2D{force * body.GetInvMass()};
    const auto invRotI = body.GetInvRotInertia(); // L^-2 M^-1 QP^2
    const auto dp = Length2D{point - body.GetWorldCenter()}; // L
    const auto cp = Torque{Cross(dp, force) / Radian}; // L * M L T^-2 is L^2 M T^-2
    // L^2 M T^-2 QP^-1 * L^-2 M^-1 QP^2 = QP T^-2;
    const auto angAccel = AngularAcceleration{cp * invRotI};
    body.SetAcceleration(body.GetLinearAcceleration() + linAccel,
                         body.GetAngularAcceleration() + angAccel);
}

/// Apply a force to the center of mass.
/// @note Non-zero forces wakes up the body.
/// @param body Body to apply the force to.
/// @param force World force vector.
inline void ApplyForceToCenter(Body& body, const Force2D force) noexcept
{
    const auto linAccel = body.GetLinearAcceleration() + force * body.GetInvMass();
    const auto angAccel = body.GetAngularAcceleration();
    body.SetAcceleration(linAccel, angAccel);
}

inline void SetTorque(Body& body, const Torque torque) noexcept
{
    const auto linAccel = body.GetLinearAcceleration();
    const auto invRotI = body.GetInvRotInertia();
    const auto angAccel = torque * invRotI;
    body.SetAcceleration(linAccel, angAccel);
}

/// Apply a torque.
/// @note This affects the angular velocity without affecting the linear velocity of the
///   center of mass.
/// @note Non-zero forces wakes up the body.
/// @param body Body to apply the torque to.
/// @param torque about the z-axis (out of the screen).
inline void ApplyTorque(Body& body, const Torque torque) noexcept
{
    const auto linAccel = body.GetLinearAcceleration();
    const auto invRotI = body.GetInvRotInertia();
    const auto angAccel = body.GetAngularAcceleration() + torque * invRotI;
    body.SetAcceleration(linAccel, angAccel);
}

/// Apply an impulse at a point.
/// @note This immediately modifies the velocity.
/// @note This also modifies the angular velocity if the point of application
///   is not at the center of mass.
/// @note Non-zero impulses wakes up the body.
/// @param body Body to apply the impulse to.
/// @param impulse the world impulse vector.
/// @param point the world position of the point of application.
inline void ApplyLinearImpulse(Body& body, const Momentum2D impulse, const Length2D point) noexcept
{
    auto velocity = body.GetVelocity();
    velocity.linear += body.GetInvMass() * impulse;
    const auto invRotI = body.GetInvRotInertia();
    const auto dp = point - body.GetWorldCenter();
    velocity.angular += AngularVelocity{invRotI * Cross(dp, impulse) / Radian};
    body.SetVelocity(velocity);
}

/// Apply an angular impulse.
/// @param body Body to apply the angular impulse to.
/// @param impulse Angular impulse to be applied.
inline void ApplyAngularImpulse(Body& body, AngularMomentum impulse) noexcept
{
    auto velocity = body.GetVelocity();
    const auto invRotI = body.GetInvRotInertia();
    velocity.angular += AngularVelocity{invRotI * impulse};
    body.SetVelocity(velocity);
}

Force2D GetCentripetalForce(const Body& body, const Length2D axis);

/// Gets the rotational inertia of the body.
/// @param body Body to get the rotational inertia for.
/// @return the rotational inertia.
inline RotInertia GetRotInertia(const Body& body) noexcept
{
    return RealNum{1} / body.GetInvRotInertia();
}

/// Gets the rotational inertia of the body about the local origin.
/// @return the rotational inertia.
inline RotInertia GetLocalInertia(const Body& body) noexcept
{
    return GetRotInertia(body)
         + GetMass(body) * GetLengthSquared(body.GetLocalCenter()) / SquareRadian;
}

/// Gets the linear velocity of the center of mass.
/// @param body Body to get the linear velocity for.
/// @return the linear velocity of the center of mass.
inline LinearVelocity2D GetLinearVelocity(const Body& body) noexcept
{
    return body.GetVelocity().linear;
}

/// Gets the angular velocity.
/// @param body Body to get the angular velocity for.
/// @return the angular velocity.
inline AngularVelocity GetAngularVelocity(const Body& body) noexcept
{
    return body.GetVelocity().angular;
}

/// Sets the linear velocity of the center of mass.
/// @param body Body to set the linear velocity of.
/// @param v the new linear velocity of the center of mass.
inline void SetLinearVelocity(Body& body, const LinearVelocity2D v) noexcept
{
    body.SetVelocity(Velocity{v, GetAngularVelocity(body)});
}

/// Sets the angular velocity.
/// @param body Body to set the angular velocity of.
/// @param omega the new angular velocity.
inline void SetAngularVelocity(Body& body, AngularVelocity omega) noexcept
{
    body.SetVelocity(Velocity{GetLinearVelocity(body), omega});
}

/// Gets the world coordinates of a point given in coordinates relative to the body's origin.
/// @param body Body that the given point is relative to.
/// @param localPoint a point measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
inline Length2D GetWorldPoint(const Body& body, const Length2D localPoint) noexcept
{
    return Transform(localPoint, body.GetTransformation());
}

/// Gets the world coordinates of a vector given the local coordinates.
/// @param body Body that the given vector is relative to.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
inline Length2D GetWorldVector(const Body& body, const Length2D localVector) noexcept
{
    return Rotate(localVector, body.GetTransformation().q);
}

inline UnitVec2 GetWorldVector(const Body& body, const UnitVec2 localVector) noexcept
{
    return Rotate(localVector, body.GetTransformation().q);
}

/// Gets a local point relative to the body's origin given a world point.
/// @param body Body that the returned point should be relative to.
/// @param worldPoint point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
inline Length2D GetLocalPoint(const Body& body, const Length2D worldPoint) noexcept
{
    return InverseTransform(worldPoint, body.GetTransformation());
}

/// Gets a locally oriented unit vector given a world oriented unit vector.
/// @param body Body that the returned vector should be relative to.
/// @param uv Unit vector in world orientation.
/// @return the corresponding local vector.
inline UnitVec2 GetLocalVector(const Body& body, const UnitVec2 uv) noexcept
{
    return InverseRotate(uv, body.GetTransformation().q);
}

/// Gets the linear velocity from a world point attached to this body.
/// @param body Body to get the linear velocity for.
/// @param worldPoint point in world coordinates.
/// @return the world velocity of a point.
inline LinearVelocity2D GetLinearVelocityFromWorldPoint(const Body& body,
                                                        const Length2D worldPoint) noexcept
{
    const auto velocity = body.GetVelocity();
    const auto worldCtr = body.GetWorldCenter();
    const auto dp = Length2D{worldPoint - worldCtr};
    const auto rlv = LinearVelocity2D{GetRevPerpendicular(dp) * velocity.angular / Radian};
    return velocity.linear + rlv;
}

/// Gets the linear velocity from a local point.
/// @param body Body to get the linear velocity for.
/// @param localPoint point in local coordinates.
/// @return the world velocity of a point.
inline LinearVelocity2D GetLinearVelocityFromLocalPoint(const Body& body,
                                                        const Length2D localPoint) noexcept
{
    return GetLinearVelocityFromWorldPoint(body, GetWorldPoint(body, localPoint));
}

inline Force2D GetForce(const Body& body) noexcept
{
    return body.GetLinearAcceleration() * GetMass(body);
}

inline Torque GetTorque(const Body& body) noexcept
{
    return body.GetAngularAcceleration() * GetRotInertia(body);
}

/// Gets the velocity of the body after the given time accounting for the body's acceleration.
/// @warning Behavior is undefined if the given elapsed time is an invalid value (like NaN).
/// @param body Body to get the velocity for.
/// @param h Time elapsed to get velocity for. Behavior is undefined if this value is invalid.
Velocity GetVelocity(const Body& body, Time h) noexcept;

size_t GetWorldIndex(const Body* body);

size_t GetFixtureCount(const Body& body);

/// Rotates a body a given amount around a point in world coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param worldPoint Point in world coordinates.
void RotateAboutWorldPoint(Body& body, Angle amount, Length2D worldPoint);

/// Rotates a body a given amount around a point in body local coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @note This is a convenience function that translates the local point into world coordinates
///   and then calls the <code>RotateAboutWorldPoint</code> function.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param localPoint Point in local coordinates.
void RotateAboutLocalPoint(Body& body, Angle amount, Length2D localPoint);

inline Body* GetBodyPtr(Body& body)
{
    return &body;
}

inline const Body* GetBodyPtr(const Body& body)
{
    return &body;
}

} // namespace box2d

#endif
