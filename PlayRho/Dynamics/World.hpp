/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_DYNAMICS_WORLD_HPP
#define PLAYRHO_DYNAMICS_WORLD_HPP

/// @file
/// Declarations of the World class and associated free functions.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Range.hpp> // for SizedRange
#include <PlayRho/Common/propagate_const.hpp>

#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp> // for GetDefaultBodyConf
#include <PlayRho/Dynamics/StepStats.hpp>
#include <PlayRho/Dynamics/Contacts/KeyedContactID.hpp> // for KeyedContactPtr
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Dynamics/WorldConf.hpp>
#include <PlayRho/Dynamics/Joints/JointID.hpp>
#include <PlayRho/Dynamics/Joints/JointType.hpp>

#include <iterator>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <functional>

namespace playrho {

class StepConf;
struct Filter;
struct FixtureProxy;

namespace d2 {

struct JointConf;
class WorldImpl;
class Manifold;
class ContactImpulsesList;
class DynamicTree;

/// @defgroup PhysicalEntities Physical Entity Classes
///
/// @brief Classes representing physical entities typically created/destroyed via factory methods.
///
/// @details Classes of creatable and destroyable managed instances that associate
///   physical properties to simulations. These instances are typically created via a
///   method whose name begins with the prefix of <code>Create</code>. Similarly, these
///   instances are typically destroyed using a method whose name begins with the prefix
///   of <code>Destroy</code>.
///
/// @note For example, the following could be used to create a dynamic body having a one meter
///   radius disk shape:
/// @code{.cpp}
/// auto world = World{};
/// const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
/// const auto fixture = body->CreateFixture(Shape{DiskShapeConf{1_m}});
/// @endcode
///
/// @see World, World::CreateBody, World::CreateJoint, World::Destroy.
/// @see Body::CreateFixture, Body::Destroy, Body::DestroyFixtures.
/// @see BodyType, Shape, DiskShapeConf.

/// @brief Definition of an independent and simulatable "world".
///
/// @details The world class manages physics entities, dynamic simulation, and queries.
///   In a physical sense, perhaps this is more like a universe in that entities in a
///   world have no interaction with entities in other worlds. In any case, there's
///   precedence, from a physics-engine standpoint, for this being called a world.
///
/// @note World instances do not themselves have any force or acceleration properties.
///  They simply utilize the acceleration property of the bodies they manage. This is
///  different than some other engines (like <code>Box2D</code> which provides a world
///  gravity property).
/// @note World instances are composed of &mdash; i.e. contain and own &mdash; Body, Joint,
///   and Contact instances.
/// @note This data structure is 232-bytes large (with 4-byte Real on at least one 64-bit
///   platform).
/// @attention For example, the following could be used to create a dynamic body having a one meter
///   radius disk shape:
/// @code{.cpp}
/// auto world = World{};
/// const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
/// const auto fixture = body->CreateFixture(Shape{DiskShapeConf{1_m}});
/// @endcode
///
/// @see Body, Joint, Contact, PhysicalEntities.
///
class World
{
public:
    /// @brief Bodies container type.
    using Bodies = std::vector<BodyID>;

    /// @brief Contacts container type.
    using Contacts = std::vector<KeyedContactPtr>;

    /// @brief Joints container type.
    /// @note Cannot be container of Joint instances since joints are polymorphic types.
    using Joints = std::vector<JointID>;

    /// @brief Body joints container type.
    using BodyJoints = std::vector<std::pair<BodyID, JointID>>;

    /// @brief Fixtures container type.
    using Fixtures = std::vector<FixtureID>;

    using FixtureProxies = std::vector<FixtureProxy>;

    using FixtureListener = std::function<void(FixtureID)>;
    using JointListener = std::function<void(JointID)>;
    using ContactListener = std::function<void(ContactID)>;
    using ManifoldContactListener = std::function<void(ContactID, const Manifold&)>;
    using ImpulsesContactListener = std::function<void(ContactID, const ContactImpulsesList&, unsigned)>;

    /// @brief Constructs a world object.
    /// @param def A customized world configuration or its default value.
    /// @note A lot more configurability can be had via the <code>StepConf</code>
    ///   data that's given to the world's <code>Step</code> method.
    /// @throws InvalidArgument if the given max vertex radius is less than the min.
    /// @see Step.
    explicit World(const WorldConf& def = GetDefaultWorldConf());

    /// @brief Copy constructor.
    /// @details Copy constructs this world with a deep copy of the given world.
    /// @post The state of this world is like that of the given world except this world now
    ///   has deep copies of the given world with pointers having the new addresses of the
    ///   new memory required for those copies.
    World(const World& other);

    /// @brief Assignment operator.
    /// @details Copy assigns this world with a deep copy of the given world.
    /// @post The state of this world is like that of the given world except this world now
    ///   has deep copies of the given world with pointers having the new addresses of the
    ///   new memory required for those copies.
    /// @warning This method should not be called while the world is locked!
    /// @throws WrongState if this method is called while the world is locked.
    World& operator= (const World& other);

    /// @brief Destructor.
    /// @details All physics entities are destroyed and all dynamically allocated memory
    ///    is released.
    ~World() noexcept;

    /// @brief Clears this world.
    /// @post The contents of this world have all been destroyed and this world's internal
    ///   state reset as though it had just been constructed.
    /// @throws WrongState if this method is called while the world is locked.
    void Clear();

    /// @brief Register a destruction listener for fixtures.
    void SetFixtureDestructionListener(const FixtureListener& listener) noexcept;

    /// @brief Register a destruction listener for joints.
    void SetJointDestructionListener(JointListener listener) noexcept;

    /// @brief Register a begin contact event listener.
    void SetBeginContactListener(ContactListener listener) noexcept;

    /// @brief Register an end contact event listener.
    void SetEndContactListener(ContactListener listener) noexcept;

    /// @brief Register a pre-solve contact event listener.
    void SetPreSolveContactListener(ManifoldContactListener listener) noexcept;

    /// @brief Register a post-solve contact event listener.
    void SetPostSolveContactListener(ImpulsesContactListener listener) noexcept;

    /// @brief Steps the world simulation according to the given configuration.
    ///
    /// @details
    /// Performs position and velocity updating, sleeping of non-moving bodies, updating
    /// of the contacts, and notifying the contact listener of begin-contact, end-contact,
    /// pre-solve, and post-solve events.
    ///
    /// @warning Behavior is undefined if given a negative step time delta.
    /// @warning Varying the step time delta may lead to non-physical behaviors.
    ///
    /// @note Calling this with a zero step time delta results only in fixtures and bodies
    ///   registered for proxy handling being processed. No physics is performed.
    /// @note If the given velocity and position iterations are zero, this method doesn't
    ///   do velocity or position resolutions respectively of the contacting bodies.
    /// @note While body velocities are updated accordingly (per the sum of forces acting on them),
    ///   body positions (barring any collisions) are updated as if they had moved the entire time
    ///   step at those resulting velocities. In other words, a body initially at position 0
    ///   (<code>p0</code>) going velocity 0 (<code>v0</code>) fast with a sum acceleration of
    ///   <code>a</code>, after time <code>t</code> and barring any collisions, will have a new
    ///   velocity (<code>v1</code>) of <code>v0 + (a * t)</code> and a new position
    ///   (<code>p1</code>) of <code>p0 + v1 * t</code>.
    ///
    /// @post Static bodies are unmoved.
    /// @post Kinetic bodies are moved based on their previous velocities.
    /// @post Dynamic bodies are moved based on their previous velocities, gravity, applied
    ///   forces, applied impulses, masses, damping, and the restitution and friction values
    ///   of their fixtures when they experience collisions.
    /// @post The bodies for proxies queue will be empty.
    /// @post The fixtures for proxies queue will be empty.
    ///
    /// @param conf Configuration for the simulation step.
    ///
    /// @return Statistics for the step.
    ///
    /// @throws WrongState if this method is called while the world is locked.
    ///
    /// @see GetBodiesForProxies, GetFixturesForProxies.
    ///
    StepStats Step(const StepConf& conf);

    /// @brief Whether or not "step" is complete.
    /// @details The "step" is completed when there are no more TOI events for the current time step.
    /// @return <code>true</code> unless sub-stepping is enabled and the step method returned
    ///   without finishing all of its sub-steps.
    /// @see GetSubStepping, SetSubStepping.
    bool IsStepComplete() const noexcept;
    
    /// @brief Gets whether or not sub-stepping is enabled.
    /// @see SetSubStepping, IsStepComplete.
    bool GetSubStepping() const noexcept;

    /// @brief Enables/disables single stepped continuous physics.
    /// @note This is not normally used. Enabling sub-stepping is meant for testing.
    /// @post The <code>GetSubStepping()</code> method will return the value this method was
    ///   called with.
    /// @see IsStepComplete, GetSubStepping.
    void SetSubStepping(bool flag) noexcept;

    /// @brief Gets access to the broad-phase dynamic tree information.
    const DynamicTree& GetTree() const noexcept;

    /// @brief Is the world locked (in the middle of a time step).
    bool IsLocked() const noexcept;

    /// @brief Shifts the world origin.
    /// @note Useful for large worlds.
    /// @note The body shift formula is: <code>position -= newOrigin</code>.
    /// @post The "origin" of this world's bodies, joints, and the board-phase dynamic tree
    ///   have been translated per the shift amount and direction.
    /// @param newOrigin the new origin with respect to the old origin
    /// @throws WrongState if this method is called while the world is locked.
    void ShiftOrigin(Length2 newOrigin);

    /// @brief Gets the minimum vertex radius that shapes in this world can be.
    Length GetMinVertexRadius() const noexcept;
    
    /// @brief Gets the maximum vertex radius that shapes in this world can be.
    Length GetMaxVertexRadius() const noexcept;

    /// @brief Gets the inverse delta time.
    /// @details Gets the inverse delta time that was set on construction or assignment, and
    ///   updated on every call to the <code>Step()</code> method having a non-zero delta-time.
    /// @see Step.
    Frequency GetInvDeltaTime() const noexcept;

    std::size_t GetShapeCount() const noexcept;

    /// @brief Gets the world body range for this constant world.
    /// @details Gets a range enumerating the bodies currently existing within this world.
    ///   These are the bodies that had been created from previous calls to the
    ///   <code>CreateBody(const BodyConf&)</code> method that haven't yet been destroyed.
    /// @return Body range that can be iterated over using its begin and end methods
    ///   or using ranged-based for-loops.
    /// @see CreateBody(const BodyConf&).
    SizedRange<Bodies::const_iterator> GetBodies() const noexcept;

    /// @brief Gets the bodies-for-proxies range for this world.
    /// @details Provides insight on what bodies have been queued for proxy processing
    ///   during the next call to the world step method.
    /// @see Step.
    SizedRange<World::Bodies::const_iterator> GetBodiesForProxies() const noexcept;

    /// @brief Creates a rigid body with the given configuration.
    /// @warning This function should not be used while the world is locked &mdash; as it is
    ///   during callbacks. If it is, it will throw an exception or abort your program.
    /// @note No references to the configuration are retained. Its value is copied.
    /// @post The created body will be present in the range returned from the
    ///   <code>GetBodies()</code> method.
    /// @param def A customized body configuration or its default value.
    /// @return Pointer to newly created body which can later be destroyed by calling the
    ///   <code>Destroy(Body*)</code> method.
    /// @throws WrongState if this method is called while the world is locked.
    /// @throws LengthError if this operation would create more than <code>MaxBodies</code>.
    /// @see Destroy(Body*), GetBodies.
    /// @see PhysicalEntities.
    BodyID CreateBody(const BodyConf& def = GetDefaultBodyConf());

    /// @brief Destroys the given body.
    /// @details Destroys a given body that had previously been created by a call to this
    ///   world's <code>CreateBody(const BodyConf&)</code> method.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    /// @warning Behavior is undefined if given a null body.
    /// @warning Behavior is undefined if the passed body was not created by this world.
    /// @note This function is locked during callbacks.
    /// @post The destroyed body will no longer be present in the range returned from the
    ///   <code>GetBodies()</code> method.
    /// @post None of the body's fixtures will be present in the fixtures-for-proxies
    ///   collection.
    /// @param id Body to destroy that had been created by this world.
    /// @throws WrongState if this method is called while the world is locked.
    /// @see CreateBody(const BodyConf&), GetBodies, GetFixturesForProxies.
    /// @see PhysicalEntities.
    void Destroy(BodyID id);

    /// @brief Gets the type of this body.
    BodyType GetType(BodyID id) const;

    /// @brief Sets the type of the given body.
    /// @note This may alter the body's mass and velocity.
    /// @throws WrongState if this method is called while the world is locked.
    void SetType(BodyID id, BodyType type);

    /// @brief Creates a fixture and attaches it to the given body.
    /// @details Creates a fixture for attaching a shape and other characteristics to this
    ///   body. Fixtures automatically go away when this body is destroyed. Fixtures can
    ///   also be manually removed and destroyed using the
    ///   <code>Destroy(Fixture*, bool)</code>, or <code>DestroyFixtures()</code> methods.
    ///
    /// @note This function should not be called if the world is locked.
    /// @warning This function is locked during callbacks.
    ///
    /// @post After creating a new fixture, it will show up in the fixture enumeration
    ///   returned by the <code>GetFixtures()</code> methods.
    ///
    /// @param shape Shareable shape definition.
    ///   Its vertex radius must be less than the minimum or more than the maximum allowed by
    ///   the body's world.
    /// @param def Initial fixture settings.
    ///   Friction and density must be >= 0.
    ///   Restitution must be > -infinity and < infinity.
    /// @param resetMassData Whether or not to reset the mass data of the body.
    ///
    /// @return Pointer to the created fixture.
    ///
    /// @throws WrongState if called while the world is "locked".
    /// @throws InvalidArgument if called for a shape with a vertex radius less than the
    ///    minimum vertex radius.
    /// @throws InvalidArgument if called for a shape with a vertex radius greater than the
    ///    maximum vertex radius.
    ///
    /// @see Destroy, GetFixtures
    /// @see PhysicalEntities
    ///
    FixtureID CreateFixture(BodyID body, const Shape& shape,
                            const FixtureConf& def = GetDefaultFixtureConf(),
                            bool resetMassData = true);

    /// @brief Destroys fixtures of the given body.
    /// @details Destroys all of the fixtures previously created for this body by the
    ///   <code>CreateFixture(const Shape&, const FixtureConf&, bool)</code> method.
    /// @note This unconditionally calls the <code>ResetMassData()</code> method.
    /// @post After this call, no fixtures will show up in the fixture enumeration
    ///   returned by the <code>GetFixtures()</code> methods.
    /// @see CreateFixture, GetFixtures, ResetMassData.
    /// @see PhysicalEntities
    void DestroyFixtures(BodyID id);

    /// @brief Gets the enabled/disabled state of the body.
    /// @see SetEnabled.
    bool IsEnabled(BodyID id) const;

    /// @brief Sets the enabled state of the body.
    ///
    /// @details A disabled body is not simulated and cannot be collided with or woken up.
    ///   If you pass a flag of true, all fixtures will be added to the broad-phase.
    ///   If you pass a flag of false, all fixtures will be removed from the broad-phase
    ///   and all contacts will be destroyed. Fixtures and joints are otherwise unaffected.
    ///
    /// @note A disabled body is still owned by a World object and remains in the world's
    ///   body container.
    /// @note You may continue to create/destroy fixtures and joints on disabled bodies.
    /// @note Fixtures on a disabled body are implicitly disabled and will not participate in
    ///   collisions, ray-casts, or queries.
    /// @note Joints connected to a disabled body are implicitly disabled.
    ///
    /// @throws WrongState If call would change body's state when world is locked.
    ///
    /// @post <code>IsEnabled()</code> returns the state given to this function.
    ///
    /// @see IsEnabled.
    ///
    void SetEnabled(BodyID id, bool flag);

    /// @brief Gets the range of all joints attached to this body.
    SizedRange<BodyJoints::const_iterator> GetJoints(BodyID id) const;

    /// @brief Computes the body's mass data.
    /// @details This basically accumulates the mass data over all fixtures.
    /// @note The center is the mass weighted sum of all fixture centers. Divide it by the
    ///   mass to get the averaged center.
    /// @return accumulated mass data for all fixtures associated with the given body.
    MassData ComputeMassData(BodyID id) const;

    /// @brief Set the mass properties to override the mass properties of the fixtures.
    /// @note This changes the center of mass position.
    /// @note Creating or destroying fixtures can also alter the mass.
    /// @note This function has no effect if the body isn't dynamic.
    /// @param massData the mass properties.
    void SetMassData(BodyID id, const MassData& massData);

    /// @brief Gets the body configuration for the identified body.
    /// @throws std::out_of_range If given an invalid body identifier.
    BodyConf GetBodyConf(BodyID id) const;

    /// @brief Gets the range of all constant fixtures attached to the given body.
    SizedRange<Fixtures::const_iterator> GetFixtures(BodyID id) const;

    FixtureCounter GetFixtureCount(BodyID id) const;

    /// @brief Get the angle.
    /// @return the current world rotation angle.
    Angle GetAngle(BodyID id) const;

    /// @brief Gets the body's transformation.
    /// @see SetTransformation(BodyID id, Transformation xfm).
    Transformation GetTransformation(BodyID id) const;

    /// @brief Sets the transformation of the body.
    /// @details This instantly adjusts the body to be at the new transformation.
    /// @warning Manipulating a body's transformation can cause non-physical behavior!
    /// @note Contacts are updated on the next call to World::Step.
    /// @see GetTransformation(BodyID id).
    void SetTransformation(BodyID id, Transformation xfm);

    /// @brief Gets the local position of the center of mass of the specified body.
    Length2 GetLocalCenter(BodyID id) const;

    /// @brief Gets the world position of the center of mass of the specified body.
    Length2 GetWorldCenter(BodyID id) const;

    /// @brief Gets the velocity of the identified body.
    /// @see SetVelocity(BodyID id, const Velocity& value).
    Velocity GetVelocity(BodyID id) const;

    /// @brief Sets the body's velocity (linear and angular velocity).
    /// @note This method does nothing if this body is not speedable.
    /// @note A non-zero velocity will awaken this body.
    /// @see GetVelocity(BodyID), SetAwake, SetUnderActiveTime.
    void SetVelocity(BodyID id, const Velocity& value);

    /// @brief Gets the awake/asleep state of this body.
    /// @warning Being awake may or may not imply being speedable.
    /// @return true if the body is awake.
    bool IsAwake(BodyID id) const;

    /// @brief Wakes up the identified body.
    void SetAwake(BodyID id);

    /// @brief Sleeps the identified body.
    /// @see IsAwake(BodyID id), SetAwake(BodyID id).
    void UnsetAwake(BodyID id);

    /// @brief Gets this body's linear acceleration.
    LinearAcceleration2 GetLinearAcceleration(BodyID id) const;

    /// @brief Gets this body's angular acceleration.
    AngularAcceleration GetAngularAcceleration(BodyID id) const;

    /// @brief Sets the linear and rotational accelerations on the body.
    /// @note This has no effect on non-accelerable bodies.
    /// @note A non-zero acceleration will also awaken the body.
    /// @param id Body whose acceleration should be set.
    /// @param linear Linear acceleration.
    /// @param angular Angular acceleration.
    void SetAcceleration(BodyID id, LinearAcceleration2 linear, AngularAcceleration angular);

    /// @brief Gets whether the body's mass-data is dirty.
    bool IsMassDataDirty(BodyID id) const;

    /// @brief Gets whether the body has fixed rotation.
    /// @see SetFixedRotation(BodyID id, bool value).
    bool IsFixedRotation(BodyID id) const;

    /// @brief Sets the body to have fixed rotation.
    /// @note This causes the mass to be reset.
    /// @see IsFixedRotation(BodyID id).
    void SetFixedRotation(BodyID id, bool value);

    /// @brief Gets the inverse total mass of the body.
    /// @return Value of zero or more representing the body's inverse mass (in 1/kg).
    /// @see SetMassData.
    InvMass GetInvMass(BodyID id) const;

    /// @brief Gets the inverse rotational inertia of the body.
    /// @return Inverse rotational inertia (in 1/kg-m^2).
    InvRotInertia GetInvRotInertia(BodyID id) const;

    /// @brief Is identified body "speedable".
    /// @details Is the body able to have a non-zero speed associated with it.
    /// Kinematic and Dynamic bodies are speedable. Static bodies are not.
    bool IsSpeedable(BodyID id) const;

    /// @brief Is identified body "accelerable"?
    /// @details Indicates whether the body is accelerable, i.e. whether it is effected by
    ///   forces. Only Dynamic bodies are accelerable.
    /// @return true if the body is accelerable, false otherwise.
    bool IsAccelerable(BodyID id) const;

    /// @brief Is the body treated like a bullet for continuous collision detection?
    bool IsImpenetrable(BodyID id) const;

    /// @brief Gets the container of all contacts attached to the body.
    /// @warning This collection changes during the time step and you may
    ///   miss some collisions if you don't use <code>ContactListener</code>.
    SizedRange<Contacts::const_iterator> GetContacts(BodyID id) const;

    /// @brief Gets the user data associated with the identified body.
    void* GetUserData(BodyID id) const;

    /// @brief Gets the world joint range.
    /// @details Gets a range enumerating the joints currently existing within this world.
    ///   These are the joints that had been created from previous calls to the
    ///   <code>CreateJoint(const JointConf&)</code> method that haven't yet been destroyed.
    /// @return World joints sized-range.
    /// @see CreateJoint(const JointConf&).
    SizedRange<Joints::const_iterator> GetJoints() const noexcept;

    /// @brief Creates a joint to constrain one or more bodies.
    /// @warning This function is locked during callbacks.
    /// @note No references to the configuration are retained. Its value is copied.
    /// @post The created joint will be present in the range returned from the
    ///   <code>GetJoints()</code> method.
    /// @return Pointer to newly created joint which can later be destroyed by calling the
    ///   <code>Destroy(Joint*)</code> method.
    /// @throws WrongState if this method is called while the world is locked.
    /// @throws LengthError if this operation would create more than <code>MaxJoints</code>.
    /// @throws InvalidArgument if the given definition is not allowed.
    /// @see PhysicalEntities.
    /// @see Destroy(Joint*), GetJoints.
    JointID CreateJoint(const JointConf& def);

    /// @brief Destroys a joint.
    /// @details Destroys a given joint that had previously been created by a call to this
    ///   world's <code>CreateJoint(const JointConf&)</code> method.
    /// @warning This function is locked during callbacks.
    /// @warning Behavior is undefined if the passed joint was not created by this world.
    /// @note This may cause the connected bodies to begin colliding.
    /// @post The destroyed joint will no longer be present in the range returned from the
    ///   <code>GetJoints()</code> method.
    /// @param id Joint to destroy that had been created by this world.
    /// @throws WrongState if this method is called while the world is locked.
    /// @see CreateJoint(const JointConf&), GetJoints.
    /// @see PhysicalEntities.
    void Destroy(JointID id);

    /// @brief Wakes up the joined bodies.
    void SetAwake(JointID id);

    /// @brief Gets collide connected for the specified joint.
    /// @note Modifying the collide connect flag won't work correctly because
    ///   the flag is only checked when fixture AABBs begin to overlap.
    bool GetCollideConnected(JointID id) const;

    JointType GetType(JointID id) const;

    /// @brief Gets the user data associated with the identified joint.
    /// @relatedalso World
    void* GetUserData(JointID id) const;

    BodyID GetBodyA(JointID id) const;
    BodyID GetBodyB(JointID id) const;
    Length2 GetLocalAnchorA(JointID id) const;
    Length2 GetLocalAnchorB(JointID id) const;

    /// Gets the linear reaction on body-B at the joint anchor.
    Momentum2 GetLinearReaction(JointID id) const;

    /// @brief Get the angular reaction on body-B for the identified joint.
    AngularMomentum GetAngularReaction(JointID id) const;

    Angle GetReferenceAngle(JointID id) const;
    UnitVec GetLocalAxisA(JointID id) const;

    /// @brief Gets the angular motor speed for joints which support this.
    /// @see SetMotorSpeed(JointID id, AngularVelocity value)
    AngularVelocity GetMotorSpeed(JointID id) const;

    /// @brief Sets the angular motor speed for joints which support this.
    /// @see GetMotorSpeed(JointID id)
    void SetMotorSpeed(JointID id, AngularVelocity value);

    /// @brief Gets the max motor torque.
    Torque GetMaxMotorTorque(JointID id) const;

    /// Sets the maximum motor torque.
    void SetMaxMotorTorque(JointID id, Torque value);

    /// @brief Gets the angular motor impulse of the identified joint.
    AngularMomentum GetAngularMotorImpulse(JointID id) const;

    /// @brief Gets the fixtures-for-proxies range for this world.
    /// @details Provides insight on what fixtures have been queued for proxy processing
    ///   during the next call to the world step method.
    /// @see Step.
    SizedRange<Fixtures::const_iterator> GetFixturesForProxies() const noexcept;

    /// @brief Destroys a fixture.
    ///
    /// @details Destroys a fixture previously created by the
    ///   <code>CreateFixture(const Shape&, const FixtureConf&, bool)</code>
    ///   method. This removes the fixture from the broad-phase and destroys all contacts
    ///   associated with this fixture. All fixtures attached to a body are implicitly
    ///   destroyed when the body is destroyed.
    ///
    /// @warning This function is locked during callbacks.
    /// @note Make sure to explicitly call <code>ResetMassData()</code> after fixtures have
    ///   been destroyed if resetting the mass data is not requested via the reset mass data
    ///   parameter.
    /// @throws WrongState if this method is called while the world is locked.
    ///
    /// @post After destroying a fixture, it will no longer show up in the fixture enumeration
    ///   returned by the <code>GetFixtures()</code> methods.
    ///
    /// @param id the fixture to be removed.
    /// @param resetMassData Whether or not to reset the mass data of the associated body.
    ///
    /// @see CreateFixture, Body::GetFixtures, Body::ResetMassData.
    /// @see PhysicalEntities
    ///
    bool Destroy(FixtureID id, bool resetMassData = true);

    /// @brief Re-filter the fixture.
    /// @note Call this if you want to establish collision that was previously disabled by
    ///   <code>ShouldCollide(const Fixture&, const Fixture&)</code>.
    /// @see bool ShouldCollide(const Fixture& fixtureA, const Fixture& fixtureB) noexcept
    void Refilter(FixtureID id);

    /// @brief Sets the contact filtering data.
    /// @note This won't update contacts until the next time step when either parent body
    ///    is speedable and awake.
    /// @note This automatically calls <code>Refilter</code>.
    void SetFilterData(FixtureID id, const Filter& filter);

    /// @brief Gets the world contact range.
    /// @warning contacts are created and destroyed in the middle of a time step.
    /// Use <code>ContactListener</code> to avoid missing contacts.
    /// @return World contacts sized-range.
    SizedRange<Contacts::const_iterator> GetContacts() const noexcept;

    /// @brief Gets the identifier of the body associated with the specified fixture.
    BodyID GetBody(FixtureID id) const;

    /// @brief Gets the user data associated with the identified fixture.
    void* GetUserData(FixtureID id) const;

    Shape GetShape(FixtureID id) const;

    /// @brief Sets whether the fixture is a sensor or not.
    /// @see IsSensor(FixtureID id).
    void SetSensor(FixtureID id, bool value);

    /// @brief Is the specified fixture a sensor (non-solid)?
    /// @return the true if the fixture is a sensor.
    bool IsSensor(FixtureID id) const;

    AreaDensity GetDensity(FixtureID id) const;

    const FixtureProxies& GetProxies(FixtureID id) const;

    /// Is the joint motor enabled?
    /// @see EnableMotor(JointID id, bool value)
    bool IsMotorEnabled(JointID id) const;

    /// Enable/disable the joint motor.
    /// @see IsMotorEnabled(JointID id).
    void EnableMotor(JointID id, bool value);

    /// @brief Gets the awake status of the specified contact.
    /// @see SetAwake(ContactID id)
    bool IsAwake(ContactID id) const;

    /// @brief Sets the awake status of the specified contact.
    /// @see IsAwake(ContactID id)
    void SetAwake(ContactID id);

    /// @brief Is this contact touching?
    /// @details
    /// Touching is defined as either:
    ///   1. This contact's manifold has more than 0 contact points, or
    ///   2. This contact has sensors and the two shapes of this contact are found to be
    ///      overlapping.
    /// @return true if this contact is said to be touching, false otherwise.
    bool IsTouching(ContactID id) const;

    /// @brief Whether or not the contact needs filtering.
    bool NeedsFiltering(ContactID id) const;

    /// @brief Whether or not the contact needs updating.
    bool NeedsUpdating(ContactID id) const;

    /// @brief Gets fixture A of the given contact.
    FixtureID GetFixtureA(ContactID id) const;

    /// @brief Gets fixture B of the given contact.
    FixtureID GetFixtureB(ContactID id) const;

    Real GetDefaultFriction(ContactID id) const;

    Real GetDefaultRestitution(ContactID id) const;

    /// @brief Gets the friction used with the specified contact.
    /// @see SetFriction(ContactID id, Real value)
    Real GetFriction(ContactID id) const;

    /// @brief Gets the restitution used with the specified contact.
    /// @see SetRestitution(ContactID id, Real value)
    Real GetRestitution(ContactID id) const;

    /// @brief Sets the friction value for the specified contact.
    /// @details Overrides the default friction mixture.
    /// @note You can call this in "pre-solve" listeners.
    /// @note This value persists until set or reset.
    /// @warning Behavior is undefined if given a negative friction value.
    /// @param value Co-efficient of friction value of zero or greater.
    void SetFriction(ContactID id, Real value);

    /// @brief Sets the restitution value for the specified contact.
    /// @details This override the default restitution mixture.
    /// @note You can call this in "pre-solve" listeners.
    /// @note The value persists until you set or reset.
    void SetRestitution(ContactID id, Real value);

    /// @brief Gets the collision manifold for the identified contact.
    const Manifold& GetManifold(ContactID id) const;

private:
    propagate_const<std::unique_ptr<WorldImpl>> m_impl;
};

/// @example HelloWorld.cpp
/// This is the source file for the <code>HelloWorld</code> application that demonstrates
/// use of the playrho::d2::World class and more.

/// @example World.cpp
/// This is the <code>googletest</code> based unit testing file for the
/// <code>playrho::d2::World</code> class.

// Free functions.

/// @brief Gets the body count in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline BodyCounter GetBodyCount(const World& world) noexcept
{
    return static_cast<BodyCounter>(size(world.GetBodies()));
}

/// Gets the count of joints in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline JointCounter GetJointCount(const World& world) noexcept
{
    return static_cast<JointCounter>(size(world.GetJoints()));
}

/// @brief Gets the count of contacts in the given world.
/// @note Not all contacts are for shapes that are actually touching. Some contacts are for
///   shapes which merely have overlapping AABBs.
/// @return 0 or higher.
/// @relatedalso World
inline ContactCounter GetContactCount(const World& world) noexcept
{
    return static_cast<ContactCounter>(size(world.GetContacts()));
}

/// @brief Gets the touching count for the given world.
/// @relatedalso World
ContactCounter GetTouchingCount(const World& world) noexcept;

/// @brief Steps the world ahead by a given time amount.
///
/// @details Performs position and velocity updating, sleeping of non-moving bodies, updating
///   of the contacts, and notifying the contact listener of begin-contact, end-contact,
///   pre-solve, and post-solve events.
///   If the given velocity and position iterations are more than zero, this method also
///   respectively performs velocity and position resolution of the contacting bodies.
///
/// @note While body velocities are updated accordingly (per the sum of forces acting on them),
///   body positions (barring any collisions) are updated as if they had moved the entire time
///   step at those resulting velocities. In other words, a body initially at <code>p0</code>
///   going <code>v0</code> fast with a sum acceleration of <code>a</code>, after time
///   <code>t</code> and barring any collisions, will have a new velocity (<code>v1</code>) of
///   <code>v0 + (a * t)</code> and a new position (<code>p1</code>) of <code>p0 + v1 * t</code>.
///
/// @warning Varying the time step may lead to non-physical behaviors.
///
/// @post Static bodies are unmoved.
/// @post Kinetic bodies are moved based on their previous velocities.
/// @post Dynamic bodies are moved based on their previous velocities, gravity,
/// applied forces, applied impulses, masses, damping, and the restitution and friction values
/// of their fixtures when they experience collisions.
///
/// @param world World to step.
/// @param delta Time to simulate as a delta from the current state. This should not vary.
/// @param velocityIterations Number of iterations for the velocity constraint solver.
/// @param positionIterations Number of iterations for the position constraint solver.
///   The position constraint solver resolves the positions of bodies that overlap.
///
/// @relatedalso World
///
StepStats Step(World& world, Time delta,
               TimestepIters velocityIterations = 8,
               TimestepIters positionIterations = 3);

/// @copydoc World::GetFixturesForProxies
/// @relatedalso World
inline SizedRange<World::Fixtures::const_iterator> GetFixturesForProxies(const World& world) noexcept
{
    return world.GetFixturesForProxies();
}

/// @brief Gets the range of all constant fixtures attached to the given body.
/// @relatedalso World
inline SizedRange<World::Fixtures::const_iterator> GetFixtures(const World& world, BodyID id)
{
    return world.GetFixtures(id);
}

/// @relatedalso World
inline FixtureCounter GetFixtureCount(const World& world, BodyID id)
{
    return world.GetFixtureCount(id);
}

/// @brief Gets the count of fixtures in the given world.
/// @relatedalso World
FixtureCounter GetFixtureCount(const World& world) noexcept;

/// @brief Gets the count of unique shapes in the given world.
/// @relatedalso World
std::size_t GetShapeCount(const World& world) noexcept;

/// @brief Gets the count of awake bodies in the given world.
/// @relatedalso World
BodyCounter GetAwakeCount(const World& world) noexcept;

/// @brief Awakens all of the bodies in the given world.
/// @details Calls all of the world's bodies' <code>SetAwake</code> method.
/// @return Sum total of calls to bodies' <code>SetAwake</code> method that returned true.
/// @see Body::SetAwake.
/// @relatedalso World
BodyCounter Awaken(World& world) noexcept;

/// @copydoc World::GetLinearAcceleration
/// @relatedalso World
inline LinearAcceleration2 GetLinearAcceleration(const World& world, BodyID id)
{
    return world.GetLinearAcceleration(id);
}

/// @copydoc World::GetAngularAcceleration
/// @relatedalso World
inline AngularAcceleration GetAngularAcceleration(const World& world, BodyID id)
{
    return world.GetAngularAcceleration(id);
}

/// @brief Gets the acceleration of the identified body.
/// @relatedalso World
inline Acceleration GetAcceleration(const World& world, BodyID id)
{
    return Acceleration{
        world.GetLinearAcceleration(id),
        world.GetAngularAcceleration(id)
    };
}

/// @copydoc World::SetAcceleration
/// @relatedalso World
inline void SetAcceleration(World& world, BodyID id,
                            LinearAcceleration2 linear, AngularAcceleration angular)
{
    world.SetAcceleration(id, linear, angular);
}

inline void SetAcceleration(World& world, BodyID id, LinearAcceleration2 value)
{
    world.SetAcceleration(id, value, world.GetAngularAcceleration(id));
}

/// @brief Sets the accelerations on the given body.
/// @note This has no effect on non-accelerable bodies.
/// @note A non-zero acceleration will also awaken the body.
/// @param id Body whose acceleration should be set.
/// @param value Acceleration value to set.
/// @relatedalso World
inline void SetAcceleration(World& world, BodyID id, Acceleration value)
{
    world.SetAcceleration(id, value.linear, value.angular);
}

/// @brief Sets the accelerations of all the world's bodies.
/// @param world World instance to set the acceleration of all contained bodies for.
/// @param fn Function or functor with a signature like:
///   <code>Acceleration (*fn)(const Body& body)</code>.
/// @relatedalso World
template <class F>
void SetAccelerations(World& world, F fn)
{
    const auto bodies = world.GetBodies();
    std::for_each(begin(bodies), end(bodies), [&](const auto &b) {
        SetAcceleration(world, b, fn(b));
    });
}

/// @brief Sets the accelerations of all the world's bodies to the given value.
/// @relatedalso World
void SetAccelerations(World& world, Acceleration acceleration) noexcept;

/// @brief Sets the accelerations of all the world's bodies to the given value.
/// @note This will leave the angular acceleration alone.
/// @relatedalso World
void SetAccelerations(World& world, LinearAcceleration2 acceleration) noexcept;

/// @brief Clears forces.
/// @details Manually clear the force buffer on all bodies.
/// @relatedalso World
inline void ClearForces(World& world) noexcept
{
    SetAccelerations(world, Acceleration{
        LinearAcceleration2{0_mps2, 0_mps2}, 0 * RadianPerSquareSecond
    });
}

/// @brief Finds body in given world that's closest to the given location.
/// @relatedalso World
BodyID FindClosestBody(const World& world, Length2 location) noexcept;

/// @brief Sets the body's transformation.
/// @see GetTransformation
inline void SetTransformation(World& world, BodyID id, Transformation xfm)
{
    world.SetTransformation(id, xfm);
}

/// @brief Sets the position of the body's origin and rotation.
/// @details This instantly adjusts the body to be at the new position and new orientation.
/// @warning Manipulating a body's transform can cause non-physical behavior!
/// @note Contacts are updated on the next call to World::Step.
/// @param location Valid world location of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @param angle Valid world rotation. Behavior is undefined if value is invalid.
inline void SetTransform(World& world, BodyID id, Length2 location, Angle angle)
{
    SetTransformation(world, id, Transformation{location, UnitVec::Get(angle)});
}

/// @brief Sets the body's location.
/// @details This instantly adjusts the body to be at the new location.
/// @warning Manipulating a body's location this way can cause non-physical behavior!
/// @param body Body to move.
/// @param value Valid world location of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @see Body::SetTransform
/// @relatedalso Body
void SetLocation(World& world, BodyID body, Length2 value);

/// @brief Sets the body's angular orientation.
/// @details This instantly adjusts the body to be at the new angular orientation.
/// @warning Manipulating a body's angle this way can cause non-physical behavior!
/// @param body Body to move.
/// @param value Valid world angle of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @see Body::SetTransform
/// @relatedalso Body
void SetAngle(World& world, BodyID body, Angle value);

/// @brief Rotates a body a given amount around a point in world coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param worldPoint Point in world coordinates.
/// @relatedalso Body
void RotateAboutWorldPoint(World& world, BodyID body, Angle amount, Length2 worldPoint);

/// @brief Rotates a body a given amount around a point in body local coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @note This is a convenience function that translates the local point into world coordinates
///   and then calls the <code>RotateAboutWorldPoint</code> function.
/// @param body Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param localPoint Point in local coordinates.
/// @relatedalso Body
void RotateAboutLocalPoint(World& world, BodyID body, Angle amount, Length2 localPoint);

/// @brief Calculates the gravitationally associated acceleration for the given body within its world.
/// @relatedalso Body
/// @return Zero acceleration if given body is has no mass, else the acceleration of
///    the body due to the gravitational attraction to the other bodies.
Acceleration CalcGravitationalAcceleration(const World& world, const BodyID body);

/// @brief Gets the world index for the given body.
/// @relatedalso Body
BodyCounter GetWorldIndex(const World& world, const BodyID id) noexcept;

/// @brief Gets the body configuration for the identified body.
/// @throws std::out_of_range If given an invalid body identifier.
inline BodyConf GetBodyConf(const World& world, BodyID id)
{
    return world.GetBodyConf(id);
}

/// @copydoc World::SetType
/// @see GetType(const World& world, BodyID id)
/// @relatedalso World
inline void SetType(World& world, BodyID id, BodyType value)
{
    world.SetType(id, value);
}

/// @copydoc World::GetType
/// @see SetType(World& world, BodyID id, BodyType value)
/// @relatedalso World
inline BodyType GetType(const World& world, BodyID id)
{
    return world.GetType(id);
}

/// @brief Gets the type of the joint.
/// @relatedalso World
inline JointType GetType(const World& world, JointID id)
{
    return world.GetType(id);
}

/// @copydoc World::GetCollideConnected
/// @relatedalso World
inline bool GetCollideConnected(const World& world, JointID id)
{
    return world.GetCollideConnected(id);
}

/// @see SetTransformation
/// @relatedalso World
inline Transformation GetTransformation(const World& world, BodyID id)
{
    return world.GetTransformation(id);
}

/// @relatedalso World
inline Length2 GetLocation(const World& world, BodyID id)
{
    return GetTransformation(world, id).p;
}

/// @brief Gets the world coordinates of a point given in coordinates relative to the body's origin.
/// @param world World context.
/// @param id Body that the given point is relative to.
/// @param localPoint a point measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
/// @relatedalso World
inline Length2 GetWorldPoint(const World& world, BodyID id, const Length2 localPoint)
{
    return Transform(localPoint, GetTransformation(world, id));
}

/// @relatedalso World
inline BodyID GetBody(const World& world, FixtureID id)
{
    return world.GetBody(id);
}

/// @copydoc World::GetUserData(FixtureID)
/// @relatedalso World
inline void* GetUserData(const World& world, FixtureID id)
{
    return world.GetUserData(id);
}

/// @brief Gets the coefficient of friction of the specified fixture.
/// @return Value of 0 or higher.
inline Real GetFriction(const World& world, FixtureID id)
{
    return GetFriction(world.GetShape(id));
}

/// @brief Gets the coefficient of restitution of the specified fixture.
inline Real GetRestitution(const World& world, FixtureID id)
{
    return GetRestitution(world.GetShape(id));
}

/// @brief Gets the transformation associated with the given fixture.
/// @warning Behavior is undefined if the fixture doesn't have an associated body - i.e.
///   behavior is undefined if the fixture has <code>nullptr</code> as its associated body.
/// @relatedalso World
inline Transformation GetTransformation(const World& world, FixtureID id)
{
    return GetTransformation(world, GetBody(world, id));
}

inline Shape GetShape(const World& world, FixtureID id)
{
    return world.GetShape(id);
}

/// @brief Sets whether the fixture is a sensor or not.
/// @see IsSensor.
inline void SetSensor(World& world, FixtureID id, bool value)
{
    world.SetSensor(id, value);
}

/// @brief Is the specified fixture a sensor (non-solid)?
/// @return the true if the fixture is a sensor.
/// @see SetSensor.
inline bool IsSensor(const World& world, FixtureID id)
{
    return world.IsSensor(id);
}

/// @brief Gets the density of this fixture.
/// @return Non-negative density (in mass per area).
inline AreaDensity GetDensity(const World& world, FixtureID id)
{
    return world.GetDensity(id);
}

inline const World::FixtureProxies& GetProxies(const World& world, FixtureID id)
{
    return world.GetProxies(id);
}

/// @relatedalso World
ChildCounter GetProxyCount(const World& world, FixtureID id);

/// @relatedalso World
const FixtureProxy& GetProxy(const World& world, FixtureID id, ChildCounter child);

/// @relatedalso World
inline UnitVec GetLocalVector(const World& world, BodyID body, const UnitVec uv)
{
    return InverseRotate(uv, GetTransformation(world, body).q);
}

/// @brief Gets a local point relative to the body's origin given a world point.
/// @param body Body that the returned point should be relative to.
/// @param worldPoint point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
/// @relatedalso Body
inline Length2 GetLocalPoint(const World& world, BodyID body, const Length2 worldPoint)
{
    return InverseTransform(worldPoint, GetTransformation(world, body));
}

/// @copydoc World::GetAngle
/// @relatedalso World
inline Angle GetAngle(const World& world, BodyID id)
{
    return world.GetAngle(id);
}

inline Position GetPosition(const World& world, BodyID id)
{
    return Position{GetLocation(world, id), GetAngle(world, id)};
}

/// @relatedalso World
inline UnitVec GetWorldVector(const World& world, BodyID body, UnitVec localVector)
{
    return Rotate(localVector, GetTransformation(world, body).q);
}

/// @copydoc World::GetVelocity
/// @relatedalso World
inline Velocity GetVelocity(const World& world, BodyID id)
{
    return world.GetVelocity(id);
}

/// @brief Gets the linear velocity of the center of mass of the identified body.
/// @param world World in which body is identified for.
/// @param id Body to get the linear velocity for.
/// @return the linear velocity of the center of mass.
/// @relatedalso World
inline LinearVelocity2 GetLinearVelocity(const World& world, BodyID id)
{
    return GetVelocity(world, id).linear;
}

/// @copydoc World::SetVelocity
/// @see GetVelocity(const World& world, BodyID id)
/// @relatedalso World
inline void SetVelocity(World& world, BodyID id, const Velocity& value)
{
    world.SetVelocity(id, value);
}

inline void SetVelocity(World& world, BodyID id, const LinearVelocity2& value)
{
    world.SetVelocity(id, Velocity{value, GetVelocity(world, id).angular});
}

/// @copydoc World::DestroyFixtures()
/// @relatedalso World
inline void DestroyFixtures(World& world, BodyID id)
{
    world.DestroyFixtures(id);
}

/// @copydoc World::IsEnabled()
/// @see SetEnabled(World& world, BodyID id, bool value).
/// @relatedalso World
inline bool IsEnabled(const World& world, BodyID id)
{
    return world.IsEnabled(id);
}

/// @copydoc World::SetEnabled()
/// @see IsEnabled(const World& world, BodyID id).
/// @relatedalso World
inline void SetEnabled(World& world, BodyID id, bool value)
{
    world.SetEnabled(id, value);
}

/// @copydoc World::IsMotorEnabled()
/// @relatedalso World
inline bool IsMotorEnabled(const World& world, JointID id)
{
    return world.IsMotorEnabled(id);
}

/// @copydoc World::EnableMotor()
/// @relatedalso World
inline void EnableMotor(World& world, JointID id, bool value)
{
    world.EnableMotor(id, value);
}

/// @brief Gets the awake/asleep state of this body.
/// @warning Being awake may or may not imply being speedable.
/// @return true if the body is awake.
/// @relatedalso World
inline bool IsAwake(const World& world, BodyID id)
{
    return world.IsAwake(id);
}

/// @copydoc World::SetAwake(BodyID)
/// @relatedalso World
inline void SetAwake(World& world, BodyID id)
{
    world.SetAwake(id);
}

/// @copydoc World::UnsetAwake(BodyID)
/// @relatedalso World
inline void UnsetAwake(World& world, BodyID id)
{
    world.UnsetAwake(id);
}

/// @brief Awakens the body if it's asleep.
/// @relatedalso World
inline bool Awaken(World& world, BodyID id)
{
    if (!IsAwake(world, id) && IsSpeedable(GetType(world, id)))
    {
        SetAwake(world, id);
        return true;
    }
    return false;
}

/// @brief Gets whether the body's mass-data is dirty.
inline bool IsMassDataDirty(const World& world, BodyID id)
{
    return world.IsMassDataDirty(id);
}

/// @brief Gets whether the body has fixed rotation.
/// @see SetFixedRotation.
inline bool IsFixedRotation(const World& world, BodyID id)
{
    return world.IsFixedRotation(id);
}

/// @brief Sets this body to have fixed rotation.
/// @note This causes the mass to be reset.
inline void SetFixedRotation(World& world, BodyID id, bool value)
{
    world.SetFixedRotation(id, value);
}

/// @brief Get the world position of the center of mass of the specified body.
inline Length2 GetWorldCenter(const World& world, BodyID id)
{
    return world.GetWorldCenter(id);
}

/// @brief Gets the inverse total mass of the body.
/// @return Value of zero or more representing the body's inverse mass (in 1/kg).
/// @see SetMassData.
inline InvMass GetInvMass(const World& world, BodyID id)
{
    return world.GetInvMass(id);
}

/// @brief Gets the inverse rotational inertia of the body.
/// @return Inverse rotational inertia (in 1/kg-m^2).
inline InvRotInertia GetInvRotInertia(const World& world, BodyID id)
{
    return world.GetInvRotInertia(id);
}

/// @brief Gets the mass of the body.
/// @note This may be the total calculated mass or it may be the set mass of the body.
/// @return Value of zero or more representing the body's mass.
/// @see GetInvMass, SetMassData
/// @relatedalso World
inline Mass GetMass(const World& world, BodyID id)
{
    const auto invMass = world.GetInvMass(id);
    return (invMass != InvMass{0})? Mass{Real{1} / invMass}: 0_kg;
}

/// @brief Gets the rotational inertia of the body.
/// @param id Body to get the rotational inertia for.
/// @return the rotational inertia.
/// @relatedalso World
inline RotInertia GetRotInertia(const World& world, BodyID id)
{
    return Real{1} / GetInvRotInertia(world, id);
}

/// @brief Gets the local position of the center of mass of the specified body.
inline Length2 GetLocalCenter(const World& world, BodyID id)
{
    return world.GetLocalCenter(id);
}

/// @brief Gets the rotational inertia of the body about the local origin.
/// @return the rotational inertia.
/// @relatedalso World
inline RotInertia GetLocalRotInertia(const World& world, BodyID id)
{
    return GetRotInertia(world, id)
         + GetMass(world, id) * GetMagnitudeSquared(GetLocalCenter(world, id)) / SquareRadian;
}

inline MassData GetMassData(const World& world, FixtureID id)
{
    return GetMassData(world.GetShape(id));
}

/// @brief Gets the mass data of the body.
/// @return Data structure containing the mass, inertia, and center of the body.
/// @relatedalso World
inline MassData GetMassData(const World& world, BodyID id)
{
    return MassData{GetLocalCenter(world, id), GetMass(world, id), GetLocalRotInertia(world, id)};
}

/// @brief Computes the body's mass data.
/// @details This basically accumulates the mass data over all fixtures.
/// @note The center is the mass weighted sum of all fixture centers. Divide it by the
///   mass to get the averaged center.
/// @return accumulated mass data for all fixtures associated with the given body.
/// @relatedalso World
inline MassData ComputeMassData(const World& world, BodyID id)
{
    return world.ComputeMassData(id);
}

/// @brief Sets the mass properties to override the mass properties of the fixtures.
/// @note This changes the center of mass position.
/// @note Creating or destroying fixtures can also alter the mass.
/// @note This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
inline void SetMassData(World& world, BodyID id, const MassData& massData)
{
    world.SetMassData(id, massData);
}

/// @brief Resets the mass data properties.
/// @details This resets the mass data to the sum of the mass properties of the fixtures.
/// @note This method must be called after calling <code>CreateFixture</code> to update the
///   body mass data properties unless <code>SetMassData</code> is used.
/// @see SetMassData.
inline void ResetMassData(World& world, BodyID id)
{
    SetMassData(world, id, ComputeMassData(world, id));
}

/// @brief Should collide.
/// @details Determines whether a body should possibly be able to collide with the other body.
/// @relatedalso World
/// @return true if either body is dynamic and no joint prevents collision, false otherwise.
bool ShouldCollide(const World& world, BodyID lhs, BodyID rhs);

/// @brief Gets the range of all joints attached to this body.
inline SizedRange<World::BodyJoints::const_iterator> GetJoints(const World& world, BodyID id)
{
    return world.GetJoints(id);
}

/// @brief Is identified body "speedable".
/// @details Is the body able to have a non-zero speed associated with it.
/// Kinematic and Dynamic bodies are speedable. Static bodies are not.
inline bool IsSpeedable(const World& world, BodyID id)
{
    return world.IsSpeedable(id);
}

/// @brief Is identified body "accelerable"?
/// @details Indicates whether the body is accelerable, i.e. whether it is effected by
///   forces. Only Dynamic bodies are accelerable.
/// @return true if the body is accelerable, false otherwise.
inline bool IsAccelerable(const World& world, BodyID id)
{
    return world.IsAccelerable(id);
}

/// @brief Is the body treated like a bullet for continuous collision detection?
inline bool IsImpenetrable(const World& world, BodyID id)
{
    return world.IsImpenetrable(id);
}

/// @brief Gets the container of all contacts attached to this body.
/// @warning This collection changes during the time step and you may
///   miss some collisions if you don't use <code>ContactListener</code>.
inline SizedRange<World::Contacts::const_iterator> GetContacts(const World& world, BodyID id)
{
    return world.GetContacts(id);
}

/// @brief Gets the user data associated with the identified body.
/// @relatedalso World
inline void* GetUserData(const World& world, BodyID id)
{
    return world.GetUserData(id);
}

/// @copydoc World::IsTouching(ContactID)
inline bool IsTouching(const World& world, ContactID id)
{
    return world.IsTouching(id);
}

/// @copydoc World::IsAwake(ContactID)
inline bool IsAwake(const World& world, ContactID id)
{
    return world.IsAwake(id);
}

/// @brief Sets awake the bodies of the fixtures of the given contact.
/// @relatedalso World
inline void SetAwake(World& world, ContactID id)
{
    world.SetAwake(id);
}

/// @copydoc World::GetFixtureA
/// @relatedalso World
inline FixtureID GetFixtureA(const World& world, ContactID id)
{
    return world.GetFixtureA(id);
}

/// @copydoc World::GetFixtureB
/// @relatedalso World
inline FixtureID GetFixtureB(const World& world, ContactID id)
{
    return world.GetFixtureB(id);
}

/// @copydoc World::NeedsFiltering
/// @relatedalso World
inline bool NeedsFiltering(const World& world, ContactID id)
{
    return world.NeedsFiltering(id);
}

/// @copydoc World::NeedsUpdating
/// @relatedalso World
inline bool NeedsUpdating(const World& world, ContactID id)
{
    return world.NeedsUpdating(id);
}

inline Real GetDefaultFriction(const World& world, ContactID id)
{
    return world.GetDefaultFriction(id);
}

inline Real GetDefaultRestitution(const World& world, ContactID id)
{
    return world.GetDefaultRestitution(id);
}

/// @copydoc World::GetFriction(ContactID id)
/// @see SetFriction(World& world, ContactID id, Real friction)
inline Real GetFriction(const World& world, ContactID id)
{
    return world.GetFriction(id);
}

/// @copydoc GetRestitution(ContactID id)
/// @see SetRestitution(World& world, ContactID id, Real restitution)
inline Real GetRestitution(const World& world, ContactID id)
{
    return world.GetRestitution(id);
}

/// @brief Sets the friction value for the specified contact.
/// @details Overrides the default friction mixture.
/// @note You can call this in "pre-solve" listeners.
/// @note This value persists until set or reset.
/// @warning Behavior is undefined if given a negative friction value.
/// @param friction Co-efficient of friction value of zero or greater.
/// @relatedalso World
inline void SetFriction(World& world, ContactID id, Real friction)
{
    world.SetFriction(id, friction);
}

/// @brief Sets the restitution value for the specified contact.
/// @details This override the default restitution mixture.
/// @note You can call this in "pre-solve" listeners.
/// @note The value persists until you set or reset.
/// @relatedalso World
inline void SetRestitution(World& world, ContactID id, Real restitution)
{
    world.SetRestitution(id, restitution);
}

/// Resets the friction mixture to the default value.
/// @relatedalso World
inline void ResetFriction(World& world, ContactID id)
{
    SetFriction(world, id, GetDefaultFriction(world, id));
}

/// Resets the restitution to the default value.
/// @relatedalso World
inline void ResetRestitution(World& world, ContactID id)
{
    SetRestitution(world, id, GetDefaultRestitution(world, id));
}

inline const Manifold& GetManifold(const World& world, ContactID id)
{
    return world.GetManifold(id);
}

/// @copydoc World::GetUserData(JointID)
/// @relatedalso World
inline void* GetUserData(const World& world, JointID id)
{
    return world.GetUserData(id);
}

inline BodyID GetBodyA(const World& world, JointID id)
{
    return world.GetBodyA(id);
}

inline BodyID GetBodyB(const World& world, JointID id)
{
    return world.GetBodyB(id);
}

/// Get the anchor point on body-A in local coordinates.
inline Length2 GetLocalAnchorA(const World& world, JointID id)
{
    return world.GetLocalAnchorA(id);
}

/// Get the anchor point on body-B in local coordinates.
inline Length2 GetLocalAnchorB(const World& world, JointID id)
{
    return world.GetLocalAnchorB(id);
}

/// @copydoc World::GetLinearReaction
inline Momentum2 GetLinearReaction(const World& world, JointID id)
{
    return world.GetLinearReaction(id);
}

/// @copydoc World::GetAngularReaction
inline AngularMomentum GetAngularReaction(const World& world, JointID id)
{
    return world.GetAngularReaction(id);
}

inline Angle GetReferenceAngle(const World& world, JointID id)
{
    return world.GetReferenceAngle(id);
}

inline UnitVec GetLocalAxisA(const World& world, JointID id)
{
    return world.GetLocalAxisA(id);
}

/// @copydoc World::GetMotorSpeed
/// @see SetMotorSpeed(World& world, JointID id, AngularVelocity value)
inline AngularVelocity GetMotorSpeed(const World& world, JointID id)
{
    return world.GetMotorSpeed(id);
}

/// @copydoc World::SetMotorSpeed
/// @see GetMotorSpeed(const World& world, JointID id)
inline void SetMotorSpeed(World& world, JointID id, AngularVelocity value)
{
    world.SetMotorSpeed(id, value);
}

/// @brief Gets the max motor torque.
inline Torque GetMaxMotorTorque(const World& world, JointID id)
{
    return world.GetMaxMotorTorque(id);
}

/// Set the maximum motor torque.
inline void SetMaxMotorTorque(World& world, JointID id, Torque value)
{
    world.SetMaxMotorTorque(id, value);
}

inline AngularMomentum GetAngularMotorImpulse(const World& world, JointID id)
{
    return world.GetAngularMotorImpulse(id);
}

/// @brief Gets the enabled/disabled state of the joint.
inline bool IsEnabled(const World& world, JointID id)
{
    const auto bA = GetBodyA(world, id);
    const auto bB = GetBodyB(world, id);
    return (bA == InvalidBodyID || IsEnabled(world, bA))
        && (bB == InvalidBodyID || IsEnabled(world, bB));
}

/// @brief Gets the world index of the given joint.
/// @relatedalso World
JointCounter GetWorldIndex(const World& world, JointID id) noexcept;

/// Get the anchor point on body-A in world coordinates.
inline Length2 GetAnchorA(const World& world, JointID id)
{
    return GetWorldPoint(world, GetBodyA(world, id), GetLocalAnchorA(world, id));
}

/// Get the anchor point on body-B in world coordinates.
inline Length2 GetAnchorB(const World& world, JointID id)
{
    return GetWorldPoint(world, GetBodyB(world, id), GetLocalAnchorB(world, id));
}

/// @brief Tests a point for containment in a fixture.
/// @param id Fixture to use for test.
/// @param p Point in world coordinates.
/// @relatedalso World
/// @ingroup TestPointGroup
bool TestPoint(const World& world, FixtureID id, Length2 p);

/// @brief Gets the centripetal force necessary to put the body into an orbit having
///    the given radius.
/// @relatedalso World
Force2 GetCentripetalForce(const World& world, BodyID id, Length2 axis);

/// @brief Applies a force to the center of mass of the given body.
/// @note Non-zero forces wakes up the body.
/// @param id Body to apply the force to.
/// @param force World force vector.
/// @relatedalso World
inline void ApplyForceToCenter(World& world, BodyID id, Force2 force) noexcept
{
    const auto linAccel = GetLinearAcceleration(world, id) + force * GetInvMass(world, id);
    const auto angAccel = GetAngularAcceleration(world, id);
    SetAcceleration(world, id, linAccel, angAccel);
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLD_HPP
