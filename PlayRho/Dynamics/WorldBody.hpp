/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DYNAMICS_WORLDBODY_HPP
#define PLAYRHO_DYNAMICS_WORLDBODY_HPP

/// @file
/// Declarations of free functions of World for bodies identified by <code>BodyID</code>.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Range.hpp> // for SizedRange

#include <PlayRho/Collision/MassData.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp> // for GetDefaultBodyConf
#include <PlayRho/Dynamics/Contacts/KeyedContactID.hpp> // for KeyedContactPtr
#include <PlayRho/Dynamics/Joints/JointID.hpp>

#include <iterator>
#include <vector>
#include <functional>

namespace playrho {
namespace d2 {

class World;
class Shape;

/// @example WorldBody.cpp
/// This is the <code>googletest</code> based unit testing file for the free function
///   interfaces to <code>playrho::d2::World</code> body member functions and additional
///   functionality.

/// @brief Gets the bodies of the specified world.
/// @relatedalso World
SizedRange<std::vector<BodyID>::const_iterator> GetBodies(const World& world) noexcept;

/// @brief Gets the bodies-for-proxies range for the given world.
SizedRange<std::vector<BodyID>::const_iterator>
GetBodiesForProxies(const World& world) noexcept;

/// @brief Creates a rigid body with the given configuration.
/// @warning This function should not be used while the world is locked &mdash; as it is
///   during callbacks. If it is, it will throw an exception or abort your program.
/// @note No references to the configuration are retained. Its value is copied.
/// @post The created body will be present in the range returned from the
///   <code>GetBodies(const World&)</code> method.
/// @param world The world within which to create the body.
/// @param def A customized body configuration or its default value.
/// @return Identifier of the newly created body which can later be destroyed by calling
///   the <code>Destroy(World&, BodyID)</code> method.
/// @throws WrongState if this method is called while the world is locked.
/// @throws LengthError if this operation would create more than <code>MaxBodies</code>.
/// @see Destroy(BodyID), GetBodies(const World&).
/// @see PhysicalEntities.
/// @relatedalso World
BodyID CreateBody(World& world, const BodyConf& def = GetDefaultBodyConf());

/// @brief Destroys the identified body.
/// @see CreateBody(World&, const BodyConf&).
/// @relatedalso World
void Destroy(World& world, BodyID id);

/// @brief Gets the range of all constant fixtures attached to the given body.
/// @relatedalso World
SizedRange<std::vector<FixtureID>::const_iterator> GetFixtures(const World& world, BodyID id);

/// @brief Gets the count of fixtures associated with the identified body.
/// @relatedalso World
inline FixtureCounter GetFixtureCount(const World& world, BodyID id)
{
    using std::size;
    return static_cast<FixtureCounter>(size(GetFixtures(world, id)));
}

/// @copydoc World::GetLinearAcceleration
/// @relatedalso World
LinearAcceleration2 GetLinearAcceleration(const World& world, BodyID id);

/// @copydoc World::GetAngularAcceleration
/// @relatedalso World
AngularAcceleration GetAngularAcceleration(const World& world, BodyID id);

/// @brief Gets the acceleration of the identified body.
/// @relatedalso World
Acceleration GetAcceleration(const World& world, BodyID id);

/// @copydoc World::SetAcceleration
/// @relatedalso World
void SetAcceleration(World& world, BodyID id,
                     LinearAcceleration2 linear, AngularAcceleration angular);

/// @copydoc World::SetAcceleration
/// @relatedalso World
void SetAcceleration(World& world, BodyID id, LinearAcceleration2 value);

/// @copydoc World::SetAcceleration
/// @relatedalso World
void SetAcceleration(World& world, BodyID id, AngularAcceleration value);

/// @brief Sets the accelerations on the given body.
/// @note This has no effect on non-accelerable bodies.
/// @note A non-zero acceleration will also awaken the body.
/// @param id Body whose acceleration should be set.
/// @param value Acceleration value to set.
/// @relatedalso World
void SetAcceleration(World& world, BodyID id, Acceleration value);

/// @brief Sets the body's transformation.
/// @see GetTransformation
/// @relatedalso World
void SetTransformation(World& world, BodyID id, Transformation xfm);

/// @brief Sets the position of the body's origin and rotation.
/// @details This instantly adjusts the body to be at the new position and new orientation.
/// @warning Manipulating a body's transform can cause non-physical behavior!
/// @note Contacts are updated on the next call to World::Step.
/// @param location Valid world location of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @param angle Valid world rotation. Behavior is undefined if value is invalid.
/// @relatedalso World
inline void SetTransform(World& world, BodyID id, Length2 location, Angle angle)
{
    SetTransformation(world, id, Transformation{location, UnitVec::Get(angle)});
}

/// @brief Sets the body's location.
/// @details This instantly adjusts the body to be at the new location.
/// @warning Manipulating a body's location this way can cause non-physical behavior!
/// @param id Body to move.
/// @param value Valid world location of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @see Body::SetTransform
/// @relatedalso World
void SetLocation(World& world, BodyID id, Length2 value);

/// @brief Sets the body's angular orientation.
/// @details This instantly adjusts the body to be at the new angular orientation.
/// @warning Manipulating a body's angle this way can cause non-physical behavior!
/// @param id Body to move.
/// @param value Valid world angle of the body's local origin. Behavior is undefined
///   if value is invalid.
/// @see Body::SetTransform
/// @relatedalso World
void SetAngle(World& world, BodyID id, Angle value);

/// @brief Rotates a body a given amount around a point in world coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @param id Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param worldPoint Point in world coordinates.
/// @relatedalso World
void RotateAboutWorldPoint(World& world, BodyID id, Angle amount, Length2 worldPoint);

/// @brief Rotates a body a given amount around a point in body local coordinates.
/// @details This changes both the linear and angular positions of the body.
/// @note Manipulating a body's position this way may cause non-physical behavior.
/// @note This is a convenience function that translates the local point into world coordinates
///   and then calls the <code>RotateAboutWorldPoint</code> function.
/// @param id Body to rotate.
/// @param amount Amount to rotate body by (in counter-clockwise direction).
/// @param localPoint Point in local coordinates.
/// @relatedalso World
void RotateAboutLocalPoint(World& world, BodyID id, Angle amount, Length2 localPoint);

/// @brief Calculates the gravitationally associated acceleration for the given body within its world.
/// @return Zero acceleration if given body is has no mass, else the acceleration of
///    the body due to the gravitational attraction to the other bodies.
/// @relatedalso World
Acceleration CalcGravitationalAcceleration(const World& world, BodyID id);

/// @brief Gets the world index for the given body.
/// @relatedalso World
BodyCounter GetWorldIndex(const World& world, const BodyID id) noexcept;

/// @brief Gets the body configuration for the identified body.
/// @throws std::out_of_range If given an invalid body identifier.
/// @relatedalso World
BodyConf GetBodyConf(const World& world, BodyID id);

/// @copydoc World::SetType
/// @see GetType(const World& world, BodyID id)
/// @relatedalso World
void SetType(World& world, BodyID id, BodyType value);

/// @copydoc World::GetType
/// @see SetType(World& world, BodyID id, BodyType value)
/// @relatedalso World
BodyType GetType(const World& world, BodyID id);

/// @see SetTransformation
/// @relatedalso World
Transformation GetTransformation(const World& world, BodyID id);

/// @brief Convenience function for getting just the location of the identified body.
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

/// @brief Convenience function for getting the local vector of the identified body.
/// @relatedalso World
inline UnitVec GetLocalVector(const World& world, BodyID body, const UnitVec uv)
{
    return InverseRotate(uv, GetTransformation(world, body).q);
}

/// @brief Gets a local point relative to the body's origin given a world point.
/// @param body Body that the returned point should be relative to.
/// @param worldPoint point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
/// @relatedalso World
inline Length2 GetLocalPoint(const World& world, BodyID body, const Length2 worldPoint)
{
    return InverseTransform(worldPoint, GetTransformation(world, body));
}

/// @copydoc World::GetAngle
/// @relatedalso World
Angle GetAngle(const World& world, BodyID id);

/// @brief Convenience function for getting the position of the identified body.
inline Position GetPosition(const World& world, BodyID id)
{
    return Position{GetLocation(world, id), GetAngle(world, id)};
}

/// @brief Convenience function for getting a world vector of the identified body.
/// @relatedalso World
inline UnitVec GetWorldVector(const World& world, BodyID body, UnitVec localVector)
{
    return Rotate(localVector, GetTransformation(world, body).q);
}

/// @copydoc World::GetVelocity
/// @relatedalso World
Velocity GetVelocity(const World& world, BodyID id);

/// @brief Gets the linear velocity of the center of mass of the identified body.
/// @param world World in which body is identified for.
/// @param id Body to get the linear velocity for.
/// @return the linear velocity of the center of mass.
/// @relatedalso World
inline LinearVelocity2 GetLinearVelocity(const World& world, BodyID id)
{
    return GetVelocity(world, id).linear;
}

/// @brief Gets the angular velocity.
/// @param world World in which body is identified for.
/// @param id Body to get the angular velocity for.
/// @return the angular velocity.
/// @relatedalso World
inline AngularVelocity GetAngularVelocity(const World& world, BodyID id)
{
    return GetVelocity(world, id).angular;
}

/// @copydoc World::SetVelocity
/// @see GetVelocity(const World& world, BodyID id)
/// @relatedalso World
void SetVelocity(World& world, BodyID id, const Velocity& value);

/// @brief Sets the velocity of the identified body.
/// @relatedalso World
void SetVelocity(World& world, BodyID id, const LinearVelocity2& value);

/// @brief Sets the velocity of the identified body.
/// @relatedalso World
void SetVelocity(World& world, BodyID id, AngularVelocity value);

/// @copydoc World::DestroyFixtures()
/// @relatedalso World
void DestroyFixtures(World& world, BodyID id);

/// @copydoc World::IsEnabled()
/// @see SetEnabled(World& world, BodyID id, bool value).
/// @relatedalso World
bool IsEnabled(const World& world, BodyID id);

/// @copydoc World::SetEnabled()
/// @see IsEnabled(const World& world, BodyID id).
/// @relatedalso World
void SetEnabled(World& world, BodyID id, bool value);

/// @brief Gets the awake/asleep state of this body.
/// @warning Being awake may or may not imply being speedable.
/// @return true if the body is awake.
/// @relatedalso World
bool IsAwake(const World& world, BodyID id);

/// @copydoc World::SetAwake(BodyID)
/// @relatedalso World
void SetAwake(World& world, BodyID id);

/// @copydoc World::UnsetAwake(BodyID)
/// @relatedalso World
void UnsetAwake(World& world, BodyID id);

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
/// @relatedalso World
bool IsMassDataDirty(const World& world, BodyID id);

/// @brief Gets whether the body has fixed rotation.
/// @see SetFixedRotation.
/// @relatedalso World
bool IsFixedRotation(const World& world, BodyID id);

/// @brief Sets this body to have fixed rotation.
/// @note This causes the mass to be reset.
/// @relatedalso World
void SetFixedRotation(World& world, BodyID id, bool value);

/// @brief Get the world position of the center of mass of the specified body.
/// @relatedalso World
Length2 GetWorldCenter(const World& world, BodyID id);

/// @brief Gets the inverse total mass of the body.
/// @return Value of zero or more representing the body's inverse mass (in 1/kg).
/// @see SetMassData.
/// @relatedalso World
InvMass GetInvMass(const World& world, BodyID id);

/// @brief Gets the inverse rotational inertia of the body.
/// @return Inverse rotational inertia (in 1/kg-m^2).
/// @relatedalso World
InvRotInertia GetInvRotInertia(const World& world, BodyID id);

/// @brief Gets the mass of the body.
/// @note This may be the total calculated mass or it may be the set mass of the body.
/// @return Value of zero or more representing the body's mass.
/// @see GetInvMass, SetMassData
/// @relatedalso World
inline Mass GetMass(const World& world, BodyID id)
{
    const auto invMass = GetInvMass(world, id);
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
/// @relatedalso World
Length2 GetLocalCenter(const World& world, BodyID id);

/// @brief Gets the rotational inertia of the body about the local origin.
/// @return the rotational inertia.
/// @relatedalso World
inline RotInertia GetLocalRotInertia(const World& world, BodyID id)
{
    return GetRotInertia(world, id)
         + GetMass(world, id) * GetMagnitudeSquared(GetLocalCenter(world, id)) / SquareRadian;
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
MassData ComputeMassData(const World& world, BodyID id);

/// @brief Sets the mass properties to override the mass properties of the fixtures.
/// @note This changes the center of mass position.
/// @note Creating or destroying fixtures can also alter the mass.
/// @note This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
/// @relatedalso World
void SetMassData(World& world, BodyID id, const MassData& massData);

/// @brief Resets the mass data properties.
/// @details This resets the mass data to the sum of the mass properties of the fixtures.
/// @note This method must be called after calling <code>CreateFixture</code> to update the
///   body mass data properties unless <code>SetMassData</code> is used.
/// @see SetMassData.
/// @relatedalso World
inline void ResetMassData(World& world, BodyID id)
{
    SetMassData(world, id, ComputeMassData(world, id));
}

/// @brief Should collide.
/// @details Determines whether a body should possibly be able to collide with the other body.
/// @return true if either body is dynamic and no joint prevents collision, false otherwise.
/// @relatedalso World
bool ShouldCollide(const World& world, BodyID lhs, BodyID rhs);

/// @brief Gets the range of all joints attached to the identified body.
/// @relatedalso World
SizedRange<std::vector<std::pair<BodyID, JointID>>::const_iterator>
GetJoints(const World& world, BodyID id);

/// @brief Is identified body "speedable".
/// @details Is the body able to have a non-zero speed associated with it.
///  Kinematic and Dynamic bodies are speedable. Static bodies are not.
/// @relatedalso World
bool IsSpeedable(const World& world, BodyID id);

/// @brief Is identified body "accelerable"?
/// @details Indicates whether the body is accelerable, i.e. whether it is effected by
///   forces. Only Dynamic bodies are accelerable.
/// @return true if the body is accelerable, false otherwise.
/// @relatedalso World
bool IsAccelerable(const World& world, BodyID id);

/// @copydoc World::IsImpenetrable
/// @relatedalso World
bool IsImpenetrable(const World& world, BodyID id);

/// @brief Sets the impenetrable status of the identified body.
/// @relatedalso World
void SetImpenetrable(World& world, BodyID id);

/// @brief Unsets the impenetrable status of the identified body.
/// @relatedalso World
void UnsetImpenetrable(World& world, BodyID id);

/// @brief Convenience function that sets/unsets the impenetrable status of the identified body.
/// @relatedalso World
inline void SetImpenetrable(World& world, BodyID id, bool value)
{
    if (value)
        SetImpenetrable(world, id);
    else
        UnsetImpenetrable(world, id);
}

/// @brief Gets whether the identified body is allowed to sleep.
/// @relatedalso World
bool IsSleepingAllowed(const World& world, BodyID id);

/// @brief Sets whether the identified body is allowed to sleep.
/// @relatedalso World
void SetSleepingAllowed(World& world, BodyID, bool value);

/// @brief Gets the container of all contacts attached to the identified body.
/// @warning This collection changes during the time step and you may
///   miss some collisions if you don't use <code>ContactListener</code>.
/// @relatedalso World
SizedRange<std::vector<KeyedContactPtr>::const_iterator>
GetContacts(const World& world, BodyID id);

/// @brief Gets the centripetal force necessary to put the body into an orbit having
///    the given radius.
/// @relatedalso World
Force2 GetCentripetalForce(const World& world, BodyID id, Length2 axis);

/// @brief Applies a force to the center of mass of the given body.
/// @note Non-zero forces wakes up the body.
/// @param id Body to apply the force to.
/// @param force World force vector.
/// @relatedalso World
inline void ApplyForceToCenter(World& world, BodyID id, Force2 force)
{
    const auto linAccel = GetLinearAcceleration(world, id) + force * GetInvMass(world, id);
    const auto angAccel = GetAngularAcceleration(world, id);
    SetAcceleration(world, id, linAccel, angAccel);
}

/// @brief Apply a force at a world point.
/// @note If the force is not applied at the center of mass, it will generate a torque and
///   affect the angular velocity.
/// @note Non-zero forces wakes up the body.
/// @param world World in which body exists.
/// @param id Identity of body to apply the force to.
/// @param force World force vector.
/// @param point World position of the point of application.
/// @relatedalso World
void ApplyForce(World& world, BodyID id, Force2 force, Length2 point);

/// @brief Applies a torque.
/// @note This affects the angular velocity without affecting the linear velocity of the
///   center of mass.
/// @note Non-zero forces wakes up the body.
/// @param id Body to apply the torque to.
/// @param torque about the z-axis (out of the screen).
/// @relatedalso World
void ApplyTorque(World& world, BodyID id, Torque torque);

/// @brief Applies an impulse at a point.
/// @note This immediately modifies the velocity.
/// @note This also modifies the angular velocity if the point of application
///   is not at the center of mass.
/// @note Non-zero impulses wakes up the body.
/// @param id Body to apply the impulse to.
/// @param impulse the world impulse vector.
/// @param point the world position of the point of application.
/// @relatedalso World
void ApplyLinearImpulse(World& world, BodyID id, Momentum2 impulse, Length2 point);

/// @brief Applies an angular impulse.
/// @param id Body to apply the angular impulse to.
/// @param impulse Angular impulse to be applied.
/// @relatedalso World
void ApplyAngularImpulse(World& world, BodyID id, AngularMomentum impulse);

/// @brief Sets the given amount of force at the given point to the given body.
/// @relatedalso World
inline void SetForce(World& world, BodyID id, Force2 force, Length2 point) noexcept
{
    const auto linAccel = LinearAcceleration2{force * GetInvMass(world, id)};
    const auto invRotI = GetInvRotInertia(world, id);
    const auto dp = point - GetWorldCenter(world, id);
    const auto cp = Torque{Cross(dp, force) / Radian};
    const auto angAccel = AngularAcceleration{cp * invRotI};
    SetAcceleration(world, id, linAccel, angAccel);
}

/// @brief Sets the given amount of torque to the given body.
/// @relatedalso World
inline void SetTorque(World& world, BodyID id, Torque torque) noexcept
{
    const auto linAccel = GetLinearAcceleration(world, id);
    const auto invRotI = GetInvRotInertia(world, id);
    const auto angAccel = torque * invRotI;
    SetAcceleration(world, id, linAccel, angAccel);
}

/// @brief Gets the linear damping of the body.
/// @relatedalso World
Frequency GetLinearDamping(const World& world, BodyID id);

/// @brief Sets the linear damping of the body.
/// @relatedalso World
void SetLinearDamping(World& world, BodyID id, NonNegative<Frequency> linearDamping);

/// @brief Gets the angular damping of the body.
/// @relatedalso World
Frequency GetAngularDamping(const World& world, BodyID id);

/// @brief Sets the angular damping of the body.
/// @relatedalso World
void SetAngularDamping(World& world, BodyID id, NonNegative<Frequency> angularDamping);

/// @brief Gets the count of awake bodies in the given world.
/// @relatedalso World
BodyCounter GetAwakeCount(const World& world) noexcept;

/// @brief Awakens all of the bodies in the given world.
/// @details Calls all of the world's bodies' <code>SetAwake</code> method.
/// @return Sum total of calls to bodies' <code>SetAwake</code> method that returned true.
/// @see Body::SetAwake.
/// @relatedalso World
BodyCounter Awaken(World& world) noexcept;

/// @brief Finds body in given world that's closest to the given location.
/// @relatedalso World
BodyID FindClosestBody(const World& world, Length2 location) noexcept;

/// @brief Gets the body count in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline BodyCounter GetBodyCount(const World& world) noexcept
{
    using std::size;
    return static_cast<BodyCounter>(size(GetBodies(world)));
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
    SetAccelerations(world, Acceleration{});
}

/// @brief Sets the accelerations of all the world's bodies.
/// @param world World instance to set the acceleration of all contained bodies for.
/// @param fn Function or functor with a signature like:
///   <code>Acceleration (*fn)(const Body& body)</code>.
/// @relatedalso World
template <class F>
void SetAccelerations(World& world, F fn)
{
    const auto bodies = GetBodies(world);
    std::for_each(begin(bodies), end(bodies), [&](const auto &b) {
        SetAcceleration(world, b, fn(world, b));
    });
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDBODY_HPP
