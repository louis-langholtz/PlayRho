/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

/**
 * @file
 * @brief Types and default settings file.
 */

#ifndef BOX2D_SETTINGS_H
#define BOX2D_SETTINGS_H

#include <cstddef>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <string>
#include <limits>
#include <cstdint>
#include <algorithm>

// #define USE_BOOST_UNITS
#ifdef USE_BOOST_UNITS
#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/frequency.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/mass.hpp>
#include <boost/units/systems/si/momentum.hpp>
#include <boost/units/systems/si/inverse_mass.hpp>
#include <boost/units/systems/si/area.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/angular_momentum.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <boost/units/systems/si/second_moment_of_area.hpp>
#include <boost/units/systems/si/surface_density.hpp>
#include <boost/units/systems/si/moment_of_inertia.hpp>
#include <boost/units/systems/si/inverse_moment_of_inertia.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#endif

#include <Box2D/Defines.hpp>
#include <Box2D/Common/RealNum.hpp>
#include <Box2D/Common/Wider.hpp>

/**
 * @namespace box2d
 *
 * Namespace for all Box2D related names.
 */
namespace box2d
{
template<class... T> void NOT_USED(T&&...){}
    
class Body;
class Contact;
class Joint;

/// @brief Pi.
///
/// @details
/// While the include file definition of M_PI may be a POSIX compliance requirement
/// and initially attractive to use, it's apparently not a C++ standards requirement
/// and casually including it pollutes the namespace of all code that uses this library.
/// Whatever the case, MSVS2017 doesn't make it part of the cmath include without enabling
///  _USE_MATH_DEFINES. So rather than add yet more C-preprocessor macros to all
/// sources that this library may be compiled with, it's simply hard-coded in here
/// instead using a C++ mechanism that also keeps it with the enclosing namespace.
///
/// @note Any narrowing is intentional.
///
constexpr auto Pi = Real(3.14159265358979323846264338327950288);

#ifdef USE_BOOST_UNITS

using Time = boost::units::quantity<boost::units::si::time, Real>;
constexpr auto Second = Time{boost::units::si::second * Real{1}};

using Frequency = boost::units::quantity<boost::units::si::frequency, Real>;
constexpr auto Hertz = Frequency{boost::units::si::hertz * Real{1}};

using Length = boost::units::quantity<boost::units::si::length, Real>;
constexpr auto Meter = Length{boost::units::si::meter * Real{1}};

using LinearVelocity = boost::units::quantity<boost::units::si::velocity, Real>;
constexpr auto MeterPerSecond = LinearVelocity{boost::units::si::meter_per_second * Real{1}};

using LinearAcceleration = boost::units::quantity<boost::units::si::acceleration, Real>;
constexpr auto MeterPerSquareSecond = LinearAcceleration{boost::units::si::meter_per_second_squared * Real{1}};

using Mass = boost::units::quantity<boost::units::si::mass, Real>;
constexpr auto Kilogram = Mass{boost::units::si::kilogram * Real{1}};

using InvMass = boost::units::quantity<boost::units::si::inverse_mass, Real>;

using Area = boost::units::quantity<boost::units::si::area, Real>;
constexpr auto SquareMeter = Area{boost::units::si::square_meter * Real{1}};

using Density = boost::units::quantity<boost::units::si::surface_density, Real>;
constexpr auto KilogramPerSquareMeter = Density{boost::units::si::kilogram_per_square_meter * Real{1}};

using Angle = boost::units::quantity<boost::units::si::plane_angle, Real>;
constexpr auto Radian = Angle{boost::units::si::radian * Real{1}};
constexpr auto Degree = Angle{boost::units::degree::degree * Real{1}};
constexpr auto SquareRadian = Radian * Radian;

using AngularVelocity = boost::units::quantity<boost::units::si::angular_velocity, Real>;
constexpr auto RadianPerSecond = AngularVelocity{boost::units::si::radian_per_second * Real{1}};
constexpr auto DegreePerSecond = AngularVelocity{RadianPerSecond * Degree / Radian};

using AngularAcceleration = boost::units::quantity<boost::units::si::angular_acceleration, Real>;
constexpr auto RadianPerSquareSecond = Radian / (Second * Second);

using Force = boost::units::quantity<boost::units::si::force, Real>;
constexpr auto Newton = Force{boost::units::si::newton * Real{1}};

using Torque = boost::units::quantity<boost::units::si::torque, Real>;
constexpr auto NewtonMeter = Torque{boost::units::si::newton_meter * Real{1}};

using SecondMomentOfArea = boost::units::quantity<boost::units::si::second_moment_of_area, Real>;

using RotInertia = boost::units::quantity<boost::units::si::moment_of_inertia, Real>;
using InvRotInertia = boost::units::quantity<boost::units::si::inverse_moment_of_inertia, Real>;

using Momentum = boost::units::quantity<boost::units::si::momentum, Real>;
constexpr auto NewtonSecond = Newton * Second;

/// @brief Angular momentum.
/// @note Units of L^2 M T^-1 QP^-1.
using AngularMomentum = boost::units::quantity<boost::units::si::angular_momentum, Real>;

#else // USE_BOOST_UNITS

using Time = Real;
constexpr auto Second = Real{1};

using Frequency = Real;
constexpr auto Hertz = Real{1};

using Length = Real;
constexpr auto Meter = Real{1};

using LinearVelocity = Real;
constexpr auto MeterPerSecond = Real{1};

using LinearAcceleration = Real;
constexpr auto MeterPerSquareSecond = Real{1};

using Mass = Real;
constexpr auto Kilogram = Real{1};

using InvMass = Real;

using Area = Real;
constexpr auto SquareMeter = Real{1};

using Density = Real;
constexpr auto KilogramPerSquareMeter = Real{1};

using Angle = Real;
constexpr auto Radian = Real{1};
constexpr auto Degree = Pi / Real{180};
constexpr auto SquareRadian = Radian * Radian;

using AngularVelocity = Real;
constexpr auto RadianPerSecond = Real{1};
constexpr auto DegreePerSecond = Degree;

using AngularAcceleration = Real;
constexpr auto RadianPerSquareSecond = Real{1};

using Force = Real;
constexpr auto Newton = Real{1};

using Torque = Real;
constexpr auto NewtonMeter = Real{1};

using SecondMomentOfArea = Real;

using RotInertia = Real;
using InvRotInertia = Real;

using Momentum = Real;
constexpr auto NewtonSecond = Real{1};

using AngularMomentum = Real;

#endif // USE_BOOST_UNITS

constexpr inline Real StripUnit(const Real value)
{
    return value;
}

#ifdef USE_BOOST_UNITS

constexpr inline Real StripUnit(const Angle value)
{
    return Real{value / Radian};
}

constexpr inline Real StripUnit(const Length value)
{
    return Real{value / Meter};
}

constexpr inline Real StripUnit(const Area value)
{
    return Real{value / SquareMeter};
}

constexpr inline Real StripUnit(const Mass value)
{
    // InvMass has units of M^-1
    return Real{value / Kilogram};
}

constexpr inline Real StripUnit(const InvMass value)
{
    // InvMass has units of M^-1
    return Real{value * Kilogram};
}

constexpr inline Real StripUnit(const RotInertia value)
{
    return Real{value * SquareRadian / (SquareMeter * Kilogram)};
}

constexpr inline Real StripUnit(const InvRotInertia value)
{
    // InvRotInertia has units of L^-2 M^-1 QP^2
    return Real{value * SquareMeter * Kilogram / SquareRadian};
}

constexpr inline Real StripUnit(const Momentum value)
{
    // Momentum has units of M L T^-1
    return Real{value * Second / (Kilogram * Meter)};
}

constexpr inline Real StripUnit(const LinearVelocity value)
{
    return Real{value / MeterPerSecond};
}

constexpr inline Real StripUnit(const AngularVelocity value)
{
    return Real{value / RadianPerSecond};
}

constexpr inline Real StripUnit(const Density value)
{
    return Real{value / KilogramPerSquareMeter};
}

constexpr inline Real StripUnit(const Force value)
{
    // Force has units of Newtons - which are M L T^2
    return Real{value / Newton};
}

constexpr inline Real StripUnit(const Torque value)
{
    return Real{value / NewtonMeter};
}

#endif

/// @brief Max child count.
constexpr auto MaxChildCount = std::numeric_limits<unsigned>::max() - 1;

/// @brief Child counter type.
/// @details Relating to "children" of shape where each child is a convex shape possibly
///   comprising a concave shape.
/// @note This type must always be able to contain the <code>MaxChildCount</code> value.
using ChildCounter = std::remove_const<decltype(MaxChildCount)>::type;

/// Time step iterations type.
/// @details A type for countining iterations per time-step.
using ts_iters_t = std::uint8_t;

constexpr auto MaxFloat = std::numeric_limits<Real>::max(); // FLT_MAX

// Collision

/// Maximum manifold points.
/// This is the maximum number of contact points between two convex shapes.
/// Do not change this value.
/// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxManifoldPoints = std::uint8_t{2};

/// Maximum number of vertices for any shape type.
/// @note For memory efficiency, uses the smallest integral type that can hold the value.
constexpr auto MaxShapeVertices = std::uint8_t{254};

/// Default linear slop.
/// @details
/// Length used as a collision and constraint tolerance.
/// Usually chosen to be numerically significant, but visually insignificant.
/// Lower or raise to decrease or increase respectively the minimum of space
/// between bodies at rest.
/// @note Smaller values relative to sizes of bodies increases the time it takes for bodies to come to rest.
constexpr auto DefaultLinearSlop = Length{Meter / Real{1000}}; // originally 0.005

/// @brief Default AABB extension amount.
constexpr auto DefaultAabbExtension = DefaultLinearSlop * Real{20};

/// Default distance multiplier.
constexpr auto DefaultDistanceMultiplier = Real{2};

/// Default angular slop.
/// @details
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto DefaultAngularSlop = (Pi * Real{2} * Radian) / Real{180};

/// Default maximum linear correction.
/// @details The maximum linear position correction used when solving constraints.
///   This helps to prevent overshoot.
/// @note This value should be greater than the linear slop value.
constexpr auto DefaultMaxLinearCorrection = DefaultLinearSlop * Real{40}; // aka 0.04f

/// Default maximum angular correction.
/// @note This value should be greater than the angular slop value.
constexpr auto DefaultMaxAngularCorrection = DefaultAngularSlop * Real{4};

/// Default maximum time of impact iterations.
constexpr auto DefaultMaxToiIters = std::uint8_t{20};

/// Default maximum time of impact root iterator count.
constexpr auto DefaultMaxToiRootIters = std::uint8_t{30};

/// Default max number of distance iterations.
constexpr auto DefaultMaxDistanceIters = std::uint8_t{20};

/// Default maximum number of sub steps.
/// @details
/// This is the default maximum number of sub-steps per contact in continuous physics simulation.
/// In other words, this is the default maximum number of times in a world step that a contact will
/// have continuous collision resolution done for it.
/// @note Used in the TOI phase of step processing.
constexpr auto DefaultMaxSubSteps = std::uint8_t{48};
    
// Dynamics

/// @brief Default velocity threshold.
constexpr auto DefaultVelocityThreshold = (Real{8} / Real{10}) * MeterPerSecond;

/// @brief Maximum number of bodies in a world.
/// @note This is 65534 based off std::uint16_t and eliminating one value for invalid.
constexpr auto MaxBodies = static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() -
                                                      std::uint16_t{1});

/// @brief Body count type.
/// @note This type must always be able to contain the <code>MaxBodies</code> value.
using BodyCounter = std::remove_const<decltype(MaxBodies)>::type;

/// @brief Contact count type.
/// @note This type must be able to contain the squared value of <code>BodyCounter</code>.
using ContactCounter = Wider<BodyCounter>::type;

/// @brief Invalid contact index.
constexpr auto InvalidContactIndex = static_cast<ContactCounter>(-1);

/// @brief Maximum number of contacts in a world (2147319811).
/// @details Uses the formula for the maximum number of edges in an undirectional graph of MaxBodies nodes. 
/// This occurs when every possible body is connected to every other body.
constexpr auto MaxContacts = ContactCounter{MaxBodies} * ContactCounter{MaxBodies - 1} / ContactCounter{2};

/// @brief Maximum number of joints in a world.
/// @note This is 65534 based off std::uint16_t and eliminating one value for invalid.
constexpr auto MaxJoints = static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() -
                                                      std::uint16_t{1});

/// @brief Joint count type.
/// @note This type must be able to contain the <code>MaxJoints</code> value.
using JointCounter = std::remove_const<decltype(MaxJoints)>::type;

/// @brief Default step time.
constexpr auto DefaultStepTime = Time{Second / Real{60}};

/// @brief Default step frequency.
constexpr auto DefaultStepFrequency = Frequency{Hertz * Real{60}};

// Sleep

/// Default minimum still time to sleep.
/// @details The default minimum time bodies must be still for bodies to be put to sleep.
constexpr auto DefaultMinStillTimeToSleep = Time{Second / Real{2}}; // aka 0.5 secs

/// Default linear sleep tolerance.
/// @details A body cannot sleep if the magnitude of its linear velocity is above this amount.
constexpr auto DefaultLinearSleepTolerance = Real{0.01f} * MeterPerSecond; // aka 0.01

/// Default angular sleep tolerance.
/// @details A body cannot sleep if its angular velocity is above this amount.
constexpr auto DefaultAngularSleepTolerance = Real{(Pi * 2) / 180} * RadianPerSecond;

/// Default circles ratio.
/// @details Ratio used for switching between rounded-corner collisions and closest-face
///   biased normal collisions.
constexpr auto DefaultCirclesRatio = Real{10};

/// Maximum list size.
template <typename T>
constexpr std::size_t max_list_size();

template <>
constexpr std::size_t max_list_size<Body>()
{
    return MaxBodies;
}

template <>
constexpr std::size_t max_list_size<Contact>()
{
    return MaxContacts;
}

template <>
constexpr std::size_t max_list_size<Joint>()
{
    return MaxJoints;
}

// GetInvalid template and template specializations.

template <typename T>
constexpr inline T GetInvalid() noexcept;

template <>
constexpr float GetInvalid() noexcept
{
    return std::numeric_limits<float>::signaling_NaN();
}

template <>
constexpr double GetInvalid() noexcept
{
    return std::numeric_limits<double>::signaling_NaN();
}

template <>
constexpr long double GetInvalid() noexcept
{
    return std::numeric_limits<long double>::signaling_NaN();
}

template <>
constexpr Fixed32 GetInvalid() noexcept
{
    return Fixed32::GetNaN();
}

#ifndef _WIN32
template <>
constexpr Fixed64 GetInvalid() noexcept
{
    return Fixed64::GetNaN();
}
#endif

template <>
constexpr std::size_t GetInvalid() noexcept
{
    return static_cast<std::size_t>(-1);
}

// IsValid template and template specializations.

template <typename T>
constexpr inline bool IsValid(const T& value) noexcept
{
    // Note: This is not necessarily a no-op!! But it is a "constexpr".
    //
    // From http://en.cppreference.com/w/cpp/numeric/math/isnan:
    //   "Another way to test if a floating-point value is NaN is
    //    to compare it with itself:
    //      bool is_nan(double x) { return x != x; }
    //
    // So for all T, for which std::isnan() is implemented, this should work
    // correctly and quite usefully!
    //
    return value == value;
}

template <>
constexpr inline bool IsValid(const std::size_t& x) noexcept
{
    return x != GetInvalid<std::size_t>();
}

#ifdef USE_BOOST_UNITS

template <>
constexpr Angle GetInvalid() noexcept
{
    return GetInvalid<Real>() * Radian;
}

template <>
constexpr inline bool IsValid(const Angle& x) noexcept
{
    return IsValid(Real{x / Radian});
}

template <>
constexpr Frequency GetInvalid() noexcept
{
    return GetInvalid<Real>() * Hertz;
}

template <>
constexpr inline bool IsValid(const Frequency& x) noexcept
{
    return IsValid(Real{x / Hertz});
}

template <>
constexpr AngularVelocity GetInvalid() noexcept
{
    return GetInvalid<Real>() * RadianPerSecond;
}

template <>
constexpr inline bool IsValid(const AngularVelocity& x) noexcept
{
    return IsValid(Real{x / RadianPerSecond});
}

template <>
constexpr Time GetInvalid() noexcept
{
    return GetInvalid<Real>() * Second;
}

template <>
constexpr inline bool IsValid(const Time& x) noexcept
{
    return IsValid(Real{x / Second});
}

template <>
constexpr Length GetInvalid() noexcept
{
    return GetInvalid<Real>() * Meter;
}

template <>
constexpr inline bool IsValid(const Length& x) noexcept
{
    return IsValid(Real{x / Meter});
}

template <>
constexpr Mass GetInvalid() noexcept
{
    return GetInvalid<Real>() * Kilogram;
}

template <>
constexpr inline bool IsValid(const Mass& x) noexcept
{
    return IsValid(Real{x / Kilogram});
}

template <>
constexpr InvMass GetInvalid() noexcept
{
    return GetInvalid<Real>() / Kilogram;
}

template <>
constexpr inline bool IsValid(const InvMass& x) noexcept
{
    return IsValid(Real{x * Kilogram});
}

template <>
constexpr Momentum GetInvalid() noexcept
{
    return GetInvalid<Real>() * Kilogram * MeterPerSecond;
}

template <>
constexpr inline bool IsValid(const Momentum& x) noexcept
{
    return IsValid(Real{x / (Kilogram * MeterPerSecond)});
}

template <>
constexpr Force GetInvalid() noexcept
{
    return GetInvalid<Real>() * Newton;
}

template <>
constexpr inline bool IsValid(const Force& x) noexcept
{
    return IsValid(Real{x / Newton});
}

template <>
constexpr Torque GetInvalid() noexcept
{
    return GetInvalid<Real>() * NewtonMeter;
}

template <>
constexpr inline bool IsValid(const Torque& x) noexcept
{
    return IsValid(Real{x / NewtonMeter});
}

template <>
constexpr LinearVelocity GetInvalid() noexcept
{
    return GetInvalid<Real>() * MeterPerSecond;
}

template <>
constexpr inline bool IsValid(const LinearVelocity& x) noexcept
{
    return IsValid(Real{x / MeterPerSecond});
}

template <>
constexpr LinearAcceleration GetInvalid() noexcept
{
    return GetInvalid<Real>() * MeterPerSquareSecond;
}

template <>
constexpr inline bool IsValid(const LinearAcceleration& x) noexcept
{
    return IsValid(Real{x / MeterPerSquareSecond});
}

template <>
constexpr AngularAcceleration GetInvalid() noexcept
{
    return GetInvalid<Real>() * RadianPerSquareSecond;
}

template <>
constexpr inline bool IsValid(const AngularAcceleration& x) noexcept
{
    return IsValid(Real{x / RadianPerSquareSecond});
}

template <>
constexpr RotInertia GetInvalid() noexcept
{
    // RotInertia is L^2  M    QP^-2
    return GetInvalid<Real>() * SquareMeter * Kilogram / SquareRadian;
}

template <>
constexpr inline bool IsValid(const RotInertia& value) noexcept
{
    return IsValid(Real{value / (SquareMeter * Kilogram / SquareRadian)});
}

#endif

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* Alloc(std::size_t size);

template <typename T>
T* Alloc(std::size_t size)
{
    return static_cast<T*>(Alloc(size * sizeof(T)));
}
    
/// Implement this function to use your own memory allocator.
void* Realloc(void* ptr, std::size_t new_size);

template <typename T>
T* Realloc(T* ptr, std::size_t size)
{
    return static_cast<T*>(Realloc(static_cast<void *>(ptr), size * sizeof(T)));
}

/// If you implement Alloc, you should also implement this function.
void Free(void* mem);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct Version
{
    using revnum_type = std::int32_t;

    revnum_type major;        ///< significant changes
    revnum_type minor;        ///< incremental changes
    revnum_type revision;        ///< bug fixes
};

constexpr inline bool operator== (Version lhs, Version rhs)
{
    return lhs.major == rhs.major && lhs.minor == rhs.minor && lhs.revision == rhs.revision;
}

constexpr inline bool operator!= (Version lhs, Version rhs)
{
    return !(lhs == rhs);
}

Version GetVersion() noexcept;

std::string GetBuildDetails() noexcept;

}

#endif
