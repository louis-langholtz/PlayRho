/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
#include <Box2D/Common/Wider.hpp>
#include <Box2D/Common/Fixed.hpp>

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

/// Real-number type.
///
/// @details This is the number type underlying numerical calculations conceptually involving
/// real-numbers. Ideally the implementation of this type doesn't suffer from things like:
/// catastrophic cancellation, catastrophic division, overflows, nor underflows.
///
/// @note This can be implemented using float, double, long double, Fixed32, or Fixed64.
///
/// @note Regarding division:
///
/// While dividing 1 by a RealNum, caching the result, and then doing multiplications with the
/// result may well be faster (than repeatedly dividing), dividing 1 by RealNum can also result
/// in an underflow situation that's then compounded every time it's multiplied with other
/// values.
///
/// Meanwhile, dividing every value by RealNum isolates any underflows to the particular
/// division where underflow occurs.
///
/// @warning The note regarding division applies even more so when using a fixed-point type
/// (for RealNum).
///
using RealNum = float;

/// Pi.
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
constexpr auto Pi = RealNum(3.14159265358979323846264338327950288);

#ifdef USE_BOOST_UNITS

using Time = boost::units::quantity<boost::units::si::time, RealNum>;
constexpr auto Second = Time{boost::units::si::second * RealNum{1}};

using Frequency = boost::units::quantity<boost::units::si::frequency, RealNum>;
constexpr auto Hertz = Frequency{boost::units::si::hertz * RealNum{1}};

using Length = boost::units::quantity<boost::units::si::length, RealNum>;
constexpr auto Meter = Length{boost::units::si::meter * RealNum{1}};

using LinearVelocity = boost::units::quantity<boost::units::si::velocity, RealNum>;
constexpr auto MeterPerSecond = LinearVelocity{boost::units::si::meter_per_second * RealNum{1}};

using LinearAcceleration = boost::units::quantity<boost::units::si::acceleration, RealNum>;
constexpr auto MeterPerSquareSecond = LinearAcceleration{boost::units::si::meter_per_second_squared * RealNum{1}};

using Mass = boost::units::quantity<boost::units::si::mass, RealNum>;
constexpr auto Kilogram = Mass{boost::units::si::kilogram * RealNum{1}};

using InvMass = boost::units::quantity<boost::units::si::inverse_mass, RealNum>;

using Area = boost::units::quantity<boost::units::si::area, RealNum>;
constexpr auto SquareMeter = Area{boost::units::si::square_meter * RealNum{1}};

using Density = boost::units::quantity<boost::units::si::surface_density, RealNum>;
constexpr auto KilogramPerSquareMeter = Density{boost::units::si::kilogram_per_square_meter * RealNum{1}};

using Angle = boost::units::quantity<boost::units::si::plane_angle, RealNum>;
constexpr auto Radian = Angle{boost::units::si::radian * RealNum{1}};
constexpr auto Degree = Angle{boost::units::degree::degree * RealNum{1}};
constexpr auto SquareRadian = Radian * Radian;

using AngularVelocity = boost::units::quantity<boost::units::si::angular_velocity, RealNum>;
constexpr auto RadianPerSecond = AngularVelocity{boost::units::si::radian_per_second * RealNum{1}};
	
using AngularAcceleration = boost::units::quantity<boost::units::si::angular_acceleration, RealNum>;
constexpr auto RadianPerSquareSecond = Radian / (Second * Second);

using Force = boost::units::quantity<boost::units::si::force, RealNum>;
constexpr auto Newton = Force{boost::units::si::newton * RealNum{1}};

using Torque = boost::units::quantity<boost::units::si::torque, RealNum>;
constexpr auto NewtonMeter = Torque{boost::units::si::newton_meter * RealNum{1}};

using SecondMomentOfArea = boost::units::quantity<boost::units::si::second_moment_of_area, RealNum>;

using RotInertia = boost::units::quantity<boost::units::si::moment_of_inertia, RealNum>;
using InvRotInertia = boost::units::quantity<boost::units::si::inverse_moment_of_inertia, RealNum>;

using Momentum = boost::units::quantity<boost::units::si::momentum, RealNum>;
constexpr auto NewtonSecond = Newton * Second;

using AngularMomentum = boost::units::quantity<boost::units::si::angular_momentum, RealNum>;

#else // USE_BOOST_UNITS

using Time = RealNum;
constexpr auto Second = RealNum{1};

using Frequency = RealNum;
constexpr auto Hertz = RealNum{1};

using Length = RealNum;
constexpr auto Meter = RealNum{1};

using LinearVelocity = RealNum;
constexpr auto MeterPerSecond = RealNum{1};

using LinearAcceleration = RealNum;
constexpr auto MeterPerSquareSecond = RealNum{1};

using Mass = RealNum;
constexpr auto Kilogram = RealNum{1};

using InvMass = RealNum;

using Area = RealNum;
constexpr auto SquareMeter = RealNum{1};

using Density = RealNum;
constexpr auto KilogramPerSquareMeter = RealNum{1};

using Angle = RealNum;
constexpr auto Radian = RealNum{1};
constexpr auto Degree = Pi / RealNum{180};
constexpr auto SquareRadian = Radian * Radian;

using AngularVelocity = RealNum;
constexpr auto RadianPerSecond = RealNum{1};

using AngularAcceleration = RealNum;
constexpr auto RadianPerSquareSecond = RealNum{1};

using Force = RealNum;
constexpr auto Newton = RealNum{1};

using Torque = RealNum;
constexpr auto NewtonMeter = RealNum{1};

using SecondMomentOfArea = RealNum;

using RotInertia = RealNum;
using InvRotInertia = RealNum;

using Momentum = RealNum;
constexpr auto NewtonSecond = RealNum{1};

using AngularMomentum = RealNum;

#endif // USE_BOOST_UNITS

constexpr inline RealNum StripUnit(const RealNum value)
{
	return value;
}

#ifdef USE_BOOST_UNITS

constexpr inline RealNum StripUnit(const Angle value)
{
	return RealNum{value / Radian};
}

constexpr inline RealNum StripUnit(const Length value)
{
	return RealNum{value / Meter};
}

constexpr inline RealNum StripUnit(const Area value)
{
	return RealNum{value / SquareMeter};
}

constexpr inline RealNum StripUnit(const Mass value)
{
	// InvMass has units of M^-1
	return RealNum{value / Kilogram};
}

constexpr inline RealNum StripUnit(const InvMass value)
{
	// InvMass has units of M^-1
	return RealNum{value * Kilogram};
}

constexpr inline RealNum StripUnit(const RotInertia value)
{
	return RealNum{value * SquareRadian / (SquareMeter * Kilogram)};
}

constexpr inline RealNum StripUnit(const InvRotInertia value)
{
	// InvRotInertia has units of L^-2 M^-1 QP^2
	return RealNum{value * SquareMeter * Kilogram / SquareRadian};
}

constexpr inline RealNum StripUnit(const Momentum value)
{
	// Momentum has units of M L T^-1
	return RealNum{value * Second / (Kilogram * Meter)};
}

constexpr inline RealNum StripUnit(const LinearVelocity value)
{
	return RealNum{value / MeterPerSecond};
}

constexpr inline RealNum StripUnit(const AngularVelocity value)
{
	return RealNum{value / RadianPerSecond};
}

constexpr inline RealNum StripUnit(const Force value)
{
	// Force has units of Newtons - which are M L T^2
	return RealNum{value / Newton};
}

constexpr inline RealNum StripUnit(const Torque value)
{
	return RealNum{value / NewtonMeter};
}

#endif

/// Child count type. @details Relating to "children" of Shape.
using child_count_t = unsigned;

/// Size type for sizing data.
using size_t = std::size_t;

/// Island count type. @details Relating to items in a Island.
using island_count_t = size_t;

/// Time step iterations type.
/// @details A type for countining iterations per time-step.
using ts_iters_t = std::uint8_t;

constexpr auto MaxFloat = std::numeric_limits<RealNum>::max(); // FLT_MAX

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
constexpr auto DefaultLinearSlop = Length{Meter / RealNum{1000}}; // originally 0.005

constexpr auto DefaultAabbExtension = DefaultLinearSlop * RealNum{20};

constexpr auto DefaultDistanceMultiplier = RealNum{2};

/// Default angular slop.
/// @details
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto DefaultAngularSlop = (Pi * RealNum{2} * Radian) / RealNum{180};

/// Default maximum linear correction.
/// @details The maximum linear position correction used when solving constraints.
///   This helps to prevent overshoot.
/// @note This value should be greater than the linear slop value.
constexpr auto DefaultMaxLinearCorrection = DefaultLinearSlop * RealNum{40}; // aka 0.04f

/// Default maximum angular correction.
/// @note This value should be greater than the angular slop value.
constexpr auto DefaultMaxAngularCorrection = DefaultAngularSlop * RealNum{4};

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

/// Default velocity threshold.
constexpr auto DefaultVelocityThreshold = (RealNum{8} / RealNum{10}) * MeterPerSecond;

/// Maximum number of bodies in a world (65534 based off uint16_t and eliminating one value for invalid).
constexpr auto MaxBodies = static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() -
													  std::uint16_t{1});

/// Body count type.
using body_count_t = std::remove_const<decltype(MaxBodies)>::type;

/// Contact count type.
using contact_count_t = Wider<body_count_t>::type;
	
/// Maximum number of contacts in a world (2147319811).
/// @details Uses the formula for the maximum number of edges in an undirectional graph of MaxBodies nodes. 
/// This occurs when every possible body is connected to every other body.
constexpr auto MaxContacts = contact_count_t{MaxBodies} * contact_count_t{MaxBodies - 1} / contact_count_t{2};

/// Maximum number of joints in a world (65534 based off std::uint16_t and eliminating one value for invalid).
constexpr auto MaxJoints = static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() -
													  std::uint16_t{1});

/// Joint count type.
using joint_count_t = std::remove_const<decltype(MaxJoints)>::type;

constexpr auto DefaultStepTime = Time{Second / RealNum{60}};
constexpr auto DefaultStepFrequency = Frequency{Hertz * RealNum{60}};

// Sleep

/// Default minimum still time to sleep.
/// @details The default minimum time bodies must be still for bodies to be put to sleep.
constexpr auto DefaultMinStillTimeToSleep = Time{Second / RealNum{2}}; // aka 0.5 secs

/// Default linear sleep tolerance.
/// @details A body cannot sleep if the magnitude of its linear velocity is above this amount.
constexpr auto DefaultLinearSleepTolerance = RealNum{0.01f} * MeterPerSecond; // aka 0.01

/// Default angular sleep tolerance.
/// @details A body cannot sleep if its angular velocity is above this amount.
constexpr auto DefaultAngularSleepTolerance = RealNum{(Pi * 2) / 180} * RadianPerSecond;

/// Maximum list size.
template <typename T>
constexpr size_t max_list_size();

template <>
constexpr size_t max_list_size<Body>()
{
	return MaxBodies;
}

template <>
constexpr size_t max_list_size<Contact>()
{
	return MaxContacts;
}

template <>
constexpr size_t max_list_size<Joint>()
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
constexpr size_t GetInvalid() noexcept
{
	return static_cast<size_t>(-1);
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
constexpr inline bool IsValid(const size_t& x) noexcept
{
	return x != GetInvalid<size_t>();
}

#ifdef USE_BOOST_UNITS

template <>
constexpr Angle GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Radian;
}

template <>
constexpr inline bool IsValid(const Angle& x) noexcept
{
	return IsValid(RealNum{x / Radian});
}

template <>
constexpr Frequency GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Hertz;
}

template <>
constexpr inline bool IsValid(const Frequency& x) noexcept
{
	return IsValid(RealNum{x / Hertz});
}

template <>
constexpr AngularVelocity GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * RadianPerSecond;
}

template <>
constexpr inline bool IsValid(const AngularVelocity& x) noexcept
{
	return IsValid(RealNum{x / RadianPerSecond});
}

template <>
constexpr Time GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Second;
}

template <>
constexpr inline bool IsValid(const Time& x) noexcept
{
	return IsValid(RealNum{x / Second});
}

template <>
constexpr Length GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Meter;
}

template <>
constexpr inline bool IsValid(const Length& x) noexcept
{
	return IsValid(RealNum{x / Meter});
}

template <>
constexpr Mass GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Kilogram;
}

template <>
constexpr inline bool IsValid(const Mass& x) noexcept
{
	return IsValid(RealNum{x / Kilogram});
}

template <>
constexpr InvMass GetInvalid() noexcept
{
	return GetInvalid<RealNum>() / Kilogram;
}

template <>
constexpr inline bool IsValid(const InvMass& x) noexcept
{
	return IsValid(RealNum{x * Kilogram});
}

template <>
constexpr Momentum GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Kilogram * MeterPerSecond;
}

template <>
constexpr inline bool IsValid(const Momentum& x) noexcept
{
	return IsValid(RealNum{x / (Kilogram * MeterPerSecond)});
}

template <>
constexpr Force GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * Newton;
}

template <>
constexpr inline bool IsValid(const Force& x) noexcept
{
	return IsValid(RealNum{x / Newton});
}

template <>
constexpr Torque GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * NewtonMeter;
}

template <>
constexpr inline bool IsValid(const Torque& x) noexcept
{
	return IsValid(RealNum{x / NewtonMeter});
}

template <>
constexpr LinearVelocity GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * MeterPerSecond;
}

template <>
constexpr inline bool IsValid(const LinearVelocity& x) noexcept
{
	return IsValid(RealNum{x / MeterPerSecond});
}

template <>
constexpr LinearAcceleration GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * MeterPerSquareSecond;
}

template <>
constexpr inline bool IsValid(const LinearAcceleration& x) noexcept
{
	return IsValid(RealNum{x / MeterPerSquareSecond});
}

template <>
constexpr AngularAcceleration GetInvalid() noexcept
{
	return GetInvalid<RealNum>() * RadianPerSquareSecond;
}

template <>
constexpr inline bool IsValid(const AngularAcceleration& x) noexcept
{
	return IsValid(RealNum{x / RadianPerSquareSecond});
}

template <>
constexpr RotInertia GetInvalid() noexcept
{
	// RotInertia is L^2  M    QP^-2
	return GetInvalid<RealNum>() * SquareMeter * Kilogram / SquareRadian;
}

template <>
constexpr inline bool IsValid(const RotInertia& value) noexcept
{
	return IsValid(RealNum{value / (SquareMeter * Kilogram / SquareRadian)});
}

#endif

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* alloc(size_t size);

template <typename T>
T* alloc(size_t size)
{
	return static_cast<T*>(alloc(size * sizeof(T)));
}
	
/// Implement this function to use your own memory allocator.
void* realloc(void* ptr, size_t new_size);

template <typename T>
T* realloc(T* ptr, size_t size)
{
	return static_cast<T*>(realloc(static_cast<void *>(ptr), size * sizeof(T)));
}

/// If you implement alloc, you should also implement this function.
void free(void* mem);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct Version
{
	using revnum_type = std::int32_t;

	revnum_type major;		///< significant changes
	revnum_type minor;		///< incremental changes
	revnum_type revision;		///< bug fixes
};

constexpr auto BuiltVersion = Version{3, 0, 0};
}

#endif
