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

#ifndef BOX2D_SETTINGS_H
#define BOX2D_SETTINGS_H

#include <cstddef>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <limits>
#include <cstdint>
#include <algorithm>

#include <Box2D/Common/Wider.hpp>
#include <Box2D/Common/Fixed.hpp>

namespace box2d
{
template<class... T> void NOT_USED(T&&...){}
	
class Body;
class Contact;
class Joint;
	
using int8 = std::int8_t;
using int16 = std::int16_t;
using int32 = std::int32_t;
using uint8 = std::uint8_t;
using uint16 = std::uint16_t;
using uint32 = std::uint32_t;
using uint64 = std::uint64_t;

/// Real-number type.
///
/// @detail This is the number type underlying numerical calculations conceptually involving
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

/// Child count type. @detail Relating to "children" of Shape.
using child_count_t = unsigned;

/// Size type for sizing data.
using size_t = std::size_t;

/// Island count type. @detail Relating to items in a Island.
using island_count_t = size_t;

/// Time step iterations type.
/// @detail A type for countining iterations per time-step.
using ts_iters_t = uint8;

constexpr auto MaxFloat = std::numeric_limits<RealNum>::max(); // FLT_MAX

constexpr auto Pi = static_cast<float>(M_PI); ///< Pi (any narrowing is intentional).

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// Maximum manifold points.
/// This is the maximum number of contact points between two convex shapes.
/// Do not change this value.
/// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxManifoldPoints = uint8{2};

/// Maximum number of vertices for any shape type.
/// @note For memory efficiency, uses the smallest integral type that can hold the value.
constexpr auto MaxShapeVertices = uint8{254};

/// Default linear slop.
/// @detail
/// Length used as a collision and constraint tolerance.
/// Usually chosen to be numerically significant, but visually insignificant.
/// Lower or raise to decrease or increase respectively the minimum of space
/// between bodies at rest.
/// @note Smaller values relative to sizes of bodies increases the time it takes for bodies to come to rest.
constexpr auto DefaultLinearSlop = RealNum(0.001f); // originally 0.005

/// Default angular slop.
/// @detail
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto DefaultAngularSlop = (Pi * 2) / 180;

/// Default maximum linear correction.
/// @detail The maximum linear position correction used when solving constraints.
///   This helps to prevent overshoot.
/// @note This value should be greater than the linear slop value.
constexpr auto DefaultMaxLinearCorrection = DefaultLinearSlop * 40; // aka 0.04f

/// Default maximum angular correction.
/// @note This value should be greater than the angular slop value.
constexpr auto DefaultMaxAngularCorrection = DefaultAngularSlop * 4;

/// Default maximum time of impact iterations.
constexpr auto DefaultMaxToiIters = uint8{20};

/// Default maximum time of impact root iterator count.
constexpr auto DefaultMaxToiRootIters = uint8{30};

/// Default max number of distance iterations.
constexpr auto DefaultMaxDistanceIters = uint8{20};

/// Default maximum number of sub steps.
/// @detail
/// This is the default maximum number of sub-steps per contact in continuous physics simulation.
/// In other words, this is the default maximum number of times in a world step that a contact will
/// have continuous collision resolution done for it.
/// @note Used in the TOI phase of step processing.
constexpr auto DefaultMaxSubSteps = uint8{48};
	
// Dynamics

/// Default velocity threshold.
constexpr auto DefaultVelocityThreshold = RealNum{8} / 10; // RealNum{1}

/// Maximum number of bodies in a world (65534 based off uint16 and eliminating one value for invalid).
constexpr auto MaxBodies = uint16{std::numeric_limits<uint16>::max() - uint16{1}};

/// Body count type.
using body_count_t = std::remove_const<decltype(MaxBodies)>::type;

/// Contact count type.
using contact_count_t = Wider<body_count_t>::type;
	
/// Maximum number of contacts in a world (2147319811).
/// @detail Uses the formula for the maximum number of edges in an undirectional graph of MaxBodies nodes. 
/// This occurs when every possible body is connected to every other body.
constexpr auto MaxContacts = contact_count_t{MaxBodies} * contact_count_t{MaxBodies - 1} / contact_count_t{2};

/// Joint count type.
using joint_count_t = Wider<body_count_t>::type;

/// Maximum number of joints in a world (65534 based off uint16 and eliminating one value for invalid).
constexpr auto MaxJoints = uint16{std::numeric_limits<uint16>::max() - uint16{1}};

// Sleep

/// Default minimum still time to sleep.
/// @detail The default minimum time bodies must be still for bodies to be put to sleep.
constexpr auto DefaultMinStillTimeToSleep = RealNum{1} / 2; // aka 0.5

/// Default linear sleep tolerance.
/// @detail A body cannot sleep if the magnitude of its linear velocity is above this amount.
constexpr auto DefaultLinearSleepTolerance = RealNum{0.01f}; // aka 0.01

/// Default angular sleep tolerance.
/// @detail A body cannot sleep if its angular velocity is above this amount.
constexpr auto DefaultAngularSleepTolerance = RealNum{(Pi * 2) / 180};

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

template <>
constexpr Fixed64 GetInvalid() noexcept
{
	return Fixed64::GetNaN();
}

template <>
constexpr size_t GetInvalid() noexcept
{
	return static_cast<size_t>(-1);
}

// IsValid template and template specializations.

template <typename T>
bool IsValid(const T& value) noexcept;

/// This function is used to ensure that a floating point number is not a NaN.
template <>
inline bool IsValid(const float& x) noexcept
{
	return !std::isnan(x); // && !std::isinf(x);
}

/// This function is used to ensure that a double precision floating point number is not NaN.
template <>
inline bool IsValid(const double& x) noexcept
{
	return !std::isnan(x); // && !std::isinf(x);
}

/// This function is used to ensure that a long double precision floating point number is not NaN.
template <>
inline bool IsValid(const long double& x) noexcept
{
	return !std::isnan(x); // && !std::isinf(x);
}

template <>
inline bool IsValid(const Fixed32& x) noexcept
{
	return !std::isnan(x);
}

template <>
inline bool IsValid(const Fixed64& x) noexcept
{
	return !std::isnan(x);
}

template <>
inline bool IsValid(const size_t& x) noexcept
{
	return x != GetInvalid<size_t>();
}

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
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
};

constexpr auto BuiltVersion = Version{3, 0, 0};
}

#endif
