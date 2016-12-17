/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef BOX2D_SETTINGS_H
#define BOX2D_SETTINGS_H

#include <cstddef>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <limits>
#include <cstdint>
#include <algorithm>

#define BOX2D_NOT_USED(x) ((void)(x))
#define BOX2D_MAGIC(x) (x)

/// Current version.
#define BOX2D_MAJOR_VERSION 3
#define BOX2D_MINOR_VERSION 0
#define BOX2D_REVISION 0

namespace box2d
{
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

/// Box2D floating point type.
/// This should be float, double, or long double.
using float_t = float;

/// Child count type. @detail Relating to "children" of Shape.
using child_count_t = unsigned;

/// Size type for sizing data.
using size_t = std::size_t;

/// Island count type. @detail Relating to items in a Island.
using island_count_t = size_t;

/// Time step iterations type.
/// @detail A type for countining iterations per time-step.
using ts_iters_t = uint16;

constexpr auto MaxFloat = std::numeric_limits<float_t>::max(); // FLT_MAX

constexpr auto Pi = static_cast<float_t>(M_PI); ///< Pi as the "float_t" float-type (any narrowing is intentional).

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// Maximum manifold points.
/// This is the maximum number of contact points between two convex shapes.
/// Do not change this value.
/// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxManifoldPoints = uint8{2};

/// Maximum number of vertices on a convex polygon.
/// You cannot increase this too much because BlockAllocator has a maximum object size.
/// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxPolygonVertices = uint8{16}; // 8

/// Maximum number of vertices for any shape type.
constexpr auto MaxShapeVertices = std::max(uint8{2} /* edge vertices */, MaxPolygonVertices);

/// Length used as a collision and constraint tolerance.
/// Usually chosen to be numerically significant, but visually insignificant.
/// Lower or raise to decrease or increase respectively the minimum of space
/// between bodies at rest.
/// @note Smaller values relative to sizes of bodies increases the time it takes for bodies to come to rest.
constexpr auto LinearSlop = float_t{1} / float_t{10000}; // aka 0.0001, originally 0.005

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto AngularSlop = Pi * float_t{2} / float_t{180};

/// Maximum time of impact iterations.
constexpr auto MaxTOIIterations = BOX2D_MAGIC(uint8{20});

/// Maximum time of impact root iterator count.
constexpr auto MaxTOIRootIterCount = BOX2D_MAGIC(uint8{50});

/// Max number of distance iterations.
constexpr auto MaxDistanceIterations = BOX2D_MAGIC(uint8{20});
	
// Dynamics

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
constexpr auto VelocityThreshold = float_t{8} / float_t{10}; // float_t{1};

/// Maximum linear velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto MaxTranslation = float_t{4}; // originally 2

/// Maximum angular velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto MaxRotation = Pi / float_t{2};

/// Maximum number of bodies in a world (65534 based off uint16 and eliminating one value for invalid).
constexpr auto MaxBodies = uint16{std::numeric_limits<uint16>::max() - uint16{1}};

/// Body count type.
using body_count_t = std::remove_const<decltype(MaxBodies)>::type;

/// Contact count type.
using contact_count_t = std::conditional<sizeof(body_count_t) < sizeof(uint32), uint32, uint64>::type;
	
/// Maximum number of contacts in a world (2147319811).
/// @detail Uses the formula for the maximum number of edges in an undirectional graph of MaxBodies nodes. 
/// This occurs when every possible body is connected to every other body.
constexpr auto MaxContacts = contact_count_t{MaxBodies} * contact_count_t{MaxBodies - 1} / contact_count_t{2};

/// Joint count type.
using joint_count_t = std::conditional<sizeof(body_count_t) < sizeof(uint32), uint32, uint64>::type;

/// Maximum number of joints in a world (65534 based off uint16 and eliminating one value for invalid).
constexpr auto MaxJoints = uint16{std::numeric_limits<uint16>::max() - uint16{1}};

// Sleep

/// A body cannot sleep if its linear velocity is above this tolerance.
constexpr auto LinearSleepTolerance = float_t{1} / float_t{100}; // aka 0.01

/// A body cannot sleep if its angular velocity is above this tolerance.
constexpr auto AngularSleepTolerance = Pi * float_t{2} / float_t{180};

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

/// Logging function.
void log(const char* string, ...);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct Version
{
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
};

constexpr auto BuiltVersion = Version{BOX2D_MAJOR_VERSION, BOX2D_MINOR_VERSION, BOX2D_REVISION};
}

#endif
