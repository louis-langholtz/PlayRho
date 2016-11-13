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

/// Maximum number of supportable vertices in a simplex.
constexpr auto MaxSimplexVertices = uint8{3};

/// Maximum number of vertices on a convex polygon.
/// You cannot increase this too much because BlockAllocator has a maximum object size.
/// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxPolygonVertices = uint8{16}; // 8

/// Maximum number of vertices for any shape type.
constexpr auto MaxShapeVertices = std::max(uint8{2} /* edge vertices */, MaxPolygonVertices);

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
constexpr auto AabbMultiplier = float_t{2};

/// Length used as a collision and constraint tolerance.
/// Usually chosen to be numerically significant, but visually insignificant.
/// Lower or raise to decrease or increase respectively the minimum of space
/// between bodies at rest.
/// @note Smaller values increases the time it takes for bodies to come to rest.
constexpr auto LinearSlop = float_t{1} / float_t{10000}; // aka 0.0001, originally 0.005

/// Fattens AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
constexpr auto AabbExtension = LinearSlop * float_t{20}; // aka 0.002, originally 0.1

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto AngularSlop = Pi * float_t{2} / float_t{180};

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
constexpr auto PolygonRadius = LinearSlop * float_t{2};

/// Maximum sub steps.
/// @detail
/// This is the maximum number of sub-steps per contact in continuous physics simulation.
/// In other words, this is the maximum number of times in a world step that a contact will
/// have continuous collision resolution done for it.
constexpr auto MaxSubSteps = BOX2D_MAGIC(uint16{48}); // originally 8, often hit but no apparent help against tunneling

/// Maximum number of sub-step position iterations.
constexpr auto MaxSubStepPositionIterations = BOX2D_MAGIC(unsigned{20});

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

/// Maximum linear position correction used when solving constraints.
/// This helps to prevent overshoot.
constexpr auto MaxLinearCorrection = BOX2D_MAGIC(LinearSlop * 40); // aka 0.002, originally 0.2

/// Maximum angular position correction used when solving constraints.
/// This helps to prevent overshoot.
constexpr auto MaxAngularCorrection = Pi * float_t{8} / float_t{180};

/// Maximum linear velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto MaxTranslation = float_t{4}; // originally 2

/// Maximum angular velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto MaxRotation = Pi / float_t{2};

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
constexpr auto Baumgarte = float_t{2} / float_t{10}; // aka 0.2.

/// Time of impact Baumgarte factor.
/// @sa Baumgarte.
constexpr auto ToiBaumgarte = float_t{75} / float_t{100}; // aka .75

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

/// The time that a body must be still before it will go to sleep.
constexpr auto MinStillTimeToSleep = float_t{1} / float_t{2}; // aka 0.5

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
