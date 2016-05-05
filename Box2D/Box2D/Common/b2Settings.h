/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_SETTINGS_H
#define B2_SETTINGS_H

#include <cstddef>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <limits>

#define B2_NOT_USED(x) ((void)(x))
#define b2Assert(A) assert(A)

using int8 = signed char;
using int16 = signed short;
using int32 = signed int;
using uint8 = unsigned char;
using uint16 = unsigned short;
using uint32 = unsigned int;
using float32 = float;
using float64 = double;

/// Box2D floating point type.
/// This should be float, double, or long double.
using b2Float = float;

using child_count_t = unsigned; // relating to "children" of b2Shape
using b2_size_t = std::size_t;
using island_count_t = b2_size_t; // relating to items in a b2Island

constexpr auto b2_maxFloat = std::numeric_limits<b2Float>::max(); // FLT_MAX
constexpr auto b2_epsilon = std::numeric_limits<b2Float>::epsilon(); // FLT_EPSILON;
constexpr auto b2_pi = b2Float(M_PI); // 3.14159265359

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// Maximum manifold points.
/// This is the number of contact points between two convex shapes.
/// Do not change this value.
constexpr auto b2_maxManifoldPoints = unsigned{2};

/// Maximum number of vertices on a convex polygon.
/// You cannot increase this too much because b2BlockAllocator has a maximum object size.
constexpr auto b2_maxPolygonVertices = unsigned{16}; // 8

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
constexpr auto b2_aabbMultiplier = b2Float(2);

/// Length used as a collision and constraint tolerance.
/// Usually chosen to be numerically significant, but visually insignificant.
/// Lower or raise to decrease or increase respectively the minimum of space
/// between bodies at rest.
constexpr auto b2_linearSlop = b2Float(0.00005); // originally 0.005;

/// Fattens AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
constexpr auto b2_aabbExtension = b2_linearSlop * b2Float(20); // aka 0.001, originally 0.1

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto b2_angularSlop = b2_pi * b2Float(2) / b2Float(180);

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
constexpr auto b2_polygonRadius = b2_linearSlop * b2Float(2);

/// Maximum number of sub-steps per contact in continuous physics simulation.
constexpr auto b2_maxSubSteps = unsigned{8}; // often hit but no apparent help against tunneling

constexpr auto b2_maxTOIIterations = unsigned{20};
constexpr auto b2_maxTOIRootIterCount = unsigned{50};

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
constexpr auto b2_maxTOIContacts = unsigned{32};

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
constexpr auto b2_velocityThreshold = b2Float(0.8); // b2Float(1);

/// Maximum linear position correction used when solving constraints.
/// This helps to prevent overshoot.
constexpr auto b2_maxLinearCorrection = b2_linearSlop * b2Float(40); // aka 0.002, originally 0.2

/// Maximum angular position correction used when solving constraints.
/// This helps to prevent overshoot.
constexpr auto b2_maxAngularCorrection = b2_pi * b2Float(8) / b2Float(180);

/// Maximum linear velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto b2_maxTranslation = b2Float(2);

/// Maximum angular velocity of a body.
/// This limit is very large and is used to prevent numerical problems.
/// You shouldn't need to adjust this.
constexpr auto b2_maxRotation = b2_pi / b2Float(2);

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
constexpr auto b2_baumgarte = b2Float(0.2); // b2Float(0.6);
constexpr auto b2_toiBaugarte = b2Float(0.75);


// Sleep

/// The time that a body must be still before it will go to sleep.
constexpr auto b2_timeToSleep = b2Float(0.5);

/// A body cannot sleep if its linear velocity is above this tolerance.
constexpr auto b2_linearSleepTolerance = b2Float(0.01);

/// A body cannot sleep if its angular velocity is above this tolerance.
constexpr auto b2_angularSleepTolerance = b2_pi * b2Float(2) / b2Float(180);

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* b2Alloc(b2_size_t size);

/// Implement this function to use your own memory allocator.
void* b2Realloc(void* ptr, b2_size_t new_size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Logging function.
void b2Log(const char* string, ...);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
	int32 major;		///< significant changes
	int32 minor;		///< incremental changes
	int32 revision;		///< bug fixes
};

/// Current version.
#define b2_majorVersion 3
#define b2_minorVersion 0
#define b2_revision 0
constexpr auto b2_version = b2Version{b2_majorVersion, b2_minorVersion, b2_revision};

#endif
