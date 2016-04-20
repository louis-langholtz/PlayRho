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
using child_count_t = unsigned; // relating to "children" of b2Shape
using b2_size_t = std::size_t;
using island_count_t = b2_size_t; // relating to items in a b2Island

constexpr auto b2_maxFloat = std::numeric_limits<float32>::max(); // FLT_MAX;
constexpr auto b2_epsilon = std::numeric_limits<float32>::epsilon(); // FLT_EPSILON;
constexpr auto b2_pi = 3.14159265359f;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
constexpr auto b2_maxManifoldPoints = unsigned{2};

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
constexpr auto b2_maxPolygonVertices = unsigned{16}; // 8

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
constexpr auto b2_aabbMultiplier = float32{2.0f};

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto b2_linearSlop = float32{0.00005f}; // float32{0.005f};

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
constexpr auto b2_aabbExtension = b2_linearSlop * 20.0f; // float32{0.001f}; // float32{0.1f};

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto b2_angularSlop = float32{b2_pi * 2.0f / 180.0f};

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
constexpr auto b2_polygonRadius = float32{2.0f * b2_linearSlop};

/// Maximum number of sub-steps per contact in continuous physics simulation.
constexpr auto b2_maxSubSteps = unsigned{8}; // often hit but no apparent help against tunneling

constexpr auto b2_maxTOIIterations = unsigned{20};
constexpr auto b2_maxTOIRootIterCount = unsigned{50};

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
constexpr auto b2_maxTOIContacts = unsigned{32};

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
constexpr auto b2_velocityThreshold = float32{0.8f}; // float32{1.0f};

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
constexpr auto b2_maxLinearCorrection = b2_linearSlop * 40.0f; // float32{0.2f};

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
constexpr auto b2_maxAngularCorrection = float32{8.0f / 180.0f * b2_pi};

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
constexpr auto b2_maxTranslation = float32{2.0f};
constexpr auto b2_maxTranslationSquared = float32{b2_maxTranslation * b2_maxTranslation};

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
constexpr auto b2_maxRotation = float32{0.5f * b2_pi};
constexpr auto b2_maxRotationSquared = float32{b2_maxRotation * b2_maxRotation};

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
constexpr auto b2_baumgarte = float32{0.2f}; // float32{0.6f};
constexpr auto b2_toiBaugarte = float32{0.75f};


// Sleep

/// The time that a body must be still before it will go to sleep.
constexpr auto b2_timeToSleep = float32{0.5f};

/// A body cannot sleep if its linear velocity is above this tolerance.
constexpr auto b2_linearSleepTolerance = float32{0.01f};

/// A body cannot sleep if its angular velocity is above this tolerance.
constexpr auto b2_angularSleepTolerance = float32{2.0f / 180.0f * b2_pi};

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
