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
#include <cstdint>
#include <algorithm>

#include <Box2D/Defines.hpp>
#include <Box2D/Common/Templates.hpp>
#include <Box2D/Common/RealNum.hpp>
#include <Box2D/Common/Wider.hpp>
#include <Box2D/Common/Units.hpp>

/**
 * @namespace box2d
 *
 * Namespace for all Box2D related names.
 */
namespace box2d
{   
class Body;
class Contact;
class Joint;

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
