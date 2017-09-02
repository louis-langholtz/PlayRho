/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

/**
* @file
* @brief Types and default settings file.
*/

#ifndef PLAYRHO_SETTINGS_H
#define PLAYRHO_SETTINGS_H

#include <cstddef>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <algorithm>

#include <PlayRho/Defines.hpp>
#include <PlayRho/Common/Templates.hpp>
#include <PlayRho/Common/RealNum.hpp>
#include <PlayRho/Common/Units.hpp>
#include <PlayRho/Common/Wider.hpp>

/**
* @namespace playrho
*
* Namespace for all PlayRho related names.
*/
namespace playrho
{

namespace details
{
template <typename T>
struct Defaults
{
    static constexpr auto GetLinearSlop() noexcept
    {
        // Return the value used by Box2D 2.3.2 b2_linearSlop define....
        return Length{ Real(0.005f) * Meter };
    }

    static constexpr auto GetMaxVertexRadius() noexcept
    {
        // DefaultLinearSlop * Real{2 * 1024 * 1024};
        // linearSlop * 2550000
        return Length{ Real(255) * Meter };
    }
};

template <unsigned int FRACTION_BITS>
struct Defaults<Fixed<std::int32_t, FRACTION_BITS>>
{
    static constexpr auto GetLinearSlop() noexcept
    {
        // Needs to be big enough that the step tolerance doesn't go to zero.
        // ex: FRACTION_BITS==10, then divisor==256
        return Length{ Meter / Real{ (1 << (FRACTION_BITS - 2)) } };
    }

    static constexpr auto GetMaxVertexRadius() noexcept
    {
        // linearSlop * 2550000
        return Length{ Real(1 << (28 - FRACTION_BITS)) * Meter };
    }
};

}

/// @brief Maximum number of supportable edges in a simplex.
constexpr auto MaxSimplexEdges = std::uint8_t{ 3 };

/// @brief Max child count.
constexpr auto MaxChildCount = (std::numeric_limits<std::uint32_t>::max)() >> 6;

/// @brief Child counter type.
/// @details Relating to "children" of shape where each child is a convex shape possibly
///   comprising a concave shape.
/// @note This type must always be able to contain the <code>MaxChildCount</code> value.
using ChildCounter = std::remove_const<decltype(MaxChildCount)>::type;

/// Time step iterations type.
/// @details A type for counting iterations per time-step.
using ts_iters_t = std::uint8_t;

constexpr auto MaxFloat = (std::numeric_limits<Real>::max)(); // FLT_MAX

                                                            // Collision

                                                            /// Maximum manifold points.
                                                            /// This is the maximum number of contact points between two convex shapes.
                                                            /// Do not change this value.
                                                            /// @note For memory efficiency, uses the smallest integral type that can hold the value. 
constexpr auto MaxManifoldPoints = std::uint8_t{ 2 };

/// Maximum number of vertices for any shape type.
/// @note For memory efficiency, uses the smallest integral type that can hold the value.
constexpr auto MaxShapeVertices = std::uint8_t{ 254 };

/// @brief Default linear slop.
/// @details Length used as a collision and constraint tolerance.
///   Usually chosen to be numerically significant, but visually insignificant.
///   Lower or raise to decrease or increase respectively the minimum of space
///   between bodies at rest.
/// @note Smaller values relative to sizes of bodies increases the time it takes
///   for bodies to come to rest.
constexpr auto DefaultLinearSlop = details::Defaults<Real>::GetLinearSlop();

constexpr auto DefaultMinVertexRadius = DefaultLinearSlop * Real{ 2 };
constexpr auto DefaultMaxVertexRadius = details::Defaults<Real>::GetMaxVertexRadius();

/// @brief Default AABB extension amount.
constexpr auto DefaultAabbExtension = DefaultLinearSlop * Real{ 20 };

/// @brief Default distance multiplier.
constexpr auto DefaultDistanceMultiplier = Real{ 2 };

/// @brief Default angular slop.
/// @details
/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr auto DefaultAngularSlop = (Pi * Real{ 2 } *Radian) / Real{ 180 };

/// @brief Default maximum linear correction.
/// @details The maximum linear position correction used when solving constraints.
///   This helps to prevent overshoot.
/// @note This value should be greater than the linear slop value.
constexpr auto DefaultMaxLinearCorrection = Real{ 0.2f } *Meter;

/// @brief Default maximum angular correction.
/// @note This value should be greater than the angular slop value.
constexpr auto DefaultMaxAngularCorrection = Real(8.0f / 180.0f) * Pi * Radian;

constexpr auto DefaultMaxTranslation = Length{ Real(2.0f) * Meter };

/// @brief Default maximum rotation per world step.
/// @warning This value should be less than Pi * Radian.
/// @note This limit is meant to prevent numerical problems. Adjusting this value isn't advised.
/// @sa StepConf::maxRotation.
constexpr auto DefaultMaxRotation = Angle{ Pi * Radian / Real(2) };

/// @brief Default maximum time of impact iterations.
constexpr auto DefaultMaxToiIters = std::uint8_t{ 20 };

/// Default maximum time of impact root iterator count.
constexpr auto DefaultMaxToiRootIters = std::uint8_t{ 30 };

/// Default max number of distance iterations.
constexpr auto DefaultMaxDistanceIters = std::uint8_t{ 20 };

/// Default maximum number of sub steps.
/// @details
/// This is the default maximum number of sub-steps per contact in continuous physics simulation.
/// In other words, this is the default maximum number of times in a world step that a contact will
/// have continuous collision resolution done for it.
/// @note Used in the TOI phase of step processing.
constexpr auto DefaultMaxSubSteps = std::uint8_t{ 8 };

// Dynamics

/// @brief Default velocity threshold.
constexpr auto DefaultVelocityThreshold = Real(1) * MeterPerSecond;

constexpr auto DefaultRegMinMomentum = Momentum{ (Real(0) / Real(100)) * NewtonSecond };
constexpr auto DefaultToiMinMomentum = Momentum{ (Real(0) / Real(100)) * NewtonSecond };

/// @brief Maximum number of bodies in a world.
/// @note This is 65534 based off std::uint16_t and eliminating one value for invalid.
constexpr auto MaxBodies = static_cast<std::uint16_t>((std::numeric_limits<std::uint16_t>::max)() -
    std::uint16_t{ 1 });

/// @brief Body count type.
/// @note This type must always be able to contain the <code>MaxBodies</code> value.
using BodyCounter = std::remove_const<decltype(MaxBodies)>::type;

/// @brief Contact count type.
/// @note This type must be able to contain the squared value of <code>BodyCounter</code>.
using ContactCounter = Wider<BodyCounter>::type;

/// @brief Invalid contact index.
constexpr auto InvalidContactIndex = static_cast<ContactCounter>(-1);

/// @brief Maximum number of contacts in a world (2147319811).
/// @details Uses the formula for the maximum number of edges in an unidirectional graph of MaxBodies nodes. 
/// This occurs when every possible body is connected to every other body.
constexpr auto MaxContacts = ContactCounter{ MaxBodies } *ContactCounter{ MaxBodies - 1 } / ContactCounter{ 2 };

/// @brief Maximum number of joints in a world.
/// @note This is 65534 based off std::uint16_t and eliminating one value for invalid.
constexpr auto MaxJoints = static_cast<std::uint16_t>((std::numeric_limits<std::uint16_t>::max)() -
    std::uint16_t{ 1 });

/// @brief Joint count type.
/// @note This type must be able to contain the <code>MaxJoints</code> value.
using JointCounter = std::remove_const<decltype(MaxJoints)>::type;

/// @brief Default step time.
constexpr auto DefaultStepTime = Time{ Second / Real{ 60 } };

/// @brief Default step frequency.
constexpr auto DefaultStepFrequency = Frequency{ Hertz * Real{ 60 } };

// Sleep

/// Default minimum still time to sleep.
/// @details The default minimum time bodies must be still for bodies to be put to sleep.
constexpr auto DefaultMinStillTimeToSleep = Time{ Second / Real{ 2 } }; // aka 0.5 secs

                                                                        /// Default linear sleep tolerance.
                                                                        /// @details A body cannot sleep if the magnitude of its linear velocity is above this amount.
constexpr auto DefaultLinearSleepTolerance = Real{ 0.01f } *MeterPerSecond; // aka 0.01

                                                                            /// Default angular sleep tolerance.
                                                                            /// @details A body cannot sleep if its angular velocity is above this amount.
constexpr auto DefaultAngularSleepTolerance = Real{ (Pi * 2) / 180 } *RadianPerSecond;

/// Default circles ratio.
/// @details Ratio used for switching between rounded-corner collisions and closest-face
///   biased normal collisions.
constexpr auto DefaultCirclesRatio = Real{ 10 };

// Mathematical constants
constexpr auto SquareRootTwo = Real(1.414213562373095048801688724209698078569671875376948073176679737990732478462);

}

#endif