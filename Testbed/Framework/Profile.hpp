/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_PROFILE_HPP
#define PLAYRHO_PROFILE_HPP

#include <PlayRho/Common/Math.hpp>

namespace testbed {

/// Profiling data. Times are in milliseconds.
struct Profile
{
    Real step = Real(0);
    Real collide = Real(0);
    Real solve = Real(0);
    Real solveInit = Real(0);
    Real solveVelocity = Real(0);
    Real solvePosition = Real(0);
    Real broadphase = Real(0);
    Real solveTOI = Real(0);
};
}

#endif /* PLAYRHO_PROFILE_HPP */
