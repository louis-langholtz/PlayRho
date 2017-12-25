/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DEFINES_HPP
#define PLAYRHO_DEFINES_HPP

// Setup a define for unreachable code.
//
// Note that if someday the C++ standard picks up the unreachable attribute
// (see http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0627r0.pdf ),
// then this macro can be replaced with [[unreachable]].
//
#if defined(__clang__)
#define PLAYRHO_UNREACHABLE __builtin_unreachable()
#elif defined(__GNUC__)
#define PLAYRHO_UNREACHABLE __builtin_unreachable()
#elif defined(__ICC) || defined(__INTEL_COMPILER)
#define PLAYRHO_UNREACHABLE __builtin_unreachable()
#elif defined(__MSC_VER)
// For MS documentation on this use of its __assume, see:
//   https://msdn.microsoft.com/en-us/library/1b3fsfxw.aspx#Anchor_3
#define PLAYRHO_UNREACHABLE __assume(0)
#else // for any other compiler
#include <cstdlib>
#define PLAYRHO_UNREACHABLE std::abort()
#endif

// Macro for constant expressions.
// Value of this macro should either be 'constexpr' or empty.
#define PLAYRHO_CONSTEXPR constexpr

// Checks if platform supports 128-bit integer types and defines macros for them if so.
// Note that these could use any <code>LiteralType</code> type that has full operator and
// common mathemtical function support.
#ifdef __SIZEOF_INT128__
#define PLAYRHO_INT128 __int128_t
#define PLAYRHO_UINT128 __uint128_t
#endif

#endif /* PLAYRHO_DEFINES_HPP */
