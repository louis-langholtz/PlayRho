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

/**
 * @file
 * @brief Definitions file for constant expressions of type Real.
 * @details This file defines physically dimensionless or unitless constant expression
 *   quantities of the Real type.
 * @sa https://en.wikipedia.org/wiki/Dimensionless_quantity
 */

#ifndef PLAYRHO_COMMON_REALCONSTANTS_HPP
#define PLAYRHO_COMMON_REALCONSTANTS_HPP

#include <PlayRho/Common/Real.hpp>

namespace playrho {

/// @brief Pi.
///
/// @details An "irrational number" that's defined as the ratio of a circle's
///   circumference to its diameter.
///
/// @note While the include file definition of M_PI may be a POSIX compliance requirement
///   and initially attractive to use, it's apparently not a C++ standards requirement
///   and casually including it pollutes the namespace of all code that uses this library.
///   Whatever the case, MSVS2017 doesn't make it part of the cmath include without enabling
///   _USE_MATH_DEFINES. So rather than add yet more C-preprocessor macros to all
///   sources that this library may be compiled with, it's simply hard-coded in here
///   instead using a C++ mechanism that also keeps it with the enclosing namespace.
/// @note Any narrowing is intentional.
///
/// @sa https://en.wikipedia.org/wiki/Pi
///
constexpr auto Pi = Real(3.14159265358979323846264338327950288);

/// @brief Square root of two.
///
/// @sa https://en.wikipedia.org/wiki/Square_root_of_2
///
constexpr auto SquareRootTwo =
    Real(1.414213562373095048801688724209698078569671875376948073176679737990732478462);

} // namespace playrho

#endif // PLAYRHO_COMMON_REALCONSTANTS_HPP
