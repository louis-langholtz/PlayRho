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
 * @brief Real number definition file.
 * @details This file may have been autogenerated from the RealNum.hpp.in file.
 */

#ifndef RealNum_hpp
#define RealNum_hpp

#include <PlayRho/Common/Fixed.hpp>

namespace playrho
{

    /// @brief Real-number type.
    ///
    /// @details This is the number type underlying numerical calculations conceptually involving
    /// real-numbers. Ideally the implementation of this type doesn't suffer from things like:
    /// catastrophic cancellation, catastrophic division, overflows, nor underflows.
    ///
    /// @note This can be implemented using float, double, long double, Fixed64, or Fixed32
    ///   (though the use of Fixed32 is discouraged).
    ///
    /// @note Regarding division:
    ///
    ///   While dividing 1 by a Real, caching the result, and then doing multiplications with the
    ///   result may well be faster (than repeatedly dividing), dividing 1 by Real can also result
    ///   in an underflow situation that's then compounded every time it's multiplied with other
    ///   values.
    ///
    ///   Meanwhile, dividing every value by Real isolates any underflows to the particular
    ///   division where underflow occurs.
    ///
    /// @warning Using Fixed32 is not advised as it's numerical limitations are more likely to
    ///   result in problems like overflows or underflows.
    /// @warning The note regarding division applies even more so when using a fixed-point type
    ///   (for Real).
    ///
    using Real = float;

    /// @brief Pi.
    ///
    /// @details While the include file definition of M_PI may be a POSIX compliance requirement
    ///   and initially attractive to use, it's apparently not a C++ standards requirement
    ///   and casually including it pollutes the namespace of all code that uses this library.
    ///   Whatever the case, MSVS2017 doesn't make it part of the cmath include without enabling
    ///   _USE_MATH_DEFINES. So rather than add yet more C-preprocessor macros to all
    ///   sources that this library may be compiled with, it's simply hard-coded in here
    ///   instead using a C++ mechanism that also keeps it with the enclosing namespace.
    ///
    /// @note Any narrowing is intentional.
    ///
    constexpr auto Pi = Real(3.14159265358979323846264338327950288);

    /// @brief SquareRootTwo.
    ///
    constexpr auto SquareRootTwo = Real(1.414213562373095048801688724209698078569671875376948073176679737990732478462);

}

#endif /* RealNum_hpp */
