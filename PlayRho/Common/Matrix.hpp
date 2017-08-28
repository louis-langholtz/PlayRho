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

#ifndef Matrix_hpp
#define Matrix_hpp

#include <PlayRho/Common/Vector.hpp>
#include <PlayRho/Common/Templates.hpp>
#include <PlayRho/Common/Numbers.hpp>
#include <PlayRho/Common/Units.hpp>

namespace playrho {
    
    template <std::size_t N, std::size_t M, typename T>
    using Matrix = Vector<N, Vector<M, T>>;
    
    template <typename T>
    using Matrix22 = Matrix<2, 2, T>;
    
    template <typename T>
    using Matrix33 = Matrix<3, 3, T>;
    
    using Mat22 = Matrix22<Real>;
    
    using Mass22 = Matrix22<Mass>;
    using InvMass22 = Matrix22<InvMass>;
    
    using Mat33 = Matrix33<Real>;
    
    template <>
    constexpr inline bool IsValid(const Mat22& value) noexcept
    {
        return IsValid(Get<0>(value)) && IsValid(Get<1>(value));
    }
    
    template <>
    constexpr inline Mat22 GetInvalid() noexcept
    {
        return Mat22{GetInvalid<Real2>(), GetInvalid<Real2>()};
    }

} // namespace playrho

#endif /* Matrix_hpp */
