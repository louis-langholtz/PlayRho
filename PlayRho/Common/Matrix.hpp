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

#ifndef PLAYRHO_COMMON_MATRIX_HPP
#define PLAYRHO_COMMON_MATRIX_HPP

#include <PlayRho/Common/Vector.hpp>
#include <PlayRho/Common/Vector2.hpp>
#include <PlayRho/Common/Templates.hpp>
#include <PlayRho/Common/Real.hpp>
#include <PlayRho/Common/Units.hpp>

namespace playrho {

/// @brief Generic M by N matrix.
/// @note M is the number of rows of the matrix.
/// @note N is the number of columns of the matrix.
/// @sa https://en.wikipedia.org/wiki/Matrix_(mathematics)
template <typename T, std::size_t M, std::size_t N>
using Matrix = Vector<Vector<T, N>, M>;

/// @brief 2 by 2 matrix.
template <typename T>
using Matrix22 = Matrix<T, 2, 2>;

/// @brief 3 by 3 matrix.
template <typename T>
using Matrix33 = Matrix<T, 3, 3>;

/// @brief 2 by 2 matrix of Real elements.
using Mat22 = Matrix22<Real>;

/// @brief 2 by 2 matrix of Mass elements.
using Mass22 = Matrix22<Mass>;

/// @brief 2 by 2 matrix of <code>InvMass</code> elements.
using InvMass22 = Matrix22<InvMass>;

/// @brief 3 by 3 matrix of Real elements.
using Mat33 = Matrix33<Real>;

/// @brief Determines if the given value is valid.
template <>
PLAYRHO_CONSTEXPR inline bool IsValid(const Mat22& value) noexcept
{
    return IsValid(Get<0>(value)) && IsValid(Get<1>(value));
}

/// @brief Gets an invalid value for a <code>Mat22</code>.
template <>
PLAYRHO_CONSTEXPR inline Mat22 GetInvalid() noexcept
{
    return Mat22{GetInvalid<Vec2>(), GetInvalid<Vec2>()};
}

/// @brief Multiplies an A-by-B matrix by a B-by-C matrix.
/// @note From Wikipedia:
///   > Multiplication of two matrices is defined if and only if the number of columns
///   > of the left matrix is the same as the number of rows of the right matrix.
/// @note Matrix multiplication is not commutative.
/// @note Algorithmically speaking, this implementation is called the "naive" algorithm.
///   For small matrices, like 3-by-3 or smaller matrices, its complexity shouldn't be an issue.
///   The matrix dimensions are compile time constants anyway which can help compilers
///   automatically identify loop unrolling and hardware level parallelism opportunities.
/// @param lhs Left-hand-side matrix.
/// @param rhs Right-hand-side matrix.
/// @return A-by-C matrix product of the left-hand-side matrix and the right-hand-side matrix.
/// @sa https://en.wikipedia.org/wiki/Matrix_multiplication
/// @sa https://en.wikipedia.org/wiki/Matrix_multiplication_algorithm
/// @sa https://en.wikipedia.org/wiki/Commutative_property
/// @relatedalso Vector
template <typename T1, typename T2, std::size_t A, std::size_t B, std::size_t C>
PLAYRHO_CONSTEXPR inline
auto operator* (const Matrix<T1, A, B>& lhs, const Matrix<T2, B, C>& rhs) noexcept
{
    using OT = decltype(T1{} * T2{});
    auto result = Matrix<OT, A, C>{};
    for (auto a = static_cast<std::size_t>(0); a < A; ++a)
    {
        for (auto c = static_cast<std::size_t>(0); c < C; ++c)
        {
            // So for 2x3 lhs matrix * 3*2 rhs matrix... result is 2x2 matrix:
            // result[0][0] = lhs[0][0] * rhs[0][0] + lhs[0][1] * rhs[1][0] + lhs[0][2] * rhs[2][0]
            // result[0][1] = lhs[0][0] * rhs[0][1] + lhs[0][1] * rhs[1][1] + lhs[0][2] * rhs[2][1]
            // result[1][0] = lhs[1][0] * rhs[0][0] + lhs[1][1] * rhs[1][0] + lhs[1][2] * rhs[2][0]
            // result[1][1] = lhs[1][0] * rhs[0][1] + lhs[1][1] * rhs[1][1] + lhs[1][2] * rhs[2][1]
            auto element = OT{};
            for (auto b = static_cast<std::size_t>(0); b < B; ++b)
            {
                element += lhs[a][b] * rhs[b][c];
            }
            result[a][c] = element;
        }
    }
    return result;
}

/// @brief Matrix addition operator for two same-type, same-sized matrices.
/// @sa https://en.wikipedia.org/wiki/Matrix_addition
template <typename T, std::size_t M, std::size_t N>
PLAYRHO_CONSTEXPR inline
auto operator+ (const Matrix<T, M, N>& lhs, const Matrix<T, M, N>& rhs) noexcept
{
    auto result = Matrix<T, M, N>{};
    for (auto m = static_cast<std::size_t>(0); m < M; ++m)
    {
        for (auto n = static_cast<std::size_t>(0); n < N; ++n)
        {
            result[m][n] = lhs[m][n] + rhs[m][n];
        }
    }
    return result;
}

/// @brief Matrix subtraction operator for two same-type, same-sized matrices.
/// @sa https://en.wikipedia.org/wiki/Matrix_addition
template <typename T, std::size_t M, std::size_t N>
PLAYRHO_CONSTEXPR inline
auto operator- (const Matrix<T, M, N>& lhs, const Matrix<T, M, N>& rhs) noexcept
{
    auto result = Matrix<T, M, N>{};
    for (auto m = static_cast<std::size_t>(0); m < M; ++m)
    {
        for (auto n = static_cast<std::size_t>(0); n < N; ++n)
        {
            result[m][n] = lhs[m][n] - rhs[m][n];
        }
    }
    return result;
}

} // namespace playrho

#endif // PLAYRHO_COMMON_MATRIX_HPP
