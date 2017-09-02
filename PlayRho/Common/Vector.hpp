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

#ifndef Vector_hpp
#define Vector_hpp

#include <cstddef>
#include <type_traits>
#include <PlayRho/Common/InvalidArgument.hpp>

namespace playrho
{

    /// @brief Vector.
    /// @details Basically a constexpr and constructor enhanced std::array for C++14.
    /// @note This type is trivially default constructible - i.e. default construction
    ///   performs no actions (no initialization).
    /// @note This type should be drop-in replacable with std::array in C++17.
    template <std::size_t N, typename T>
    struct Vector
    {
        static_assert(N > 0, "Dimension must be greater than 0");

        using value_type = T;
        using size_type = std::size_t;
        using difference_type = std::ptrdiff_t;
        using reference = value_type&;
        using const_reference = const value_type&;
        using pointer = value_type*;
        using const_pointer = const value_type*;
        using iterator = value_type*;
        using const_iterator = const value_type*;

        /// @brief Default constructor.
        /// @note Defaulted explicitly.
        /// @note This constructor performs no action.
        constexpr Vector() = default;

        template<typename... Tail>
        constexpr Vector(typename std::enable_if<sizeof...(Tail)+1 == N, T>::type
            head, Tail... tail) noexcept: elements{ head, T(tail)... }
        {
            //static_assert(sizeof...(args) == N, "Invalid number of arguments");
        }

        constexpr size_type max_size() const noexcept { return N; }
        constexpr size_type size() const noexcept { return N; }
        constexpr size_type empty() const noexcept { return N == 0; }

        constexpr iterator begin() noexcept { return iterator(elements); }
        constexpr iterator end() noexcept { return iterator(elements + N); }
        constexpr const_iterator begin() const noexcept { return const_iterator(elements); }
        constexpr const_iterator end() const noexcept { return const_iterator(elements + N); }
        constexpr const_iterator cbegin() const noexcept { return begin(); }
        constexpr const_iterator cend() const noexcept { return end(); }

        /// @brief Gets a reference to the requested element.
        /// @note No bounds checking is performed.
        /// @warning Behavior is undefined if given a position equal to or greater than size().
        constexpr reference operator[](size_type pos) noexcept
        {
            assert(pos < size());
            return elements[pos];
        }


        /// @brief Gets a constant reference to the requested element.
        /// @note No bounds checking is performed.
        /// @warning Behavior is undefined if given a position equal to or greater than size().
        constexpr const_reference operator[](size_type pos) const noexcept
        {
            assert(pos < size());
            return elements[pos];
        }

        constexpr const_reference x() const noexcept { return operator[](0) };
        constexpr const_reference y() const noexcept { return operator[](1) };
        constexpr const_reference z() const noexcept { return operator[](2) };
        constexpr const_reference w() const noexcept { return operator[](3) };
        constexpr void x(const_reference value) const noexcept { operator[](0) = value; };
        constexpr void y(const_reference value) const noexcept { operator[](1) = value; };
        constexpr void z(const_reference value) const noexcept { operator[](2) = value; };
        constexpr void w(const_reference value) const noexcept { operator[](3) = value; };

        /// @brief Gets a reference to the requested element.
        /// @throws InvalidArgument if given a position that's >= size().
        constexpr reference at(size_type pos)
        {
            if (pos >= size())
            {
                throw InvalidArgument("Vector::at: position >= size()");
            }
            return elements[pos];
        }

        /// @brief Gets a constant reference to the requested element.
        /// @throws InvalidArgument if given a position that's >= size().
        constexpr const_reference at(size_type pos) const
        {
            if (pos >= size())
            {
                throw InvalidArgument("Vector::at: position >= size()");
            }
            return elements[pos];
        }

        constexpr pointer data() noexcept
        {
            return elements;
        }

        constexpr const_pointer data() const noexcept
        {
            return elements;
        }

        /// @brief Elements.
        /// @warning Don't access this directly!
        /// @warning Data is not initialized on default construction. This is intentional
        ///   to avoid any performance overhead that default initialization might incur.
        value_type elements[N];
    };

    template <size_t I, size_t N, typename T>
    constexpr auto& Get(Vector<N, T>& v) noexcept
    {
        static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
        return v[I];
    }

    template <size_t I, size_t N, typename T>
    constexpr auto Get(const Vector<N, T>& v) noexcept
    {
        static_assert(I < N, "Index out of bounds in playrho::Get<> (playrho::Vector)");
        return v[I];
    }

} // namespace playrho

#endif /* Vector_hpp */
