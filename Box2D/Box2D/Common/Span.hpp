/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef Span_hpp
#define Span_hpp

#include <cstddef>
#include <type_traits>
#include <iterator>

namespace box2d
{
	template <typename T>
	class Span
	{
	public:
		using data_type = T;
		using pointer = data_type*;
		using const_pointer = const data_type *;
		using size_type = std::size_t;
		
		Span() = default;
		
		Span(const Span& copy) = default;
		
		constexpr Span(pointer array, size_type size) noexcept:
			m_array{array}, m_size{size}
		{
		}
		
		constexpr Span(pointer first, pointer last) noexcept:
			m_array{first}, m_size{static_cast<size_type>(std::distance(first, last))}
		{
			assert(first <= last);
		}
		
		template <std::size_t SIZE>
		constexpr Span(data_type (&array)[SIZE]) noexcept: m_array{&array[0]}, m_size{SIZE} {}
		
		template <typename U, typename = std::enable_if_t< !std::is_array<U>::value > >
		constexpr Span(U& value) noexcept: m_array{value.begin()}, m_size{value.size()} {}
		
		template <typename U, typename = std::enable_if_t< !std::is_array<U>::value > >
		constexpr Span(const U& value) noexcept: m_array{value.begin()}, m_size{value.size()} {}
		
		constexpr Span(std::initializer_list<T> list) noexcept: m_array{list.begin()}, m_size{list.size()} {}
				
		pointer begin() const noexcept { return m_array; }
		const_pointer cbegin() const noexcept { return m_array; }
		
		pointer end() const noexcept { return m_array + m_size; }
		const_pointer cend() const noexcept { return m_array + m_size; }
		
		data_type& operator[](size_type index) noexcept
		{
			assert(index < m_size);
			return m_array[index];
		}
		
		const data_type& operator[](size_type index) const noexcept
		{
			assert(index < m_size);
			return m_array[index];
		}
		
		size_type size() const noexcept { return m_size; }
		
	private:
		pointer m_array = nullptr;
		size_type m_size = 0;
	};
	
} // namespace box2d

#endif /* Span_hpp */
