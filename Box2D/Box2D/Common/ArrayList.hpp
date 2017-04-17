/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef ArrayList_hpp
#define ArrayList_hpp

#include <Box2D/Defines.hpp>

#include <type_traits>
#include <initializer_list>
#include <cassert>
#include <array>

namespace box2d
{
	/// Array list.
	template <typename VALUE_TYPE, std::size_t MAXSIZE, typename SIZE_TYPE = std::size_t>
	class ArrayList
	{
	public:
		using size_type = SIZE_TYPE;
		using value_type = VALUE_TYPE;
		using pointer = value_type*;
		using const_pointer = const value_type*;
		
		ArrayList() = default;

		template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE, typename = std::enable_if_t< COPY_MAXSIZE <= MAXSIZE >>
		BOX2D_CONSTEXPR ArrayList(const ArrayList<VALUE_TYPE, COPY_MAXSIZE, SIZE_TYPE>& copy):
			m_size{copy.size()},
			m_elements{copy.data()}
		{
			// Intentionally empty
		}

		template <std::size_t COPY_MAXSIZE, typename COPY_SIZE_TYPE, typename = std::enable_if_t< COPY_MAXSIZE <= MAXSIZE >>
		ArrayList& operator= (const ArrayList<VALUE_TYPE, COPY_MAXSIZE, COPY_SIZE_TYPE>& copy)
		{
			m_size = static_cast<SIZE_TYPE>(copy.size());
			m_elements = copy.data();
			return *this;
		}

		template <std::size_t SIZE, typename = std::enable_if_t< SIZE <= MAXSIZE >>
		ArrayList(value_type (&array)[SIZE]) noexcept
		{
			for (auto&& elem: array)
			{
				push_back(elem);
			}
		}

		ArrayList(std::initializer_list<value_type> list)
		{
			for (auto&& elem: list)
			{
				push_back(elem);
			}
		}
		
		void push_back(const value_type& value)
		{
			assert(m_size < MAXSIZE);
			m_elements[m_size] = value;
			++m_size;
		}
		
		void clear() noexcept
		{
			m_size = 0;
		}
		
		bool empty() const noexcept { return m_size == 0; }

		bool add(value_type value) noexcept
		{
			if (m_size < MAXSIZE)
			{
				m_elements[m_size] = value;
				++m_size;
				return true;
			}
			return false;
		}
		
		value_type& operator[](size_type index) noexcept
		{
			assert(index < MAXSIZE);
			return m_elements[index];
		}
		
		const value_type& operator[](size_type index) const noexcept
		{
			assert(index < MAXSIZE);
			return m_elements[index];
		}

		/// Gets the size of this collection.
		/// @detail This is the number of elements that have been added to this collection.
		/// @return Value between 0 and the maximum size for this collection.
		/// @sa max_size().
		BOX2D_CONSTEXPR size_type size() const noexcept { return m_size; }
		
		/// Gets the maximum size that this collection can be.
		/// @detail This is the maximum number of elements that can be contained in this collection.
		BOX2D_CONSTEXPR size_type max_size() const noexcept { return MAXSIZE; }
		
		pointer begin() noexcept { return &m_elements[0]; }
		pointer end() noexcept { return &m_elements[0] + m_size; }
		
		const_pointer begin() const noexcept { return &m_elements[0]; }
		const_pointer end() const noexcept { return &m_elements[0] + m_size; }
		
		auto data() const noexcept { return m_elements; }

	private:
		size_type m_size = size_type{0};
		std::array<value_type,MAXSIZE> m_elements;
	};
	
	template <typename T, std::size_t S>
	ArrayList<T, S>& operator+= (ArrayList<T, S>& lhs, const typename ArrayList<T, S>::data_type& rhs)
	{
		lhs.push_back(rhs);
		return lhs;
	}

	template <typename T, std::size_t S>
	ArrayList<T, S> operator+ (ArrayList<T, S> lhs, const typename ArrayList<T, S>::data_type& rhs)
	{
		lhs.push_back(rhs);
		return lhs;
	}

} /* namespace box2d */

namespace std
{
	/// Tuple size specialization for ArrayList classes.
	template< class T, size_t N, typename SIZE_TYPE >
	class tuple_size< box2d::ArrayList<T, N, SIZE_TYPE> >:	public integral_constant<size_t, N>
	{
		// Intentionally empty.
	};
}

#endif /* ArrayList_hpp */
