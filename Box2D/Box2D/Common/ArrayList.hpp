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

#ifndef ArrayList_hpp
#define ArrayList_hpp

#include <type_traits>

namespace box2d
{
	template <typename TYPE, std::size_t MAXSIZE>
	class ArrayList
	{
	public:
		using size_type = typename std::remove_const<decltype(MAXSIZE)>::type;
		using data_type = TYPE;
		using pointer = data_type*;
		using const_pointer = const data_type*;
		
		bool add(data_type value)
		{
			if (m_size < MAXSIZE)
			{
				m_elements[m_size] = value;
				++m_size;
				return true;
			}
			return false;
		}
		
		data_type& operator[](size_type index)
		{
			assert(index < MAXSIZE);
			return m_elements[index];
		}
		
		data_type operator[](size_type index) const
		{
			assert(index < MAXSIZE);
			return m_elements[index];
		}
		
		size_type size() const noexcept { return m_size; }
		
		size_type max_size() const noexcept { return MAXSIZE; }
		
		pointer begin() noexcept { return &m_elements[0]; }
		pointer end() noexcept { return &m_elements[0] + m_size; }
		
		const_pointer begin() const noexcept { return &m_elements[0]; }
		const_pointer end() const noexcept { return &m_elements[0] + m_size; }
		
	private:
		data_type m_elements[MAXSIZE];
		size_type m_size = size_type{0};
	};
	
} /* namespace box2d */

#endif /* ArrayList_hpp */
