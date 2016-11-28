/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef VertexSet_hpp
#define VertexSet_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	template <size_t MAXSIZE>
	class VertexSet
	{
	public:
		using const_pointer = const Vec2*;

		VertexSet(Vec2::data_type minlen2 = Sqrt(std::numeric_limits<Vec2::data_type>::min()) * 2):
			m_minlen2{std::abs(minlen2)}
		{
			assert(minlen2 >= 0);
		}

		Vec2::data_type get_minlen2() const noexcept { return m_minlen2; }

		bool add(Vec2 value)
		{
			if (m_size < MAXSIZE)
			{
				if (find(value) != end())
				{
					return false;
				}
				m_elements[m_size] = value;
				++m_size;
				return true;
			}
			return false;
		}

		size_t size() const noexcept { return m_size; }
		
		const_pointer begin() const { return m_elements; }
		
		const_pointer end() const { return m_elements + m_size; }

		/// Finds contained point whose delta with the given point has a squared length less
		/// than or equal to this set's minimum length squared value.
		const_pointer find(Vec2 value) const
		{
			// squaring anything smaller than the sqrt(std::numeric_limits<Vec2::data_type>::min())
			// won't be reversible.
			// i.e. won't obey the property that square(sqrt(a)) == a and sqrt(square(a)) == a.
			for (auto&& elem: *this)
			{
				// length squared must be large enough to have a reasonable enough unit vector.
				if (GetLengthSquared(value - elem) <= m_minlen2)
				{
					// found or delta poorly conditioned
					return &elem;
				}
			}
			return end();
		}

		Vec2 operator[](size_t index) const noexcept
		{
			assert(index < m_size);
			return m_elements[index];
		}

	private:
		size_t m_size = 0; ///< Size. 8-bytes.
		Vec2 m_elements[MAXSIZE]; ///< Elements. MAXSIZE * sizeof(Vec2).
		const Vec2::data_type m_minlen2; ///< Minimum length squared. sizeof(Vec2)/2 or 4-bytes.
	};
}

#endif /* VertexSet_hpp */
