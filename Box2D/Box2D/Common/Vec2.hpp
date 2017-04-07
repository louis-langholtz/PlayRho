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

#ifndef Vec2_hpp
#define Vec2_hpp

#include <Box2D/Common/Settings.hpp>

namespace box2d
{
	/// Vector 2D.
	template <typename TYPE>
	struct Vector2D
	{
		using size_type = size_t;
		using data_type = TYPE;
		
		/// Default constructor does nothing (for performance).
		Vector2D() = default;
		
		Vector2D(const Vector2D& copy) = default;
		
		/// Construct using coordinates.
		constexpr Vector2D(data_type x_, data_type y_) noexcept : x{x_}, y{y_} {}
		
		/// Negate this vector.
		constexpr auto operator- () const noexcept { return Vector2D{-x, -y}; }
		
		/// Maximum size.
		/// @detail This is this vector type's dimensionality.
		constexpr size_type max_size() const noexcept { return 2; }
		
		/// Accesses element by index.
		/// @param i Index (0 for x, 1 for y).
		auto operator[] (size_type i) const
		{
			assert(i < max_size());
			switch (i)
			{
				case 0: return x;
				case 1: return y;
				default: break;
			}
			return x;
		}
		
		/// Accesses element by index.
		/// @param i Index (0 for x, 1 for y).
		auto& operator[] (size_type i)
		{
			assert(i < max_size());
			switch (i)
			{
				case 0: return x;
				case 1: return y;
				default: break;
			}
			return x;
		}
		
		data_type x, y;
	};
	
	template <typename TYPE>
	constexpr inline typename Vector2D<TYPE>::data_type GetX(const Vector2D<TYPE> value)
	{
		return value.x;
	}
	
	template <typename TYPE>
	constexpr inline typename Vector2D<TYPE>::data_type GetY(const Vector2D<TYPE> value)
	{
		return value.y;
	}

	template <typename TYPE>
	constexpr inline bool operator == (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
	{
		return (a.x == b.x) && (a.y == b.y);
	}
	
	template <typename TYPE>
	constexpr inline bool operator != (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
	{
		return (a.x != b.x) || (a.y != b.y);
	}

	/// Add two vectors component-wise.
	template <typename TYPE>
	constexpr inline Vector2D<TYPE> operator + (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
	{
		return Vector2D<TYPE>{a.x + b.x, a.y + b.y};
	}
	
	/// Subtract two vectors component-wise.
	template <typename TYPE>
	constexpr inline Vector2D<TYPE> operator - (const Vector2D<TYPE> a, const Vector2D<TYPE> b) noexcept
	{
		return Vector2D<TYPE>{a.x - b.x, a.y - b.y};
	}
	
	/// Increment the left hand side value by the right hand side value.
	template <typename TYPE>
	constexpr Vector2D<TYPE>& operator += (Vector2D<TYPE>& lhs, const Vector2D<TYPE> rhs) noexcept
	{
		lhs.x += rhs.x;
		lhs.y += rhs.y;
		return lhs;
	}
	
	/// Decrement the left hand side value by the right hand side value.
	template <typename TYPE>
	constexpr Vector2D<TYPE>& operator -= (Vector2D<TYPE>& lhs, const Vector2D<TYPE> rhs) noexcept
	{
		lhs.x -= rhs.x;
		lhs.y -= rhs.y;
		return lhs;
	}
	
	template <typename TYPE>
	constexpr Vector2D<TYPE>& operator *= (Vector2D<TYPE>& lhs, const RealNum rhs) noexcept
	{
		lhs.x *= rhs;
		lhs.y *= rhs;
		return lhs;
	}
	
	template <typename TYPE>
	constexpr Vector2D<TYPE>& operator /= (Vector2D<TYPE>& lhs, const RealNum rhs) noexcept
	{
		lhs.x /= rhs;
		lhs.y /= rhs;
		return lhs;
	}
	
	template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
	constexpr inline Vector2D<OUT_TYPE> operator * (const TYPE1 s, const Vector2D<TYPE2> a) noexcept
	{
		return Vector2D<OUT_TYPE>{s * a.x, s * a.y};
	}
	
	template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} * TYPE2{0})>
	constexpr inline Vector2D<OUT_TYPE> operator * (const Vector2D<TYPE1> a, const TYPE2 s) noexcept
	{
		return Vector2D<OUT_TYPE>{a.x * s, a.y * s};
	}
	
	template <typename TYPE1, typename TYPE2, typename OUT_TYPE = decltype(TYPE1{0} / TYPE2{0})>
	constexpr Vector2D<OUT_TYPE> operator/ (const Vector2D<TYPE1> a, const TYPE2 s) noexcept
	{
		return Vector2D<OUT_TYPE>{a.x / s, a.y / s};
	}
	
	/// Vector 2D of RealNum.
	/// @note This data structure is two-times the size of the <code>RealNum</code> type
	/// (or 8 using RealNum of float).
	using Vec2 = Vector2D<RealNum>;
	
} // namespace box2d

#endif /* Vec2_hpp */
