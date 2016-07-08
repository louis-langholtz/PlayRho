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

#ifndef DistanceProxy_hpp
#define DistanceProxy_hpp

#include <Box2D/Common/Math.h>
#include <array>

namespace box2d
{
	class Shape;

	/// Distance Proxy.
	/// @detail
	/// A distance proxy is used by the GJK algorithm.
	/// It encapsulates any shape.
	/// @sa https://en.wikipedia.org/wiki/Gilbert%2DJohnson%2DKeerthi_distance_algorithm
	class DistanceProxy
	{
	public:
		/// Size type.
		/// @detail Must be big enough to hold max posible count of vertices.
		using size_type = std::remove_const<decltype(MaxShapeVertices)>::type;
		
		static constexpr size_type InvalidIndex = static_cast<size_type>(-1);
		
		DistanceProxy() = default;
		
		constexpr DistanceProxy(const DistanceProxy& copy) noexcept:
			m_buffer{copy.m_buffer},
			m_vertices{copy.m_vertices == &copy.m_buffer[0]? &m_buffer[0]: copy.m_vertices},
			m_count{copy.m_count},
			m_radius{copy.m_radius}
		{}
		
		/// Initializing constructor.
		/// @detail Constructs a distance proxy for a single point shape (like a circle).
		constexpr DistanceProxy(float_t radius, Vec2 v0) noexcept:
			m_radius{radius}, m_buffer{{v0}}, m_count{1}
		{
			assert(radius >= 0);
		}
		
		/// Initializing constructor.
		/// @detail Constructs a distance proxy for dual point shape (like an edge or a chain).
		constexpr DistanceProxy(float_t radius, Vec2 v0, Vec2 v1) noexcept:
			m_radius{radius}, m_buffer{{v0, v1}}, m_count{2}
		{
			assert(radius >= 0);
		}
		
		/// Initializing constructor.
		/// @detail Constructs a distance proxy for n-point shape (like a polygon).
		constexpr DistanceProxy(float_t radius, const Vec2* vertices, size_type count) noexcept:
			m_radius{radius}, m_buffer{}, m_vertices{vertices}, m_count{count}
		{
			assert(radius >= 0);
		}
		
		/// Gets the "radius" of the associated shape.
		/// @return Non-negative distance.
		auto GetRadius() const noexcept { return m_radius; }
		
		/// Gets the supporting vertex index in the given direction.
		/// @detail This finds the vertex that's most significantly in the direction of the given
		///   vector and returns its index.
		/// @note 0 is returned for a given zero length direction vector.
		/// @param d Direction vector to find index for.
		/// @return InvalidIndex if the count of vertices is zero or a value from 0 to one less than count.
		/// @sa GetVertexCount().
		size_type GetSupportIndex(const Vec2& d) const noexcept;
		
		/// Get the vertex count.
		/// @detail This is the count of valid vertex elements that this object provides.
		inline auto GetVertexCount() const noexcept { return m_count; }
		
		/// Get a vertex by index. Used by Distance.
		/// @param index A valid index value (must not be InvalidIndex).
		/// @note Behavior is undefined if InvalidIndex is given as the index value.
		/// @return 2D vector value at the given index.
		auto GetVertex(size_type index) const noexcept
		{
			assert(index != InvalidIndex);
			assert(index < m_count);
			return m_vertices[index];
		}
		
	private:
		std::array<Vec2,2> m_buffer;
		const Vec2* m_vertices = &m_buffer[0];
		size_type m_count = 0; ///< Count of valid elements of m_vertices.
		float_t m_radius = float_t{0}; ///< "Radius" of the associated shape (in meters).
	};
	
	/// Initialize the proxy using the given shape.
	/// @note The shape must remain in scope while the proxy is in use.
	DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index);

}; // namespace box2d

#endif /* DistanceProxy_hpp */
