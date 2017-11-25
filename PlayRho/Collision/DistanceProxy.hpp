/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_DISTANCEPROXY_HPP
#define PLAYRHO_COLLISION_DISTANCEPROXY_HPP

#include <PlayRho/Common/Math.hpp>
#include <vector>
#include <algorithm>

// Define IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS to implement the DistanceProxy class
// using buffers instead of pointers. Note that timing tests suggest implementing with
// buffers is significantly slower. Using buffers could make subclassing the Shape class
// easier though so a buffering code alternative is kept in the source code for now.
// #define IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS

namespace playrho
{
    class Shape;

    /// @brief Distance Proxy.
    ///
    /// @details A distance proxy aggragates a convex set of vertices and a vertexRadius
    ///   of those vertices. This can be visualized as a convex N-gon with rounded corners.
    ///   It's meant to represent any single portion of a shape identified by its child-index.
    ///   These are used by the GJK algorithm: "a method for determining the minimium distance
    ///   between two convex sets".
    ///
    /// @note This data structure is 24-bytes.
    ///
    /// @sa https://en.wikipedia.org/wiki/Gilbert%2DJohnson%2DKeerthi_distance_algorithm
    ///
    class DistanceProxy
    {
    public:
        /// @brief Size type.
        /// @details Must be big enough to hold max posible count of vertices.
        using size_type = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        /// @brief Invalid index.
        static constexpr size_type InvalidIndex = static_cast<size_type>(-1);
        
        DistanceProxy() = default;
        
        /// @brief Copy constructor.
        DistanceProxy(const DistanceProxy& copy) noexcept:
#ifndef IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS
            m_vertices{copy.m_vertices},
            m_normals{copy.m_normals},
#endif
            m_count{copy.m_count},
            m_vertexRadius{copy.m_vertexRadius}
        {
#ifdef IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS
            const auto count = copy.m_count;
            std::copy(copy.m_vertices, copy.m_vertices + count, m_vertices);
            std::copy(copy.m_normals, copy.m_normals + count, m_normals);
#else
            // Intentionall empty.
#endif
       }

        /// Initializing constructor.
        ///
        /// @details Constructs a distance proxy for n-point shape (like a polygon).
        ///
        /// @param vertexRadius Radius of the given vertices.
        /// @param count Count of elements of the vertices and normals arrays.
        /// @param vertices Collection of vertices of the shape (relative to the shape's origin).
        /// @param normals Collection of normals of the shape.
        ///
        /// @note The vertices collection must have more than zero elements and no more than
        ///    <code>MaxShapeVertices</code> elements.
        /// @warning Behavior is undefined if the vertices collection has less than one element or
        ///   more than <code>MaxShapeVertices</code> elements.
        ///
        DistanceProxy(const NonNegative<Length> vertexRadius, const size_type count,
                                const Length2* vertices, const UnitVec2* normals) noexcept:
#ifndef IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS
            m_vertices{vertices},
            m_normals{normals},
#endif
            m_count{count},
            m_vertexRadius{vertexRadius}
        {
            assert(vertexRadius >= 0_m);
            assert(count >= 0);
            assert(count < 1 || vertices);
            assert(count < 2 || normals);
#ifdef IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS
            if (vertices)
            {
                std::copy(vertices, vertices + count, m_vertices);
            }
            if (normals)
            {
                std::copy(normals, normals + count, m_normals);
            }
#endif
        }
        
        /// Gets the vertexRadius of the vertices of the associated shape.
        /// @return Non-negative distance.
        auto GetVertexRadius() const noexcept { return m_vertexRadius; }
        
        /// Gets the vertex count.
        /// @details This is the count of valid vertex elements that this object provides.
        /// @return Value between 0 and <code>MaxShapeVertices</code>.
        /// @note This only returns 0 if this proxy was default constructed.
        auto GetVertexCount() const noexcept { return m_count; }
        
        /// Gets a vertex by index.
        ///
        /// @param index Index value less than the count of vertices represented by this proxy.
        ///
        /// @warning Behavior is undefined if the index given is not less than the count of vertices
        ///   represented by this proxy.
        /// @warning Behavior is undefined if InvalidIndex is given as the index value.
        ///
        /// @return Vertex linear position (relative to the shape's origin) at the given index.
        ///
        /// @sa Distance.
        ///
        auto GetVertex(size_type index) const noexcept
        {
            assert(index != InvalidIndex);
            assert(index < m_count);
            return *(m_vertices + index);
        }
        
        /// @brief Gets the normal for the given index.
        auto GetNormal(size_type index) const noexcept
        {
            assert(index != InvalidIndex);
            assert(index < m_count);
            return *(m_normals + index);
        }

    private:
#ifdef IMPLEMENT_DISTANCEPROXY_WITH_BUFFERS
        Length2 m_vertices[MaxShapeVertices]; ///< Vertices.
        UnitVec2 m_normals[MaxShapeVertices]; ///< Normals.
#else
        const Length2* m_vertices = nullptr; ///< Vertices.
        const UnitVec2* m_normals = nullptr; ///< Normals.
#endif
        size_type m_count = 0; ///< Count of valid elements of m_vertices.
        NonNegative<Length> m_vertexRadius = 0_m; ///< Radius of the vertices of the associated shape.
    };
    
    /// @brief Determines with the two given distance proxies are equal.
    /// @relatedalso DistanceProxy
    bool operator== (const DistanceProxy& lhs, const DistanceProxy& rhs) noexcept;
    
    /// @brief Determines with the two given distance proxies are not equal.
    /// @relatedalso DistanceProxy
    inline bool operator!= (const DistanceProxy& lhs, const DistanceProxy& rhs) noexcept
    {
        return !(lhs == rhs);
    }
    
    /// @brief Gets the supporting vertex index in the given direction for the given distance proxy.
    /// @details This finds the vertex that's most significantly in the direction of the given
    ///   vector and returns its index.
    /// @note 0 is returned for a given zero length direction vector.
    /// @param proxy Distance proxy object to find index in if a valid index exists for it.
    /// @param d Direction vector to find index for.
    /// @return InvalidIndex if d is invalid or the count of vertices is zero, otherwise a
    ///   value from 0 to one less than count.
    /// @sa GetVertexCount().
    /// @relatedalso DistanceProxy
    DistanceProxy::size_type GetSupportIndex(const DistanceProxy& proxy, Vec2 d) noexcept;

    /// @brief Finds the lowest right most vertex in the given collection.
    std::size_t FindLowestRightMostVertex(Span<const Length2> vertices);
    
    /// @brief Gets the convex hull for the given collection of vertices as a vector.
    std::vector<Length2> GetConvexHullAsVector(Span<const Length2> vertices);

    /// @brief Tests a point for containment in the given distance proxy.
    /// @param proxy Distance proxy to check if point is within.
    /// @param point Point in local coordinates.
    /// @return <code>true</code> if point is contained in the proxy, <code>false</code> otherwise.
    /// @relatedalso DistanceProxy
    /// @ingroup TestPointGroup
    bool TestPoint(const DistanceProxy& proxy, Length2 point) noexcept;
    
} // namespace playrho

#endif // PLAYRHO_COLLISION_DISTANCEPROXY_HPP
