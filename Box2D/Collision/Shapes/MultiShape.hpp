/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef ConcaveShape_hpp
#define ConcaveShape_hpp

#include <Box2D/Collision/Shapes/Shape.hpp>
#include <vector>

namespace box2d {
    
    class VertexSet;
    
    /// Concave shape.
    class MultiShape: public Shape
    {
    public:
        
        /// Vertex count type.
        ///
        /// @note This type must not support more than 255 vertices as that would conflict
        ///   with the <code>ContactFeature::index_t</code> type.
        ///
        using vertex_count_t = std::uint8_t;
        
        static constexpr auto InvalidVertex = static_cast<vertex_count_t>(-1);
        
        static constexpr Length GetDefaultVertexRadius() noexcept
        {
            return DefaultLinearSlop * RealNum{2};
        }
        
        struct Conf: public Shape::Conf
        {
            constexpr Conf(): Shape::Conf{Shape::Conf{}.UseVertexRadius(GetDefaultVertexRadius())}
            {
            }
        };
        
        static constexpr Conf GetDefaultConf() noexcept
        {
            return Conf{};
        }
        
        /// Default constructor.
        /// @details Constructs a polygon shape with a 0,0 centroid and vertex count of 0.
        /// @note Polygons with a vertex count less than 1 are "degenerate" and should be
        ///   treated as invalid.
        explicit MultiShape(const Conf& conf = GetDefaultConf()) noexcept:
            Shape{conf}
        {
            // Intentionally empty.
        }
        
        MultiShape(const MultiShape&) = default;
        
        /// Gets the number of child primitives.
        /// @return Positive non-zero count.
        child_count_t GetChildCount() const noexcept override;
        
        DistanceProxy GetChild(child_count_t index) const noexcept override;
        
        /// Computes the mass properties of this shape using its dimensions and density.
        /// The inertia tensor is computed about the local origin.
        /// @return Mass data for this shape.
        MassData GetMassData() const noexcept override;
                
        void Accept(Visitor& visitor) const override;
        
        /// Creates a convex hull from the given set of local points.
        /// The size of the set must be in the range [1, MaxShapeVertices].
        /// @warning the points may be re-ordered, even if they form a convex polygon
        /// @warning collinear points are handled but not removed. Collinear points
        ///   may lead to poor stacking behavior.
        void AddConvexHull(const VertexSet& points) noexcept;

    private:
        struct ConvexHull
        {
            /// Array of vertices.
            /// @details Consecutive vertices constitute "edges" of the polygon.
            std::vector<Length2D> vertices;
            
            /// Normals of edges.
            /// @details
            /// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
            /// consecutive pairs of elements of vertices.
            std::vector<UnitVec2> normals;
        };
        
        std::vector<ConvexHull> m_children;

        /// Centroid of this shape.
        Length2D m_centroid = Vec2_zero * Meter;
    };
    
    inline child_count_t MultiShape::GetChildCount() const noexcept
    {
        return static_cast<child_count_t>(m_children.size());
    }
    
    inline DistanceProxy MultiShape::GetChild(child_count_t index) const noexcept
    {
        const auto& child = m_children.at(index);
        return DistanceProxy{
            GetVertexRadius(), static_cast<DistanceProxy::size_type>(child.vertices.size()),
            &(child.vertices[0]), &(child.normals[0])
        };
    }
    
    inline void MultiShape::Accept(box2d::Shape::Visitor &visitor) const
    {
        visitor.Visit(*this);
    }

}

#endif /* ConcaveShape_hpp */
