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

#ifndef Manifold_hpp
#define Manifold_hpp

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/ContactFeature.hpp>

namespace playrho
{
    class DistanceProxy;
    struct Transformation;
    
    /// Manifold for two convex shapes.
    ///
    /// @details
    /// This describes zero, one, or two points of contact for which impulses should be applied to
    /// most naturally resolve those contacts. Ideally the manifold is calculated at the earliest
    /// point in time of contact occuring. The further past that time, the less natural contact
    /// resolution of solid bodies will be - eventually resulting in oddities like tunneling.
    ///
    /// Multiple types of contact are supported: clip point versus plane with radius, point versus
    /// point with radius (circles). Contacts are stored in this way so that position correction can
    /// account for movement, which is critical for continuous physics. All contact scenarios must
    /// be expressed in one of these types.
    ///
    /// Conceptually, a manifold represents the intersection of two convex sets (which is itself
    /// a convex set) and a solution for moving the sets away from each other to eliminate the
    /// intersection.
    ///
    /// @note The local point and local normal usage depends on the manifold type. For details, see
    ///   the documentation associated with the different manifold types.
    /// @note Every point adds computational overhead to the collision response calculation - so
    ///   express collision manifolds with one point if possible instead of two.
    /// @note This data structure is at least 58-bytes large (60-bytes on one 64-bit platform).
    ///
    /// @sa Contact.
    /// @sa PositionConstraint.
    /// @sa VelocityConstraint.
    /// @sa https://en.wikipedia.org/wiki/Convex_set
    ///
    class Manifold
    {
    public:
        using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;

        /// Shape index type.
        using sidx_t = std::remove_const<decltype(MaxShapeVertices)>::type;
        
        using cf_t = ContactFeature::Type;

        struct Conf;

        /// Manifold type.
        /// @note This is by design a 1-byte sized type.
        enum Type: std::uint8_t
        {
            /// Unset type.
            /// @details Manifold is unset. For manifolds of this type: the point count is zero,
            ///   point data is undefined, and all other properties are invalid.
            e_unset,
            
            /// Circles type.
            /// @details Manifold is for circle-to-circle like collisions.
            /// @note For manifolds of this type: the local point is local center of "circle-A"
            ///     (where shape A wasn't necessarily a circle but treating it as such is useful),
            ///     the local normal is invalid (and unused) and, the point count will be zero or
            ///     one where the contact feature will be
            ///               <code>ContactFeature{e_vertex, i, e_vertex, j}</code>
            ///     where i and j are indexes of the vertexes of shapes A and B respectively.
            e_circles,

            /// Face-A type.
            /// @details Indicates: local point is center of face A, local normal is normal on shape A, and the
            ///   local points of Point instances are the local center of cirlce B or a clip point of polygon B
            ///   where the contact feature will be <code>ContactFeature{e_face, i, e_vertex, j}</code> or
            ///   <code>ContactFeature{e_face, i, e_face, j} where i and j are indexes for the vertex or edge
            ///   of shapes A and B respectively.</code>.
            e_faceA,

            /// Face-B type.
            /// @details Indicates: local point is center of face B, local normal is normal on shape B, and the
            ///   local points of Point instances are the local center of cirlce A or a clip point of polygon A
            ///   where the contact feature will be <code>ContactFeature{e_face, i, e_vertex, j}</code> or
            ///   <code>ContactFeature{e_face, i, e_face, j} where i and j are indexes for the vertex or edge
            ///   of shapes A and B respectively.</code>.
            e_faceB
        };
        
        /// @brief Point data for a manifold.
        ///
        /// @details This is a contact point belonging to a contact manifold. It holds details
        /// related to the geometry and dynamics of the contact points.
        ///
        /// @note The impulses are used for internal caching and may not provide reliable contact
        ///    forces especially for high speed collisions.
        ///
        /// @note This structure is at least 20-bytes large.
        ///
        struct Point
        {
            /// @brief Local point.
            /// @details Usage depends on manifold type.
            /// @note For circles type manifolds, this is the local center of circle B.
            /// @note For face-A type manifolds, this is the local center of "cirlce" B or a clip
            /// point of shape B. It is also the point at which impulse forces should be relatively
            /// applied for position resolution.
            /// @note For face-B type manifolds, this is the local center of "circle" A or a clip
            /// point of shape A. It is also the point at which impulse forces should be relatively
            /// applied for position resolution.
            /// @note 8-bytes.
            Length2D localPoint;

            /// @brief Contact feature.
            /// @details Uniquely identifies a contact point between two shapes - A and B.
            /// @note This field is 4-bytes.
            /// @sa GetPointStates.
            ContactFeature contactFeature;
            
            /// @brief Normal impulse.
            /// @details This is the non-penetration impulse.
            /// @note This is only used for velocity constraint resolution.
            /// @note 4-bytes.
            Momentum normalImpulse = 0;
            
            /// @brief Tangent impulse.
            /// @details This is the friction impulse.
            /// @note This is only used for velocity constraint resolution.
            /// @note 4-bytes.
            Momentum tangentImpulse = 0;
        };
        
        // For Circles type manifolds...
        
        /// Gets a circles-typed manifold with one point.
        /// @param vA Local center of "circle" A.
        /// @param iA Index of vertex from shape A representing the local center of "circle" A.
        /// @param vB Local center of "circle" B.
        /// @param iB Index of vertex from shape B representing the local center of "circle" B.
        static inline Manifold GetForCircles(Length2D vA, sidx_t iA, Length2D vB, sidx_t iB) noexcept
        {
            return Manifold{e_circles, GetInvalid<UnitVec2>(), vA, 1, {{
                Point{vB, GetVertexVertexContactFeature(iA, iB)}
            }}};
        }

        // For Face A type manifolds...
        
        /// Gets a face A typed manifold.
        /// @param normalA Local normal of the face from polygon A.
        /// @param faceA Any point in local coordinates on the face whose normal was provided.
        static inline Manifold GetForFaceA(UnitVec2 normalA, Length2D faceA) noexcept
        {
            return Manifold{e_faceA, normalA, faceA, 0, {{}}};
        }
        
        /// Gets a face A typed manifold.
        /// @param ln Normal on polygon A.
        /// @param lp Center of face A.
        /// @param mp1 Manifold point 1 (of 1).
        static inline Manifold GetForFaceA(UnitVec2 ln, Length2D lp,
                                           const Point& mp1) noexcept
        {
            //assert(mp1.contactFeature.typeA == ContactFeature::e_face || mp1.contactFeature.typeB == ContactFeature::e_face);
            return Manifold{e_faceA, ln, lp, 1, {{mp1}}};
        }
        
        /// Gets a face A typed manifold.
        /// @param ln Normal on polygon A.
        /// @param lp Center of face A.
        /// @param mp1 Manifold point 1 (of 2).
        /// @param mp2 Manifold point 2 (of 2).
        static inline Manifold GetForFaceA(UnitVec2 ln, Length2D lp,
                                           const Point& mp1, const Point& mp2) noexcept
        {
            //assert(mp1.contactFeature.typeA == ContactFeature::e_face || mp1.contactFeature.typeB == ContactFeature::e_face);
            //assert(mp2.contactFeature.typeA == ContactFeature::e_face || mp2.contactFeature.typeB == ContactFeature::e_face);
            //assert(mp1.contactFeature != mp2.contactFeature);
            return Manifold{e_faceA, ln, lp, 2, {{mp1, mp2}}};
        }
        
        // For Face B...
        
        /// Gets a face B typed manifold.
        /// @param ln Normal on polygon B.
        /// @param lp Center of face B.
        static inline Manifold GetForFaceB(UnitVec2 ln, Length2D lp) noexcept
        {
            return Manifold{e_faceB, ln, lp, 0, {{}}};
        }
        
        /// Gets a face B typed manifold.
        /// @param ln Normal on polygon B.
        /// @param lp Center of face B.
        /// @param mp1 Manifold point 1.
        static inline Manifold GetForFaceB(UnitVec2 ln, Length2D lp,
                                           const Point& mp1) noexcept
        {
            //assert(mp1.contactFeature.typeA == ContactFeature::e_face || mp1.contactFeature.typeB == ContactFeature::e_face);
            return Manifold{e_faceB, ln, lp, 1, {{mp1}}};
        }
        
        /// Gets a face B typed manifold.
        /// @param ln Normal on polygon B.
        /// @param lp Center of face B.
        /// @param mp1 Manifold point 1 (of 2).
        /// @param mp2 Manifold point 2 (of 2).
        static inline Manifold GetForFaceB(UnitVec2 ln, Length2D lp,
                                           const Point& mp1, const Point& mp2) noexcept
        {
            //assert(mp1.contactFeature.typeA == ContactFeature::e_face || mp1.contactFeature.typeB == ContactFeature::e_face);
            //assert(mp2.contactFeature.typeA == ContactFeature::e_face || mp2.contactFeature.typeB == ContactFeature::e_face);
            //assert(mp1.contactFeature != mp2.contactFeature);
            return Manifold{e_faceB, ln, lp, 2, {{mp1, mp2}}};
        }
        
        static inline Manifold GetForFaceA(UnitVec2 na, sidx_t ia, Length2D pa) noexcept
        {
            return Manifold{e_faceA, na, pa, 0, {{
                Point{GetInvalid<Length2D>(), ContactFeature{ContactFeature::e_face, ia, ContactFeature::e_face, 0}},
                Point{GetInvalid<Length2D>(), ContactFeature{ContactFeature::e_face, ia, ContactFeature::e_face, 0}}
            }}};
        }
        
        static inline Manifold GetForFaceB(UnitVec2 nb, sidx_t ib, Length2D pb) noexcept
        {
            return Manifold{e_faceB, nb, pb, 0, {{
                Point{GetInvalid<Length2D>(), ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, ib}},
                Point{GetInvalid<Length2D>(), ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, ib}}
            }}};
        }

        static inline Manifold GetForFaceA(UnitVec2 na, sidx_t ia, Length2D pa,
                                              cf_t tb0, sidx_t ib0, Length2D pb0) noexcept
        {
            return Manifold{e_faceA, na, pa, 1, {{
                Point{pb0, ContactFeature{ContactFeature::e_face, ia, tb0, ib0}},
                Point{pb0, ContactFeature{ContactFeature::e_face, ia, tb0, ib0}}
            }}};
        }
        
        static inline Manifold GetForFaceB(UnitVec2 nb, sidx_t ib, Length2D pb,
                                              cf_t ta0, sidx_t ia0, Length2D pa0) noexcept
        {
            return Manifold{e_faceB, nb, pb, 1, {{
                Point{pa0, ContactFeature{ta0, ia0, ContactFeature::e_face, ib}},
                Point{pa0, ContactFeature{ta0, ia0, ContactFeature::e_face, ib}}
            }}};
        }
        
        static inline Manifold GetForFaceA(UnitVec2 na, sidx_t ia, Length2D pa,
                                              cf_t tb0, sidx_t ib0, Length2D pb0,
                                              cf_t tb1, sidx_t ib1, Length2D pb1) noexcept
        {
            return Manifold{e_faceA, na, pa, 2, {{
                Point{pb0, ContactFeature{ContactFeature::e_face, ia, tb0, ib0}},
                Point{pb1, ContactFeature{ContactFeature::e_face, ia, tb1, ib1}}
            }}};
        }

        static inline Manifold GetForFaceB(UnitVec2 nb, sidx_t ib, Length2D pb,
                                              cf_t ta0, sidx_t ia0, Length2D pa0,
                                              cf_t ta1, sidx_t ia1, Length2D pa1) noexcept
        {
            return Manifold{e_faceB, nb, pb, 2, {{
                Point{pa0, ContactFeature{ta0, ia0, ContactFeature::e_face, ib}},
                Point{pa1, ContactFeature{ta1, ia1, ContactFeature::e_face, ib}}
            }}};
        }

        /// Default constructor.
        /// @details
        /// Constructs an unset-type manifold.
        /// For an unset-type manifold:
        /// point count is zero, point data is undefined, and all other properties are invalid.
        Manifold() = default;
        
        Manifold(const Manifold& copy) = default;
        
        /// Gets the type of this manifold.
        ///
        /// @note This must be a constant expression in order to use it in the context
        ///   of the IsValid specialized template function for it.
        ///
        constexpr Type GetType() const noexcept { return m_type; }
        
        /// Gets the manifold point count.
        ///
        /// @details This is the count of contact points for this manifold.
        ///   Only up to this many points can be validly accessed using the GetPoint() method.
        /// @note Non-zero values indicate that the two shapes are touching.
        ///
        /// @return Value between 0 and MaxManifoldPoints.
        ///
        /// @sa MaxManifoldPoints.
        /// @sa AddPoint().
        /// @sa GetPoint().
        ///
        constexpr size_type GetPointCount() const noexcept { return m_pointCount; }
        
        constexpr ContactFeature GetContactFeature(size_type index) const noexcept
        {
            assert(index < m_pointCount);
            return m_points[index].contactFeature;
        }

        constexpr ContactImpulses GetContactImpulses(size_type index) const noexcept
        {
            assert(index < m_pointCount);
            return ContactImpulses{m_points[index].normalImpulse, m_points[index].tangentImpulse};
        }

        void SetContactImpulses(size_type index, ContactImpulses value) noexcept
        {
            assert(index < m_pointCount);
            m_points[index].normalImpulse = value.m_normal;
            m_points[index].tangentImpulse = value.m_tangent;
        }

        const Point& GetPoint(size_type index) const noexcept
        {
            assert((0 <= index) && (index < m_pointCount));
            return m_points[index];
        }
        
        void SetPointImpulses(size_type index, Momentum n, Momentum t)
        {
            assert((index < m_pointCount) || (index < MaxManifoldPoints && n == Momentum{0} && t == Momentum{0}));
            m_points[index].normalImpulse = n;
            m_points[index].tangentImpulse = t;
        }
        
        /// Adds a new point.
        /// @details This can be called once for circle type manifolds,
        ///   and up to twice for face-A or face-B type manifolds.
        /// GetPointCount() can be called to find out how many points have already been added.
        /// @warning Behavior is undefined if this object's type is e_unset.
        /// @warning Behavior is undefined if this is called more than twice.
        void AddPoint(const Point& mp) noexcept;

        void AddPoint(cf_t type, sidx_t index, Length2D point) noexcept;

        /// Gets the local normal for a face-type manifold.
        /// @return Local normal if the manifold type is face A or face B, else invalid value.
        constexpr UnitVec2 GetLocalNormal() const noexcept
        {
            return m_localNormal;
        }
        
        /// Gets the local point.
        /// @details
        /// This is the:
        /// local center of "circle" A for circles-type manifolds,
        /// the center of face A for face-A-type manifolds, and
        /// the center of face B for face-B-type manifolds.
        /// @note Value invalid for unset (e_unset) type manifolds.
        /// @return Local point.
        constexpr Length2D GetLocalPoint() const noexcept
        {
            return m_localPoint;
        }
        
        constexpr Length2D GetOpposingPoint(size_type index) const noexcept
        {
            assert((0 <= index) && (index < m_pointCount));
            return m_points[index].localPoint;
        }

    private:
        struct PointArray
        {
            Point elements[MaxManifoldPoints];
            constexpr Point& operator[](std::size_t i) { return elements[i]; }
            constexpr const Point& operator[](std::size_t i) const { return elements[i]; }
        };
    
        /// Constructs manifold with array of points using the given values.
        /// @param t Manifold type.
        /// @param ln Local normal.
        /// @param lp Local point.
        /// @param n number of points defined in arary.
        /// @param mpa Manifold point array.
        constexpr Manifold(Type t, UnitVec2 ln, Length2D lp, size_type n, const PointArray& mpa) noexcept;
        
        Type m_type = e_unset; ///< Type of collision this manifold is associated with (1-byte).
        size_type m_pointCount = 0; ///< Number of defined manifold points (1-byte).
        
        /// Local normal.
        /// @details Exact usage depends on manifold type (8-bytes).
        /// @note Invalid for the unset and circle manifold types.
        UnitVec2 m_localNormal = GetInvalid<decltype(m_localNormal)>();

        /// Local point.
        /// @details Exact usage depends on manifold type (8-bytes).
        /// @note Invalid for the unset manifold type.
        Length2D m_localPoint = GetInvalid<Length2D>();
        
        PointArray m_points; ///< Points of contact (at least 40-bytes). @sa pointCount.
    };
    
    struct Manifold::Conf
    {
        /// Targetted depth of impact.
        /// @note Value must be less than twice the minimum vertex radius of any shape.
        Length targetDepth = DefaultLinearSlop * Real{3};
        
        Length tolerance = DefaultLinearSlop / Real{4}; ///< Tolerance.
        
        /// Max. circles ratio.
        /// @details When the ratio of the closest face's length to the vertex radius is
        ///   more than this amount, then face-manifolds are forced, else circles-manifolds
        ///   may be computed for new contact manifolds.
        Real maxCirclesRatio = DefaultCirclesRatio;
    };
    
    constexpr inline Manifold::Conf GetDefaultManifoldConf() noexcept
    {
        return Manifold::Conf{};
    }

    bool operator==(const Manifold::Point& lhs, const Manifold::Point& rhs) noexcept;
    
    bool operator!=(const Manifold::Point& lhs, const Manifold::Point& rhs) noexcept;
    
    /// Equality operator.
    /// @note In-so-far as manifold points are concerned, order doesn't matter;
    ///    only whether the two manifolds have the same point set.
    bool operator==(const Manifold& lhs, const Manifold& rhs) noexcept;
    
    bool operator!=(const Manifold& lhs, const Manifold& rhs) noexcept;

    constexpr inline Manifold::Manifold(Type t, UnitVec2 ln, Length2D lp, size_type n,
                                              const PointArray& mpa) noexcept:
        m_type{t}, m_localNormal{ln}, m_localPoint{lp}, m_pointCount{n}, m_points{mpa}
    {
        assert(t != e_unset || n == 0);
        assert(t == e_unset || IsValid(lp));
        assert((t == e_unset) || (t == e_circles) || IsValid(ln));
        assert((t != e_circles) || (n == 1 && !IsValid(ln)));
        //assert((t != e_circles) || (n == 1 && !IsValid(ln) && mpa[0].contactFeature.typeA == ContactFeature::e_vertex && mpa[0].contactFeature.typeB == ContactFeature::e_vertex));
    }

    inline void Manifold::AddPoint(const Point& mp) noexcept
    {
        assert(m_type != e_unset);
        assert(m_type != e_circles || m_pointCount == 0);
        assert(m_pointCount < MaxManifoldPoints);
        // assert((m_pointCount == 0) || (mp.contactFeature != m_points[0].contactFeature));
        //assert((m_type != e_circles) || (mp.contactFeature.typeA == ContactFeature::e_vertex || mp.contactFeature.typeB == ContactFeature::e_vertex));
        //assert((m_type != e_faceA) || ((mp.contactFeature.typeA == ContactFeature::e_face) && (m_pointCount == 0 || mp.contactFeature.indexA == m_points[0].contactFeature.indexA)));
        //assert((m_type != e_faceB) || (mp.contactFeature.typeB == ContactFeature::e_face));
        m_points[m_pointCount] = mp;
        ++m_pointCount;
    }

    inline void Manifold::AddPoint(cf_t type, sidx_t index, Length2D point) noexcept
    {
        assert(m_pointCount < MaxManifoldPoints);
        switch (m_type)
        {
            case e_unset:
                break;
            case e_circles:
                break;
            case e_faceA:
                m_points[m_pointCount].localPoint = point;
                m_points[m_pointCount].contactFeature.typeB = type;
                m_points[m_pointCount].contactFeature.indexB = index;
                ++m_pointCount;
                break;
            case e_faceB:
                m_points[m_pointCount].localPoint = point;
                m_points[m_pointCount].contactFeature.typeA = type;
                m_points[m_pointCount].contactFeature.indexA = index;
                ++m_pointCount;
                break;
        }
    }

    template <>
    constexpr inline bool IsValid(const Manifold& value) noexcept
    {
        return value.GetType() != Manifold::e_unset;
    }
    
    /// @brief Calculates the relevant collision manifold.
    ///
    /// @note The returned touching state information typically agrees with that returned from
    ///   the DistanceProxy-based TestOverlap function. This is not always the case however
    ///   especially when the separation or overlap distance is closer to zero.
    ///
    Manifold CollideShapes(const DistanceProxy& shapeA, const Transformation& xfA,
                           const DistanceProxy& shapeB, const Transformation& xfB,
                           const Manifold::Conf conf = GetDefaultManifoldConf());
#if 0
    Manifold CollideCached(const DistanceProxy& shapeA, const Transformation& xfA,
                           const DistanceProxy& shapeB, const Transformation& xfB,
                           const Manifold::Conf conf = GetDefaultManifoldConf());
#endif

#ifdef DEFINE_GET_MANIFOLD
    Manifold GetManifold(const DistanceProxy& proxyA, const Transformation& transformA,
                         const DistanceProxy& proxyB, const Transformation& transformB);
#endif

    Length2D GetLocalPoint(const DistanceProxy& proxy, ContactFeature::Type type, ContactFeature::Index index);

    const char* GetName(Manifold::Type) noexcept;
    
} // namespace playrho

#endif /* Manifold_hpp */
