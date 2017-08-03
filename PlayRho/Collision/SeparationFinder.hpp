/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef SeparationFinder_hpp
#define SeparationFinder_hpp

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/IndexPair.hpp>

namespace playrho {

    class DistanceProxy;
    struct Transformation;
        
    /// Separation finder.
    class SeparationFinder
    {
    public:
        
        /// Separation finder type.
        enum Type
        {
            e_points,
            e_faceA,
            e_faceB
        };
        
        /// Separation finder data.
        struct Data
        {
            /// Index pair.
            /// @details Pair of indices of vertices for which distance is being returned for.
            /// @note The <code>a</code> index in this pair will be <code>InvalidIndex</code> for
            ///   face-A type separarion finders.
            /// @note The <code>b</code> index in this pair will be <code>InvalidIndex</code> for
            ///   face-B type separarion finders.
            IndexPair indexPair;

            /// Distance.
            /// @details Distance of separation between vertices indexed by the index-pair.
            Length distance;
        };
        
        /// Gets a separation finder for the given inputs.
        ///
        /// @warning Behavior is undefined if given less than one index pair or more than three.
        ///
        /// @param indices Collection of 1 to 3 index pairs. A points-type finder will be
        ///    returned if given 1 index pair. A face-type finder will be returned otherwise.
        /// @param proxyA Proxy A.
        /// @param xfA Transformation A.
        /// @param proxyB Proxy B.
        /// @param xfB Transformation B.
        ///
        static SeparationFinder Get(IndexPair3 indices,
                                    const DistanceProxy& proxyA, const Transformation& xfA,
                                    const DistanceProxy& proxyB, const Transformation& xfB);
        
        /// Finds the minimum separation.
        /// @return indexes of proxy A's and proxy B's vertices that have the minimum
        ///    distance between them and what that distance is.
        Data FindMinSeparation(const Transformation& xfA, const Transformation& xfB) const
        {
            switch (m_type)
            {
                case e_points: return FindMinSeparationForPoints(xfA, xfB);
                case e_faceA: return FindMinSeparationForFaceA(xfA, xfB);
                case e_faceB: return FindMinSeparationForFaceB(xfA, xfB);
            }
            
            // Should never be reached
            assert(false);
            return Data{IndexPair{IndexPair::InvalidIndex, IndexPair::InvalidIndex}, 0};
        }
        
        /// Evaluates the separation of the identified proxy vertices at the given time factor.
        ///
        /// @param indexPair Indexes of the proxy A and proxy B vertexes.
        /// @param xfA Transformation A.
        /// @param xfB Transformation B.
        ///
        /// @return Separation distance which will be negative when the given transforms put the
        ///    vertices on the opposite sides of the separating axis.
        ///
        Length Evaluate(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
        {
            switch (m_type)
            {
                case e_points: return EvaluateForPoints(indexPair, xfA, xfB);
                case e_faceA: return EvaluateForFaceA(indexPair, xfA, xfB);
                case e_faceB: return EvaluateForFaceB(indexPair, xfA, xfB);
                default: break;
            }
            assert(false);
            return Length{0};
        }
        
        constexpr Type GetType() const noexcept;
        constexpr UnitVec2 GetAxis() const noexcept;
        constexpr Length2D GetLocalPoint() const noexcept;

    private:
        constexpr SeparationFinder(const DistanceProxy& dpA, const DistanceProxy& dpB,
                                         const UnitVec2 axis, const Length2D lp, const Type type):
            m_proxyA{dpA}, m_proxyB{dpB}, m_axis{axis}, m_localPoint{lp}, m_type{type}
        {
            // Intentionally empty.
        }
        
        Data FindMinSeparationForPoints(const Transformation& xfA, const Transformation& xfB) const;
        
        Data FindMinSeparationForFaceA(const Transformation& xfA, const Transformation& xfB) const;
        
        Data FindMinSeparationForFaceB(const Transformation& xfA, const Transformation& xfB) const;
        
        Length EvaluateForPoints(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
        
        Length EvaluateForFaceA(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
        
        Length EvaluateForFaceB(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
        
        const DistanceProxy& m_proxyA;
        const DistanceProxy& m_proxyB;
        const UnitVec2 m_axis; ///< Axis. @details Directional vector of the axis of separation.
        const Length2D m_localPoint; ///< Local point. @note Only used if type is e_faceA or e_faceB.
        const Type m_type;
    };

    constexpr inline SeparationFinder::Type SeparationFinder::GetType() const noexcept
    {
        return m_type;
    }
    
    constexpr inline UnitVec2 SeparationFinder::GetAxis() const noexcept
    {
        return m_axis;
    }
    
    constexpr inline Length2D SeparationFinder::GetLocalPoint() const noexcept
    {
        return m_localPoint;
    }

}

#endif /* SeparationFinder_hpp */
