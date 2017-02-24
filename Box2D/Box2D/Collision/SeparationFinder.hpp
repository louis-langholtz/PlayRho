/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef SeparationFinder_hpp
#define SeparationFinder_hpp

#include <Box2D/Common/Math.hpp>
#include <Box2D/Collision/IndexPair.hpp>

namespace box2d {

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
			/// @detail Pair of indices of vertices for which distance is being returned for.
			/// @note The <code>a</code> index in this pair will be <code>InvalidIndex</code> for
			///   face-A type separarion finders.
			/// @note The <code>b</code> index in this pair will be <code>InvalidIndex</code> for
			///   face-B type separarion finders.
			IndexPair indexPair;

			/// Distance.
			/// @detail Distance of separation (in meters) between vertices indexed by the index-pair.
			RealNum distance;
		};
		
		/// Gets a separation finder for the given inputs.
		/// @warning Behavior is undefined if given less than one index pair or more than three.
		/// @param indices Collection of 1 to 3 index pairs. A points-type finder will be
		///    returned if given 1 index pair. A face-type finder will be returned otherwise.
		static SeparationFinder Get(Span<const IndexPair> indices,
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
		/// @param indexPair Indexes of the proxy A and proxy B vertexes.
		/// @return Separation distance which will be negative when the given transforms put the
		///    vertices on the opposite sides of the separating axis.
		RealNum Evaluate(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const
		{
			switch (m_type)
			{
				case e_points: return EvaluateForPoints(indexPair, xfA, xfB);
				case e_faceA: return EvaluateForFaceA(indexPair, xfA, xfB);
				case e_faceB: return EvaluateForFaceB(indexPair, xfA, xfB);
				default: break;
			}
			assert(false);
			return RealNum{0};
		}
		
		constexpr Type GetType() const noexcept;
		constexpr UnitVec2 GetAxis() const noexcept;
		constexpr Vec2 GetLocalPoint() const noexcept;

	private:
		SeparationFinder(const DistanceProxy& dpA, const DistanceProxy& dpB,
						 const UnitVec2 axis, const Vec2 lp, const Type type):
			m_proxyA{dpA}, m_proxyB{dpB}, m_axis{axis}, m_localPoint{lp}, m_type{type}
		{
			// Intentionally empty.
		}
		
		Data FindMinSeparationForPoints(const Transformation& xfA, const Transformation& xfB) const;
		
		Data FindMinSeparationForFaceA(const Transformation& xfA, const Transformation& xfB) const;
		
		Data FindMinSeparationForFaceB(const Transformation& xfA, const Transformation& xfB) const;
		
		RealNum EvaluateForPoints(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
		
		RealNum EvaluateForFaceA(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
		
		RealNum EvaluateForFaceB(IndexPair indexPair, const Transformation& xfA, const Transformation& xfB) const;
		
		const DistanceProxy& m_proxyA;
		const DistanceProxy& m_proxyB;
		const UnitVec2 m_axis; ///< Axis. @detail Directional vector of the axis of separation.
		const Vec2 m_localPoint; ///< Local point. @note Only used if type is e_faceA or e_faceB.
		const Type m_type;
	};

	constexpr SeparationFinder::Type SeparationFinder::GetType() const noexcept
	{
		return m_type;
	}
	
	constexpr UnitVec2 SeparationFinder::GetAxis() const noexcept
	{
		return m_axis;
	}
	
	constexpr Vec2 SeparationFinder::GetLocalPoint() const noexcept
	{
		return m_localPoint;
	}

}

#endif /* SeparationFinder_hpp */
