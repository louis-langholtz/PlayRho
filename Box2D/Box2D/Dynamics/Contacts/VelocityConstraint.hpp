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

#ifndef VelocityConstraint_hpp
#define VelocityConstraint_hpp

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Span.hpp>

namespace box2d {
	
	class WorldManifold;
	
	/// Contact velocity constraint.
	///
	/// @note A valid contact velocity constraint must have a point count of either 1 or 2.
	/// @note This data structure is 184-bytes large (on at least one 64-bit platform).
	///
	/// @invariant The "K" value cannot be changed independent of: the total inverse mass,
	///   the normal, and the point relative positions.
	/// @invariant The normal mass cannot be changed independent of: the "K" value.
	/// @invariant The velocity biasses cannot be changed independent of: the normal, and the
	///   point relative positions.
	///
	class VelocityConstraint
	{
	public:
		using size_type = std::remove_const<decltype(MaxManifoldPoints)>::type;
		using index_type = size_t;

		struct Conf
		{
			RealNum dtRatio;
			RealNum velocityThreshold;
			bool blockSolve;
		};

		/// Contact velocity constraint body data.
		/// @invariant The inverse mass is a value of zero or more.
		/// @invariant The inverse rotational inertia is a value of zero or more.
		class BodyData
		{
		public:
			BodyData() noexcept = default;
			BodyData(const BodyData& copy) noexcept = default;
			
			/// Initializing constructor.
			/// @note Behavior is undefined if the given inverse mass or given inverse rotational
			///   inertia is less than zero.
			/// @param invMass Inverse mass. A value of 0 or more.
			/// @param invI Inverse rotational inertia. A value of 0 or more.
			constexpr BodyData(index_type index, RealNum invMass, RealNum invI) noexcept:
				m_index{index}, m_invMass{invMass}, m_invI{invI}
			{
				assert(invMass >= 0);
				assert(invI >= 0);
			}
			
			index_type GetIndex() const noexcept { return m_index; }
			
			/// Gets the inverse mass.
			/// @return 0 or greater value.
			RealNum GetInvMass() const noexcept { return m_invMass; }
			
			/// Gets the inverse rotational inertia.
			/// @return 0 or greater value.
			RealNum GetInvRotI() const noexcept { return m_invI; }
			
		private:
			RealNum m_invMass = 0; ///< Inverse mass of body. Value of 0 or greater.
			RealNum m_invI = 0; ///< Inverse rotational interia of body. Value of 0 or greater.
			index_type m_index = GetInvalid<index_type>(); ///< Index within island of body.
		};
		
		/// Default constructor.
		/// @detail
		/// Initializes object with: a zero point count, an invalid K, an invalid normal mass,
		/// an invalid normal, invalid friction, invalid restitution, an invalid tangent speed.
		VelocityConstraint() = default;
		
		VelocityConstraint(const VelocityConstraint& copy) = default;

		VelocityConstraint& operator= (const VelocityConstraint& copy) = default;
		
		VelocityConstraint(index_type contactIndex,
						   RealNum friction, RealNum restitution, RealNum tangentSpeed,
						   BodyData bA, BodyData bB,
						   UnitVec2 normal);

		/// Adds the given point to this contact velocity constraint object.
		/// @detail Adds up to <code>MaxManifoldPoints</code> points. To find out how many points have already
		///   been added, call GetPointCount().
		/// @note Behavior is undefined if an attempt is made to add more than MaxManifoldPoints points.
		/// @sa GetPointCount().
		void AddPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, Velocity velA, Velocity velB, Conf conf);

		/// Gets the normal of the contact in world coordinates.
		/// @note This value is set on construction.
		/// @return Contact normal (in world coordinates) if previously set, an invalid value
		///   otherwise.
		UnitVec2 GetNormal() const noexcept { return m_normal; }

		UnitVec2 GetTangent() const noexcept { return m_tangent; }
		
		RealNum GetInverseMass() const noexcept { return m_invMass; }

		/// Gets the count of points added to this object.
		/// @return Value between 0 and MaxManifoldPoints
		/// @sa MaxManifoldPoints.
		/// @sa AddPoint.
		size_type GetPointCount() const noexcept { return m_pointCount; }
		
		Mat22 ComputeK() const noexcept;

		/// Gets the "K" value.
		/// @note This value is only valid if previously set.
		/// @note Call the <code>SetK</code> method to set this value.
		/// @return "K" value previously set or an invalid value.
		Mat22 GetK() const noexcept;
		
		/// Gets the normal mass.
		/// @note This value is only valid if previously set.
		/// @return normal mass previously set or an invalid value.
		Mat22 GetNormalMass() const noexcept;
		
		/// Gets the contact index.
		/// @note This value can only be set via the initializing constructor.
		/// @return Index of the associated contact (the index of the contact that this constraint is for).
		index_type GetContactIndex() const noexcept { return m_contactIndex; }
		
		/// Gets the combined friction of the associated contact.
		RealNum GetFriction() const noexcept { return m_friction; }
		
		/// Gets the combined restitution of the associated contact.
		RealNum GetRestitution() const noexcept { return m_restitution; }
		
		/// Gets the tangent speed of the associated contact.
		RealNum GetTangentSpeed() const noexcept { return m_tangentSpeed; }
		
		BodyData bodyA; ///< Body A contact velocity constraint data.
		
		BodyData bodyB; ///< Body B contact velocity constraint data.
		
		/// Gets the normal impulse at the given point.
		/// @note Call the <code>AddPoint</code> or <code>SetNormalImpulseAtPoint</code> method
		///   to set this value.
		/// @return Value previously set, or an invalid value.
		/// @sa SetNormalImpulseAtPoint.
		RealNum GetNormalImpulseAtPoint(size_type index) const noexcept;
		
		/// Gets the tangent impulse at the given point.
		/// @note Call the <code>AddPoint</code> or <code>SetTangentImpulseAtPoint</code> method
		///   to set this value.
		/// @return Value previously set, or an invalid value.
		/// @sa SetTangentImpulseAtPoint.
		RealNum GetTangentImpulseAtPoint(size_type index) const noexcept;
		
		/// Gets the velocity bias at the given point.
		/// @note The <code>AddPoint</code> method sets this value.
		/// @return Previously set value or an invalid value.
		RealNum GetVelocityBiasAtPoint(size_type index) const noexcept;

		/// Gets the normal mass at the given point.
		/// @note This value depends on the values of:
		///   the sum of the inverse-masses of the two bodies,
		///   the bodies' inverse-rotational-inertia,
		///   the point-relative A and B positions, and
		///   the normal.
		/// @note The <code>AddPoint</code> method sets this value.
		RealNum GetNormalMassAtPoint(size_type index) const noexcept;
		
		/// Gets the tangent mass at the given point.
		/// @note This value depends on the values of:
		///   the sum of the inverse-masses of the two bodies,
		///   the bodies' inverse-rotational-inertia,
		///   the point-relative A and B positions, and
		///   the tangent.
		/// @note The <code>AddPoint</code> method sets this value.
		RealNum GetTangentMassAtPoint(size_type index) const noexcept;

		/// Gets the point relative position of A.
		/// @note The <code>AddPoint</code> method sets this value.
		/// @return Previously set value or an invalid value.
		Vec2 GetPointRelPosA(size_type index) const noexcept;

		/// Gets the point relative position of B.
		/// @note The <code>AddPoint</code> method sets this value.
		/// @return Previously set value or an invalid value.
		Vec2 GetPointRelPosB(size_type index) const noexcept;
		
		void SetNormalImpulseAtPoint(size_type index, RealNum value);
		
		void SetTangentImpulseAtPoint(size_type index, RealNum value);

		/// Removes the last point added.
		void RemovePoint() noexcept;
		
		/// Sets this object's K value.
		/// @param value A position constraint dependent value or the zero matrix (Mat22_zero).
		void SetK(const Mat22& value) noexcept;

		/// Velocity constraint point.
		/// @note This structure is at least 36-bytes large.
		struct Point
		{
			Vec2 rA = GetInvalid<decltype(rA)>(); ///< Position of body A relative to world manifold point (8-bytes).
			Vec2 rB = GetInvalid<decltype(rB)>(); ///< Position of body B relative to world manifold point (8-bytes).
			RealNum normalImpulse = GetInvalid<decltype(normalImpulse)>(); ///< Normal impulse (4-bytes).
			RealNum tangentImpulse = GetInvalid<decltype(tangentImpulse)>(); ///< Tangent impulse (4-bytes).
			RealNum normalMass = GetInvalid<decltype(normalMass)>(); ///< Normal mass (4-bytes). Dependent on rA and rB. 0 or greater.
			RealNum tangentMass = GetInvalid<decltype(tangentMass)>(); ///< Tangent mass (4-bytes). Dependent on rA and rB. 0 or greater.
			RealNum velocityBias = GetInvalid<decltype(velocityBias)>(); ///< Velocity bias (4-bytes).
		};

		/// Accesses the point identified by the given index.
		/// @note Behavior is undefined if given index is not less than <code>MaxManifoldPoints</code>.
		/// @param index Index of the point to return. This should be a value less than returned by GetPointCount().
		/// @return velocity constraint point for the given index. This point's data will be invalid
		///   unless previously added and set.
		/// @sa GetPointCount.
		const Point& GetPointAt(size_type index) const
		{
			assert(index < MaxManifoldPoints);
			return m_points[index];
		}

	private:
	
		Point GetPoint(RealNum normalImpulse, RealNum tangentImpulse, Vec2 rA, Vec2 rB, Velocity velA, Velocity velB, Conf conf) const noexcept;

		/// Accesses the point identified by the given index.
		/// @note Behavior is undefined if given index is not less than <code>MaxManifoldPoints</code>.
		/// @param index Index of the point to return. This should be a value less than returned by GetPointCount().
		/// @return velocity constraint point for the given index. This point's data will be invalid
		///   unless previously added and set.
		/// @sa GetPointCount.
		Point& PointAt(size_type index)
		{
			assert(index < MaxManifoldPoints);
			return m_points[index];
		}

		UnitVec2 m_normal = GetInvalid<UnitVec2>(); ///< Normal of the world manifold. 8-bytes.
		UnitVec2 m_tangent = GetInvalid<UnitVec2>(); ///< Tangent of the world manifold. 8-bytes.

		RealNum m_invMass = GetInvalid<RealNum>(); ///< Total inverse mass.

		/// Friction coefficient (4-bytes). Usually in the range of [0,1].
		RealNum m_friction = GetInvalid<RealNum>();
		
		RealNum m_restitution = GetInvalid<RealNum>(); ///< Restitution coefficient (4-bytes).
		
		RealNum m_tangentSpeed = GetInvalid<RealNum>(); ///< Tangent speed (4-bytes).
		
		/// Index of the contact that this constraint is for (typically 8-bytes).
		index_type m_contactIndex = GetInvalid<index_type>();
		
		// K and normalMass fields are only used for the block solver.

		/// Block solver "K" info.
		/// @note Depends on the total inverse mass, the normal, and the point relative positions.
		/// @note Only used by block solver.
		/// @note This field is 16-bytes (on at least one 64-bit platform).
		Mat22 m_K = GetInvalid<Mat22>();

		/// Normal mass information.
		/// @detail This is the cached inverse of the K value or an invalid value.
		/// @note Depends on the K value.
		/// @note Only used by block solver.
		/// @note This field is 16-bytes (on at least one 64-bit platform).
		Mat22 m_normalMass = GetInvalid<Mat22>();
		
		Point m_points[MaxManifoldPoints]; ///< Velocity constraint points array (at least 72-bytes).
		size_type m_pointCount = 0; ///< Point count (at least 1-byte).
	};
		
	inline void VelocityConstraint::RemovePoint() noexcept
	{
		assert(m_pointCount > 0);
		--m_pointCount;
	}

	inline void VelocityConstraint::SetK(const Mat22& value) noexcept
	{
		m_K = value;
		m_normalMass = (IsValid(value))? Invert(value): GetInvalid<Mat22>();
	}
	
	/// Gets the "K" value.
	/// @return "K" value or an invalid Mat22 if no other value has been set.
	/// @sa SetK.
	inline Mat22 VelocityConstraint::GetK() const noexcept
	{
		return m_K;
	}
	
	/// Gets the "normal mass" value.
	/// @return "normal mass" value or the invalid Mat22 if no other value has been set.
	/// @sa SetK.
	inline Mat22 VelocityConstraint::GetNormalMass() const noexcept
	{
		return m_normalMass;
	}

	inline Vec2 VelocityConstraint::GetPointRelPosA(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).rA;
	}
	
	inline Vec2 VelocityConstraint::GetPointRelPosB(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).rB;
	}

	/// Gets the normal of the velocity constraint contact in world coordinates.
	/// @note This value is set via the velocity constraint's <code>SetNormal</code> method.
	/// @return Contact normal (in world coordinates) if previously set, an invalid value
	///   otherwise.
	inline UnitVec2 GetNormal(const VelocityConstraint& vc) noexcept
	{
		return vc.GetNormal();
	}

	inline UnitVec2 GetTangent(const VelocityConstraint& vc) noexcept
	{
		return vc.GetTangent();
	}

	inline RealNum GetInverseMass(const VelocityConstraint& vc) noexcept
	{
		return vc.bodyA.GetInvMass() + vc.bodyB.GetInvMass();
	}
	
	inline Vec2 GetPointRelPosA(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetPointRelPosA(index);
	}
	
	inline Vec2 GetPointRelPosB(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetPointRelPosB(index);
	}
	
	inline RealNum VelocityConstraint::GetVelocityBiasAtPoint(size_type index) const noexcept
	{
		return GetPointAt(index).velocityBias;
	}
	
	inline RealNum GetVelocityBiasAtPoint(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetVelocityBiasAtPoint(index);
	}

	inline RealNum VelocityConstraint::GetNormalMassAtPoint(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).normalMass;
	}
	
	inline RealNum VelocityConstraint::GetTangentMassAtPoint(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).tangentMass;
	}

	inline RealNum GetNormalMassAtPoint(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetNormalMassAtPoint(index);
	}
	
	inline RealNum GetTangentMassAtPoint(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetTangentMassAtPoint(index);
	}

	inline RealNum VelocityConstraint::GetNormalImpulseAtPoint(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).normalImpulse;
	}
	
	inline RealNum VelocityConstraint::GetTangentImpulseAtPoint(VelocityConstraint::size_type index) const noexcept
	{
		return GetPointAt(index).tangentImpulse;
	}
	
	inline RealNum GetNormalImpulseAtPoint(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetNormalImpulseAtPoint(index);
	}
	
	inline RealNum GetTangentImpulseAtPoint(const VelocityConstraint& vc, VelocityConstraint::size_type index)
	{
		return vc.GetTangentImpulseAtPoint(index);
	}
	
	inline Vec2 GetNormalImpulses(const VelocityConstraint& vc)
	{
		return Vec2{GetNormalImpulseAtPoint(vc, 0), GetNormalImpulseAtPoint(vc, 1)};
	}
	
	inline Vec2 GetTangentImpulses(const VelocityConstraint& vc)
	{
		return Vec2{GetTangentImpulseAtPoint(vc, 0), GetTangentImpulseAtPoint(vc, 1)};
	}
	
	inline void VelocityConstraint::SetNormalImpulseAtPoint(VelocityConstraint::size_type index, RealNum value)
	{
		PointAt(index).normalImpulse = value;
	}

	inline void VelocityConstraint::SetTangentImpulseAtPoint(VelocityConstraint::size_type index, RealNum value)
	{
		PointAt(index).tangentImpulse = value;		
	}

	inline void SetNormalImpulseAtPoint(VelocityConstraint& vc, VelocityConstraint::size_type index, RealNum value)
	{
		vc.SetNormalImpulseAtPoint(index, value);
	}
	
	inline void SetTangentImpulseAtPoint(VelocityConstraint& vc, VelocityConstraint::size_type index, RealNum value)
	{
		vc.SetTangentImpulseAtPoint(index, value);		
	}
	
	inline void SetNormalImpulses(VelocityConstraint& vc, const Vec2 impulses)
	{
		SetNormalImpulseAtPoint(vc, 0, impulses[0]);
		SetNormalImpulseAtPoint(vc, 1, impulses[1]);
	}
	
	inline void SetTangentImpulses(VelocityConstraint& vc, const Vec2 impulses)
	{
		SetTangentImpulseAtPoint(vc, 0, impulses[0]);
		SetTangentImpulseAtPoint(vc, 1, impulses[1]);
	}

} // namespace box2d

#endif /* VelocityConstraint_hpp */
