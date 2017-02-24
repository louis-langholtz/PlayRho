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

#ifndef BodyConstraint_hpp
#define BodyConstraint_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d {
	
	/// Body Constraint.
	/// @detail Body data related to constraint processing.
	/// @note Only position and velocity is independently changable after construction.
	/// @note This data structure is 44-bytes large (with 4-byte RealNum on at least one 64-bit platform).
	class BodyConstraint
	{
	public:
		using index_type = std::remove_const<decltype(MaxBodies)>::type;
		
		BodyConstraint() noexcept = default;
		
		constexpr BodyConstraint(index_type index, RealNum invMass, RealNum invRotI, Vec2 localCenter,
								 Position position, Velocity velocity) noexcept:
			m_position{position},
			m_velocity{velocity},
			m_localCenter{localCenter},
			m_invMass{invMass},
			m_invRotI{invRotI},
			m_index{index}
		{
			assert(IsValid(position));
			assert(IsValid(velocity));
			assert(IsValid(localCenter));
			assert(invMass >= 0);
			assert(invRotI >= 0);
		}
		
		index_type GetIndex() const noexcept;
		
		RealNum GetInvMass() const noexcept;
		
		RealNum GetInvRotI() const noexcept;
		
		Vec2 GetLocalCenter() const noexcept;
		
		Position GetPosition() const noexcept;
		
		Velocity GetVelocity() const noexcept;
		
		BodyConstraint& SetPosition(Position value) noexcept;
		
		BodyConstraint& SetVelocity(Velocity value) noexcept;
		
	private:
		Position m_position; ///< Body position data.
		Velocity m_velocity; ///< Body velocity data.
		Vec2 m_localCenter; ///< Local center of the associated body's sweep (8-bytes).
		RealNum m_invMass; ///< Inverse mass of associated body (a non-negative value, 4-bytes).
		RealNum m_invRotI; ///< Inverse rotational inertia about the center of mass of the associated body (a non-negative value, 4-bytes).
		index_type m_index; ///< Index within island of the associated body (2-bytes).
	};
	
	inline BodyConstraint::index_type BodyConstraint::GetIndex() const noexcept
	{
		return m_index;
	}
	
	inline RealNum BodyConstraint::GetInvMass() const noexcept
	{
		return m_invMass;
	}
	
	inline RealNum BodyConstraint::GetInvRotI() const noexcept
	{
		return m_invRotI;
	}
	
	inline Vec2 BodyConstraint::GetLocalCenter() const noexcept
	{
		return m_localCenter;
	}
	
	inline Position BodyConstraint::GetPosition() const noexcept
	{
		return m_position;
	}
	
	inline Velocity BodyConstraint::GetVelocity() const noexcept
	{
		return m_velocity;
	}
	
	inline BodyConstraint& BodyConstraint::SetPosition(Position value) noexcept
	{
		m_position = value;
		return *this;
	}
	
	inline BodyConstraint& BodyConstraint::SetVelocity(Velocity value) noexcept
	{
		m_velocity = value;
		return *this;
	}
	
} // namespace box2d

#endif /* BodyConstraint_hpp */
