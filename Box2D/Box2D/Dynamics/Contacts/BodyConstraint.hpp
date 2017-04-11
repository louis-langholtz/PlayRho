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

#ifndef BodyConstraint_hpp
#define BodyConstraint_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d {
	
	class Body;
	
	/// Body Constraint.
	/// @detail Body data related to constraint processing.
	/// @note Only position and velocity is independently changable after construction.
	/// @note This data structure is 40-bytes large (with 4-byte RealNum on at least one 64-bit platform).
	class BodyConstraint
	{
	public:
		using index_type = std::remove_const<decltype(MaxBodies)>::type;
		
		BodyConstraint() = default;
		
		constexpr BodyConstraint(InvMass invMass, InvRotInertia invRotI, Length2D localCenter,
								 Position position, Velocity velocity) noexcept:
			m_position{position},
			m_velocity{velocity},
			m_localCenter{localCenter},
			m_invMass{invMass},
			m_invRotI{invRotI}
		{
			assert(IsValid(position));
			assert(IsValid(velocity));
			assert(IsValid(localCenter));
			assert(invMass >= InvMass{0});
			assert(invRotI >= InvRotInertia{0});
		}
				
		InvMass GetInvMass() const noexcept;
		
		InvRotInertia GetInvRotInertia() const noexcept;
		
		Length2D GetLocalCenter() const noexcept;
		
		Position GetPosition() const noexcept;
		
		Velocity GetVelocity() const noexcept;
		
		BodyConstraint& SetPosition(Position value) noexcept;
		
		BodyConstraint& SetVelocity(Velocity value) noexcept;
		
	private:
		Position m_position; ///< Body position data.
		Velocity m_velocity; ///< Body velocity data.
		Length2D m_localCenter; ///< Local center of the associated body's sweep (8-bytes).
		InvMass m_invMass; ///< Inverse mass of associated body (a non-negative value, 4-bytes).
		InvRotInertia m_invRotI; ///< Inverse rotational inertia about the center of mass of the associated body (a non-negative value, 4-bytes).
	};
	
	inline InvMass BodyConstraint::GetInvMass() const noexcept
	{
		return m_invMass;
	}
	
	inline InvRotInertia BodyConstraint::GetInvRotInertia() const noexcept
	{
		return m_invRotI;
	}
	
	inline Length2D BodyConstraint::GetLocalCenter() const noexcept
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
	
	BodyConstraint GetBodyConstraint(const Body& body, Time time = 0) noexcept;

} // namespace box2d

#endif /* BodyConstraint_hpp */
