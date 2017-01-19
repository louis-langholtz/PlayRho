/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include <Box2D/Common/Math.hpp>

namespace box2d {

/// 
struct RopeDef
{
	using size_type = size_t;

	constexpr RopeDef() = default;

	///
	Vec2* vertices = nullptr;

	///
	size_type count = 0;

	///
	RealNum* masses = nullptr;

	///
	Vec2 gravity = Vec2_zero;

	///
	RealNum damping = RealNum{1} / RealNum(10);

	/// Stretching stiffness
	RealNum k2 = RealNum(9) / RealNum(10);

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	RealNum k3 = RealNum{1} / RealNum(10);
};

/// 
class Rope
{
public:
	using size_type = size_t;

	constexpr Rope() = default;
	~Rope();

	///
	void Initialize(const RopeDef* def);

	///
	void Step(RealNum timeStep, int32 iterations);

	///
	size_type GetVertexCount() const noexcept
	{
		return m_count;
	}

	///
	const Vec2* GetVertices() const noexcept
	{
		return m_ps;
	}

	Vec2 GetVertex(size_type index) const noexcept
	{
		assert(index < m_count);
		return m_ps[index];
	}

	///
	void SetAngle(RealNum angle);

private:

	void SolveC2();
	void SolveC3();

	size_type m_count = 0;
	Vec2* m_ps = nullptr;
	Vec2* m_p0s = nullptr;
	Vec2* m_vs = nullptr;

	RealNum* m_ims = nullptr;

	RealNum* m_Ls = nullptr;
	RealNum* m_as = nullptr;

	Vec2 m_gravity = Vec2_zero;
	RealNum m_damping = RealNum{0};

	RealNum m_k2 = RealNum{1};
	RealNum m_k3 = RealNum(0.1);
};

} // namespace box2d

#endif
