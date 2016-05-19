/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>

namespace box2d {

class Draw;

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
	float_t* masses = nullptr;

	///
	Vec2 gravity = Vec2_zero;

	///
	float_t damping = float_t(1) / float_t(10);

	/// Stretching stiffness
	float_t k2 = float_t(9) / float_t(10);

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	float_t k3 = float_t(1) / float_t(10);
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
	void Step(float_t timeStep, int32 iterations);

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

	///
	void Draw(Draw* draw) const;

	///
	void SetAngle(float_t angle);

private:

	void SolveC2();
	void SolveC3();

	size_type m_count = 0;
	Vec2* m_ps = nullptr;
	Vec2* m_p0s = nullptr;
	Vec2* m_vs = nullptr;

	float_t* m_ims = nullptr;

	float_t* m_Ls = nullptr;
	float_t* m_as = nullptr;

	Vec2 m_gravity = Vec2_zero;
	float_t m_damping = float_t{0};

	float_t m_k2 = float_t(1);
	float_t m_k3 = float_t(0.1);
};

} // namespace box2d

#endif
