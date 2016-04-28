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

class b2Draw;

/// 
struct b2RopeDef
{
	using size_type = b2_size_t;

	constexpr b2RopeDef() = default;

	///
	b2Vec2* vertices = nullptr;

	///
	size_type count = 0;

	///
	b2Float* masses = nullptr;

	///
	b2Vec2 gravity = b2Vec2_zero;

	///
	b2Float damping = b2Float(1) / b2Float(10);

	/// Stretching stiffness
	b2Float k2 = b2Float(9) / b2Float(10);

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	b2Float k3 = b2Float(1) / b2Float(10);
};

/// 
class b2Rope
{
public:
	using size_type = b2_size_t;

	constexpr b2Rope() = default;
	~b2Rope();

	///
	void Initialize(const b2RopeDef* def);

	///
	void Step(b2Float timeStep, int32 iterations);

	///
	size_type GetVertexCount() const noexcept
	{
		return m_count;
	}

	///
	const b2Vec2* GetVertices() const noexcept
	{
		return m_ps;
	}

	///
	void Draw(b2Draw* draw) const;

	///
	void SetAngle(b2Float angle);

private:

	void SolveC2();
	void SolveC3();

	size_type m_count = 0;
	b2Vec2* m_ps = nullptr;
	b2Vec2* m_p0s = nullptr;
	b2Vec2* m_vs = nullptr;

	b2Float* m_ims = nullptr;

	b2Float* m_Ls = nullptr;
	b2Float* m_as = nullptr;

	b2Vec2 m_gravity = b2Vec2_zero;
	b2Float m_damping = b2Float{0};

	b2Float m_k2 = b2Float(1);
	b2Float m_k3 = b2Float(0.1);
};

#endif
