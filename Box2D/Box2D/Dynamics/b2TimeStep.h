/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_TIME_STEP_H
#define B2_TIME_STEP_H

#include <Box2D/Common/b2Math.h>

namespace box2d {

/// Profiling data. Times are in milliseconds.
struct b2Profile
{
	b2Float step;
	b2Float collide;
	b2Float solve;
	b2Float solveInit;
	b2Float solveVelocity;
	b2Float solvePosition;
	b2Float broadphase;
	b2Float solveTOI;
};

/// This is an internal structure.
class b2TimeStep
{
public:
	b2Float get_dt() const noexcept { return dt; }
	b2Float get_inv_dt() const noexcept { return inv_dt; }
	
	void set_dt(b2Float value) noexcept
	{
		dt = value;
		inv_dt = (value > b2Float(0))? b2Float(1) / value: b2Float(0);
	}

	b2Float dtRatio; ///< dt * inv_dt0
	int32 velocityIterations; ///< Velocity iterations.
	int32 positionIterations; ///< Position iterations.
	bool warmStarting; ///< Whether or not to perform warm starting.

private:
	b2Float dt; ///< The time step - delta time.
	b2Float inv_dt; ///< Inverse time step (1/dt or 0 if dt == 0). @see dt.
};

/// This is an internal structure.
struct b2Position
{
	b2Position() = default;
	constexpr b2Position(const b2Position& copy) = default;

	constexpr b2Position(b2Vec2 c_, b2Float a_) noexcept: c(c_), a(a_) {}

	b2Vec2 c; ///< linear position
	b2Float a; ///< angular position
};

inline b2Transform b2Displace(const b2Position& pos, const b2Vec2& local_ctr)
{
	return b2Displace(pos.c, b2Rot(pos.a), local_ctr);
}

/// This is an internal structure.
struct b2Velocity
{
	b2Velocity() = default;
	constexpr b2Velocity(b2Vec2 v_, b2Float w_) noexcept: v(v_), w(w_) {}

	b2Vec2 v; ///< Linear velocity.
	b2Float w; ///< Angular velocity.
};

/// Solver Data
struct b2SolverData
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
};

} // namespace box2d

#endif
