/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_TIME_STEP_H
#define B2_TIME_STEP_H

#include <Box2D/Common/Math.h>

namespace box2d {

/// Profiling data. Times are in milliseconds.
struct Profile
{
	float_t step;
	float_t collide;
	float_t solve;
	float_t solveInit;
	float_t solveVelocity;
	float_t solvePosition;
	float_t broadphase;
	float_t solveTOI;
};

/// This is an internal structure.
class TimeStep
{
public:
	using iteration_type = unsigned;

	static constexpr auto InvalidIteration = static_cast<iteration_type>(-1);

	/// Gets the delta time (time amount for this time step).
	/// @sa set_dt(float_t).
	/// @return Time step amount in seconds.
	float_t get_dt() const noexcept { return dt; }

	/// Gets the inverse delta-t value.
	/// @return 1/dt or 0 if dt is 0.
	/// @sa get_dt().
	float_t get_inv_dt() const noexcept { return inv_dt; }
	
	/// Sets the delta time value.
	/// @post Getting the delta time will return this set value.
	/// @post The inverse delta time value is the inverse of this set value or zero if the value is zero.
	/// @sa get_inv_dt().
	/// @param value Elapsed time amount (in seconds).
	void set_dt(float_t value) noexcept
	{
		dt = value;
		inv_dt = (value != 0)? float_t{1} / value: float_t{0};
	}

	/// Delta t ratio.
	/// @detail This is the delta-t times the inverse delta t from the previous world step.
	///   Value of 1 indicates that the time step has not varied.
	float_t dtRatio;

	iteration_type velocityIterations; ///< Velocity iterations.
	iteration_type positionIterations; ///< Position iterations.
	
	bool warmStarting; ///< Whether or not to perform warm starting.

private:
	float_t dt; ///< Delta time. This is the time step in seconds.
	float_t inv_dt; ///< Inverse time step (1/dt or 0 if dt == 0). @see dt.
};

/// Solver Data
struct SolverData
{
	TimeStep step;
	Position* positions;
	Velocity* velocities;
};

} // namespace box2d

#endif
