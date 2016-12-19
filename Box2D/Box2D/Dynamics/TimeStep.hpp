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

#include <Box2D/Common/Settings.hpp>

namespace box2d {

/// Time step.
/// @detail Step configuration data.
/// @note This data structure is 36-bytes large (on at least one 64-bit platform).
class TimeStep
{
public:
	using iteration_type = ts_iters_t;

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
	float_t dtRatio = 1;

	/// The time that a body must be still before it will go to sleep.
	float_t minStillTimeToSleep = float_t{1} / 2; // aka 0.5

	/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	/// that overlap is removed in one time step. However using values close to 1 often lead
	/// to overshoot.
	float_t regResolutionRate = float_t{2} / 10; // aka 0.2.
	
	/// Time of impact resolution rate.
	float_t toiResolutionRate = float_t{75} / 100; // aka .75

	/// A velocity threshold for elastic collisions. Any collision with a relative linear
	/// velocity below this threshold will be treated as inelastic.
	float_t velocityThreshold = float_t{8} / 10; // float_t{1};

	iteration_type velocityIterations = 8; ///< Velocity iterations.
	iteration_type positionIterations = 3; ///< Position iterations.
	iteration_type maxTOIRootIterCount = MaxTOIRootIterCount;
	iteration_type maxTOIIterations = MaxTOIIterations;
	
	bool warmStarting; ///< Whether or not to perform warm starting.

private:
	float_t dt; ///< Delta time. This is the time step in seconds.
	float_t inv_dt; ///< Inverse time step (1/dt or 0 if dt == 0). @see dt.
};

} // namespace box2d

#endif
