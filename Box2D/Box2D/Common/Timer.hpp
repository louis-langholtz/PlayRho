/*
* Original work Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_TIMER_H
#define B2_TIMER_H

#include <Box2D/Common/Settings.hpp>

namespace box2d {

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
class Timer
{
public:

	/// Constructor
	Timer();

	/// Reset the timer.
	void Reset();

	/// Get the time since construction or the last reset.
	realnum GetMilliseconds() const;

private:

#if defined(DO_TIMER_FOR_REALS)
#if defined(_WIN32)
	double m_start;
	static double s_invFrequency;
#elif defined(__linux__) || defined (__APPLE__)
	unsigned long m_start_sec;
	unsigned long m_start_usec;
#endif
#endif
};

} // namespace box2d

#endif
