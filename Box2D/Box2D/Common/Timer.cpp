/*
* Original work Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Timer.hpp>

using namespace box2d;

#if defined(DO_TIMER_FOR_REALS)
#if defined(_WIN32)

double Timer::s_invFrequency = RealNum{0};

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

Timer::Timer()
{
	LARGE_INTEGER largeInteger;

	if (s_invFrequency == RealNum{0})
	{
		QueryPerformanceFrequency(&largeInteger);
		s_invFrequency = double(largeInteger.QuadPart);
		if (s_invFrequency > RealNum{0})
		{
			s_invFrequency = 1000.0f / s_invFrequency;
		}
	}

	QueryPerformanceCounter(&largeInteger);
	m_start = double(largeInteger.QuadPart);
}

void Timer::Reset()
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	m_start = double(largeInteger.QuadPart);
}

RealNum Timer::GetMilliseconds() const
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	double count = double(largeInteger.QuadPart);
	RealNum ms = RealNum(s_invFrequency * (count - m_start));
	return ms;
}

#elif defined(__linux__) || defined (__APPLE__)

#include <sys/time.h>

Timer::Timer()
{
    Reset();
}

void Timer::Reset()
{
    timeval t;
    gettimeofday(&t, 0);
    m_start_sec = t.tv_sec;
    m_start_usec = t.tv_usec;
}

RealNum Timer::GetMilliseconds() const
{
    timeval t;
    gettimeofday(&t, 0);
    return (RealNum(1000) * (t.tv_sec - m_start_sec)) + (RealNum(0.001) * (t.tv_usec - m_start_usec));
}

#else

Timer::Timer()
{
}

void Timer::Reset()
{
}

RealNum Timer::GetMilliseconds() const
{
	return RealNum{0};
}

#endif
#else

Timer::Timer()
{
}

void Timer::Reset()
{
}

RealNum Timer::GetMilliseconds() const
{
	return RealNum{0};
}

#endif

