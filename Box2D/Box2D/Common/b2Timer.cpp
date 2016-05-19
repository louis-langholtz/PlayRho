/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#include <Box2D/Common/b2Timer.h>

using namespace box2d;

#if defined(DO_TIMER_FOR_REALS)
#if defined(_WIN32)

float64 Timer::s_invFrequency = float_t{0};

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

Timer::Timer()
{
	LARGE_INTEGER largeInteger;

	if (s_invFrequency == float_t{0})
	{
		QueryPerformanceFrequency(&largeInteger);
		s_invFrequency = float64(largeInteger.QuadPart);
		if (s_invFrequency > float_t{0})
		{
			s_invFrequency = 1000.0f / s_invFrequency;
		}
	}

	QueryPerformanceCounter(&largeInteger);
	m_start = float64(largeInteger.QuadPart);
}

void Timer::Reset()
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	m_start = float64(largeInteger.QuadPart);
}

float_t Timer::GetMilliseconds() const
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	float64 count = float64(largeInteger.QuadPart);
	float_t ms = float_t(s_invFrequency * (count - m_start));
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

float_t Timer::GetMilliseconds() const
{
    timeval t;
    gettimeofday(&t, 0);
    return (float_t(1000) * (t.tv_sec - m_start_sec)) + (float_t(0.001) * (t.tv_usec - m_start_usec));
}

#else

Timer::Timer()
{
}

void Timer::Reset()
{
}

float_t Timer::GetMilliseconds() const
{
	return float_t{0};
}

#endif
#else

Timer::Timer()
{
}

void Timer::Reset()
{
}

float_t Timer::GetMilliseconds() const
{
	return float_t{0};
}

#endif

