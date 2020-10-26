/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Dynamics/Body.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Body, FixturesByteSize)
{
    // Size is arch, os, or library dependent.
#ifdef __APPLE__
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#elif __linux__
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#elif _WIN64
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(32));
#else
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(24));
#endif
#elif _WIN32
#if !defined(NDEBUG)
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(16));
#else
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(12));
#endif
#else
    // Intentionally fail for unknown platform...
    EXPECT_EQ(sizeof(Body::Fixtures), std::size_t(0));
#endif
}

TEST(Body, ByteSize)
{
    // architecture dependent...
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN64)
#if !defined(NDEBUG)
            EXPECT_EQ(sizeof(Body), std::size_t(216));
#else
            EXPECT_EQ(sizeof(Body), std::size_t(176));
#endif
#elif defined(_WIN32)
#if !defined(NDEBUG)
            // Win32 debug
            EXPECT_EQ(sizeof(Body), std::size_t(192));
#else
            // Win32 release
            EXPECT_EQ(sizeof(Body), std::size_t(136));
#endif
#else
            EXPECT_EQ(sizeof(Body), std::size_t(128));
#endif
            break;
        case  8:
            EXPECT_EQ(sizeof(Body), std::size_t(272));
            break;
        case 16:
            EXPECT_EQ(sizeof(Body), std::size_t(480));
            break;
        default: FAIL(); break;
    }
}
