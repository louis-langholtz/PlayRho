/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "gtest/gtest.h"
#include <Box2D/Collision/SimplexEdge.hpp>

using namespace box2d;

TEST(SimplexEdge, ByteSizeIs_28_56_or_112)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(28)); break;
        case  8: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(112)); break;
        default: FAIL(); break;
    }
}

TEST(SimplexEdge, InitializingConstructor)
{
    const auto iA = SimplexEdge::index_type{1};
    const auto iB = SimplexEdge::index_type{2};
    const auto pA = Length2D{Real(2.2) * Meter, Real(-3.1) * Meter};
    const auto pB = Length2D{Real(-9.2) * Meter, Real(0.003) * Meter};

    const auto sv = SimplexEdge(pA, iA, pB, iB);
    
    EXPECT_EQ(sv.GetPointA(), pA);
    EXPECT_EQ(sv.GetPointB(), pB);
    EXPECT_EQ(sv.GetIndexA(), iA);
    EXPECT_EQ(sv.GetIndexB(), iB);
    EXPECT_EQ(sv.GetPointDelta(), pB - pA);
}
