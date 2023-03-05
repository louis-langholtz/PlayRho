/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Collision/SimplexEdge.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(SimplexEdge, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(20)); break;
        case  8: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(SimplexEdge), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(SimplexEdge, InitializingConstructor)
{
    const auto iA = VertexCounter{1};
    const auto iB = VertexCounter{2};
    const auto pA = Length2{2.2_m, -3.1_m};
    const auto pB = Length2{-9.2_m, 0.003_m};

    const auto sv = SimplexEdge(pA, iA, pB, iB);
    
    EXPECT_EQ(sv.GetPointA(), pA);
    EXPECT_EQ(sv.GetPointB(), pB);
    EXPECT_EQ(sv.GetIndexA(), iA);
    EXPECT_EQ(sv.GetIndexB(), iB);
    EXPECT_EQ(GetPointDelta(sv), pB - pA);
}
