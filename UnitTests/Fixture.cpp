/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/Fixture.hpp>

#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Fixture, ByteSize)
{
#if defined(_WIN32) && !defined(_WIN64)
    EXPECT_EQ(sizeof(Fixture::Proxies), std::size_t(12));
#else
    EXPECT_EQ(sizeof(Fixture::Proxies), std::size_t(24));
#endif
    switch (sizeof(Real))
    {
        case  4:
#if defined(_WIN32) && !defined(_WIN64)
            EXPECT_EQ(sizeof(Fixture), std::size_t(32));
#else
            EXPECT_EQ(sizeof(Fixture), std::size_t(56));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(Fixture), std::size_t(56)); break;
        case 16: EXPECT_EQ(sizeof(Fixture), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(Fixture, DefaultConstructor)
{
    const auto fixture = Fixture{};
    EXPECT_EQ(fixture.GetBody(), InvalidBodyID);
    EXPECT_EQ(fixture.GetFilterData().categoryBits, Filter{}.categoryBits);
    EXPECT_EQ(fixture.GetFilterData().maskBits, Filter{}.maskBits);
    EXPECT_EQ(fixture.GetFilterData().groupIndex, Filter{}.groupIndex);
    EXPECT_EQ(fixture.IsSensor(), false);
    EXPECT_TRUE(fixture.GetProxies().empty());
    EXPECT_EQ(fixture.GetFriction(), Real(0));
    EXPECT_EQ(fixture.GetRestitution(), Real(0));
    EXPECT_EQ(fixture.GetDensity(), 0_kgpm2);
}

TEST(Fixture, InitializingConstructor)
{
    const auto body = BodyID(23u);
    const auto vertexRadius = 0.26_m;
    const auto friction = Real(2.5);
    const auto restitution = Real(0.8);
    const auto density = 2.3_kgpm2;
    const auto conf = EdgeShapeConf{}.UseVertexRadius(vertexRadius).UseFriction(friction)
    .UseRestitution(restitution).UseDensity(density);
    const auto filter = Filter{};
    const auto isSensor = true;
    const auto def = FixtureConf{}.UseIsSensor(isSensor).UseFilter(filter);
    const auto fixture = Fixture{body, Shape(conf), def};
    EXPECT_EQ(fixture.GetBody(), body);
    EXPECT_EQ(fixture.GetFilterData().categoryBits, filter.categoryBits);
    EXPECT_EQ(fixture.GetFilterData().maskBits, filter.maskBits);
    EXPECT_EQ(fixture.GetFilterData().groupIndex, Filter{}.groupIndex);
    EXPECT_EQ(fixture.IsSensor(), isSensor);
    EXPECT_TRUE(fixture.GetProxies().empty());
    EXPECT_EQ(fixture.GetFriction(), friction);
    EXPECT_EQ(fixture.GetRestitution(), restitution);
    EXPECT_EQ(fixture.GetDensity(), density);
}
