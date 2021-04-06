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

#include <PlayRho/Dynamics/FixtureConf.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(FixtureConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
#if defined(_WIN32) && !defined(_WIN64)
        EXPECT_EQ(sizeof(FixtureConf), std::size_t(20));
#else
        EXPECT_EQ(sizeof(FixtureConf), std::size_t(32));
#endif
        break;
    case 8:
        EXPECT_EQ(sizeof(FixtureConf), std::size_t(32));
        break;
    case 16:
        EXPECT_EQ(sizeof(FixtureConf), std::size_t(32));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(FixtureConf, DefaultConstructor)
{
    const auto fixture = FixtureConf{};
    EXPECT_EQ(GetBody(fixture), InvalidBodyID);
    EXPECT_EQ(GetFilterData(fixture).categoryBits, Filter{}.categoryBits);
    EXPECT_EQ(GetFilterData(fixture).maskBits, Filter{}.maskBits);
    EXPECT_EQ(GetFilterData(fixture).groupIndex, Filter{}.groupIndex);
    EXPECT_EQ(IsSensor(fixture), false);
    EXPECT_EQ(GetFriction(fixture), Real(0));
    EXPECT_EQ(GetRestitution(fixture), Real(0));
    EXPECT_EQ(GetDensity(fixture), 0_kgpm2);
}

TEST(FixtureConf, InitializingConstructor)
{
    const auto body = BodyID(23u);
    const auto vertexRadius = 0.26_m;
    const auto friction = Real(2.5);
    const auto restitution = Real(0.8);
    const auto density = 2.3_kgpm2;
    const auto conf = EdgeShapeConf{}
                          .UseVertexRadius(vertexRadius)
                          .UseFriction(friction)
                          .UseRestitution(restitution)
                          .UseDensity(density);
    const auto filter = Filter{};
    const auto isSensor = true;
    const auto fixture =
        FixtureConf{}.UseIsSensor(isSensor).UseFilter(filter).UseBody(body).UseShape(conf);
    EXPECT_EQ(GetBody(fixture), body);
    EXPECT_EQ(GetFilterData(fixture).categoryBits, filter.categoryBits);
    EXPECT_EQ(GetFilterData(fixture).maskBits, filter.maskBits);
    EXPECT_EQ(GetFilterData(fixture).groupIndex, Filter{}.groupIndex);
    EXPECT_EQ(IsSensor(fixture), isSensor);
    EXPECT_EQ(GetFriction(fixture), friction);
    EXPECT_EQ(GetRestitution(fixture), restitution);
    EXPECT_EQ(GetDensity(fixture), density);
}

TEST(FixtureConf, EqualsOperator)
{
    constexpr auto filter = Filter{0x2u, 0x8, 0x1};
    EXPECT_TRUE(FixtureConf() == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseShape(DiskShapeConf{1_m}) == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseFilter(filter) == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseBody(BodyID(1u)) == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseIsSensor(true) == FixtureConf());
}

TEST(FixtureConf, NotEqualsOperator)
{
    constexpr auto filter = Filter{0x2u, 0x8, 0x1};
    EXPECT_FALSE(FixtureConf() != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseShape(DiskShapeConf{1_m}) != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseFilter(filter) != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseBody(BodyID(1u)) != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseIsSensor(true) != FixtureConf());
}
