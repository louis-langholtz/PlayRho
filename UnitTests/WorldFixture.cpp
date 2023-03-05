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

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldImpl.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldMisc.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldFixture, CreateDestroy)
{
    auto world = World{};
    EXPECT_EQ(GetFixtureRange(world), 0u);
    EXPECT_THROW(CreateFixture(world, BodyID(0u), Shape{DiskShapeConf{}}), std::out_of_range);
    EXPECT_THROW(Destroy(world, FixtureID(0u)), std::out_of_range);
    const auto body = CreateBody(world);
    auto fixture = FixtureID(0u);
    EXPECT_NO_THROW(fixture = CreateFixture(world, body, Shape{DiskShapeConf{}}));
    const auto fixtures = std::vector<FixtureID>{};
    using ValueType = std::remove_cv_t<std::remove_reference_t<decltype(GetFixtures(world, body))>>;
    auto fixturesRange = ValueType();
    EXPECT_NO_THROW(fixturesRange = GetFixtures(world, body));
    EXPECT_EQ(GetFixtureRange(world), 1u);
    ASSERT_EQ(size(fixturesRange), 1u);
    EXPECT_EQ(*begin(fixturesRange), fixture);
    EXPECT_NO_THROW(Destroy(world, fixture));
    EXPECT_NO_THROW(fixturesRange = GetFixtures(world, body));
    EXPECT_EQ(size(fixturesRange), 0u);
}

TEST(WorldFixture, SetFilterData)
{
    auto world = World{};
    const auto body = CreateBody(world);
    const auto fixture = CreateFixture(world, body, Shape{DiskShapeConf{}});
    const auto origFilter = Filter{1u, 2u, 3u};
    auto copyFilter = Filter{};
    EXPECT_NO_THROW(SetFilterData(world, fixture, origFilter));
    EXPECT_NO_THROW(copyFilter = GetFilterData(world, fixture));
    EXPECT_EQ(origFilter.categoryBits, copyFilter.categoryBits);
    EXPECT_EQ(origFilter.maskBits, copyFilter.maskBits);
    EXPECT_EQ(origFilter.groupIndex, copyFilter.groupIndex);
}

TEST(WorldFixture, CreateMatchesConf)
{
    const auto density = 2_kgpm2;
    const auto friction = Real(0.5);
    const auto restitution = Real(0.4);
    const auto isSensor = true;
    const auto conf = DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density);
    const auto shapeA = Shape(conf);
    auto def = FixtureConf{};
    def.isSensor = isSensor;

    World world;
    const auto body = CreateBody(world);
    const auto fixture = CreateFixture(world, body, shapeA, def);
    EXPECT_EQ(GetBody(world, fixture), body);
    EXPECT_EQ(GetShape(world, fixture), shapeA);
    EXPECT_EQ(GetDensity(world, fixture), density);
    EXPECT_EQ(GetFriction(world, fixture), friction);
    EXPECT_EQ(GetRestitution(world, fixture), restitution);
    EXPECT_EQ(IsSensor(world, fixture), isSensor);
}

TEST(WorldFixture, SetSensor)
{
    const auto shapeA = Shape{DiskShapeConf{}};
    const auto bodyCtrPos = Length2(1_m, 2_m);
    
    World world;
    const auto body = CreateBody(world, BodyConf{}.UseLocation(bodyCtrPos));
    const auto fixture = CreateFixture(world, body, shapeA);
    SetSensor(world, fixture, true);
    EXPECT_TRUE(IsSensor(world, fixture));
    SetSensor(world, fixture, true);
    EXPECT_TRUE(IsSensor(world, fixture));
    SetSensor(world, fixture, false);
    EXPECT_FALSE(IsSensor(world, fixture));
}

TEST(WorldFixture, TestPointFreeFunction)
{
    const auto shapeA = Shape{DiskShapeConf{}};
    const auto bodyCtrPos = Length2(1_m, 2_m);

    World world;
    const auto body = CreateBody(world, BodyConf{}.UseLocation(bodyCtrPos));
    const auto fixture = CreateFixture(world, body, shapeA);
    EXPECT_TRUE(TestPoint(world, fixture, bodyCtrPos));
    EXPECT_FALSE(TestPoint(world, fixture, Length2{}));
}
