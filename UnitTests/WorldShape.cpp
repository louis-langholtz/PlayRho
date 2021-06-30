/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>

#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Rectangle.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldShape, CreateAttachDetach)
{
    auto world = World{};
    EXPECT_THROW(Attach(world, BodyID(0u), ShapeID(0u)), std::out_of_range);
    EXPECT_THROW(Detach(world, BodyID(0u), ShapeID(0u)), std::out_of_range);
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    const auto body = CreateBody(world);
    EXPECT_NO_THROW(Attach(world, body, shapeId));
    auto shapeIds = std::vector<ShapeID>{};
    EXPECT_NO_THROW(shapeIds = GetShapes(world, body));
    ASSERT_EQ(size(shapeIds), 1u);
    EXPECT_EQ(*begin(shapeIds), shapeId);
    EXPECT_NO_THROW(Detach(world, body, shapeId));
    EXPECT_NO_THROW(shapeIds = GetShapes(world, body));
    EXPECT_EQ(size(shapeIds), 0u);
}

TEST(WorldShape, CreateMatchesConf)
{
    const auto density = 2_kgpm2;
    const auto friction = Real(0.5);
    const auto restitution = Real(0.4);
    const auto isSensor = true;
    const auto conf = DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density).UseIsSensor(isSensor);
    const auto shapeA = Shape(conf);

    World world;
    const auto shapeId = CreateShape(world, shapeA);
    EXPECT_EQ(GetShape(world, shapeId), shapeA);
    EXPECT_EQ(GetDensity(world, shapeId), density);
    EXPECT_EQ(GetFriction(world, shapeId), friction);
    EXPECT_EQ(GetRestitution(world, shapeId), restitution);
    EXPECT_EQ(IsSensor(world, shapeId), isSensor);
}

TEST(WorldShape, SetFilterData)
{
    auto world = World{};
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    const auto original = GetFilterData(world, shapeId);
    auto filter = original;
    ASSERT_EQ(filter, Filter());
    filter.categoryBits = ~filter.categoryBits;
    filter.groupIndex = ~filter.groupIndex;
    filter.maskBits = ~filter.maskBits;
    ASSERT_NE(original.categoryBits, filter.categoryBits);
    ASSERT_NE(original.groupIndex, filter.groupIndex);
    ASSERT_NE(original.maskBits, filter.maskBits);
    EXPECT_NO_THROW(SetFilterData(world, shapeId, filter));
    EXPECT_EQ(GetFilterData(world, shapeId).categoryBits, filter.categoryBits);
    EXPECT_EQ(GetFilterData(world, shapeId).groupIndex, filter.groupIndex);
    EXPECT_EQ(GetFilterData(world, shapeId).maskBits, filter.maskBits);
}

TEST(WorldShape, SetSensor)
{
    World world;
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    EXPECT_NO_THROW(SetSensor(world, shapeId, true));
    EXPECT_TRUE(IsSensor(world, shapeId));
    EXPECT_NO_THROW(SetSensor(world, shapeId, true));
    EXPECT_TRUE(IsSensor(world, shapeId));
    EXPECT_NO_THROW(SetSensor(world, shapeId, false));
    EXPECT_FALSE(IsSensor(world, shapeId));
}

TEST(WorldShape, SetFriction)
{
    World world;
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    auto value = Real(0);
    EXPECT_NO_THROW(SetFriction(world, shapeId, value));
    EXPECT_EQ(GetFriction(world, shapeId), value);
    value = Real(0.5);
    EXPECT_NO_THROW(SetFriction(world, shapeId, value));
    EXPECT_EQ(GetFriction(world, shapeId), value);
    value = Real(1.0);
    EXPECT_NO_THROW(SetFriction(world, shapeId, value));
    EXPECT_EQ(GetFriction(world, shapeId), value);
}

TEST(WorldShape, SetRestitution)
{
    World world;
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    auto value = Real(0);
    EXPECT_NO_THROW(SetRestitution(world, shapeId, value));
    EXPECT_EQ(GetRestitution(world, shapeId), value);
    value = Real(0.5);
    EXPECT_NO_THROW(SetRestitution(world, shapeId, value));
    EXPECT_EQ(GetRestitution(world, shapeId), value);
    value = Real(1.0);
    EXPECT_NO_THROW(SetRestitution(world, shapeId, value));
    EXPECT_EQ(GetRestitution(world, shapeId), value);
}

TEST(WorldShape, SetDensity)
{
    World world;
    const auto shapeId = CreateShape(world, DiskShapeConf{});
    auto value = AreaDensity(0_kgpm2);
    EXPECT_NO_THROW(SetDensity(world, shapeId, value));
    EXPECT_EQ(GetDensity(world, shapeId), value);
    value = AreaDensity{1_kgpm2};
    EXPECT_NO_THROW(SetDensity(world, shapeId, value));
    EXPECT_EQ(GetDensity(world, shapeId), value);
    value = AreaDensity(2_kgpm2);
    EXPECT_NO_THROW(SetDensity(world, shapeId, value));
    EXPECT_EQ(GetDensity(world, shapeId), value);
}

TEST(WorldShape, TransformDiskShape)
{
    World world;
    const auto location0 = Length2{1_m, 2_m};
    const auto shapeId = CreateShape(world, DiskShapeConf{}.UseLocation(location0));
    ASSERT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    auto value = GetIdentity<Mat22>() * Real(1);
    EXPECT_NO_THROW(Transform(world, shapeId, value));
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    value = GetIdentity<Mat22>() * Real(2);
    EXPECT_NO_THROW(Transform(world, shapeId, value));
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0 * Real(2));
}

TEST(WorldShape, TransformStaticRectangle)
{
    World world;
    const auto location0 = Length2{+0.5_m, -0.5_m};
    const auto shapeId = CreateShape(world, ::playrho::part::Compositor<>{});
    ASSERT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    auto value = GetIdentity<Mat22>() * Real(1);
    EXPECT_NO_THROW(Transform(world, shapeId, value));
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    value = GetIdentity<Mat22>() * Real(2);
    EXPECT_THROW(Transform(world, shapeId, value), InvalidArgument);
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
}

TEST(WorldShape, TransformDynamicRectangle)
{
    using namespace ::playrho::part;
    World world;
    const auto location0 = Length2{+0.5_m, -0.5_m};
    const auto shapeId = CreateShape(world, Compositor<GeometryIs<DynamicRectangle<>>>{});
    ASSERT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    EXPECT_NO_THROW(Transform(world, shapeId, GetIdentity<Mat22>()));
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0);
    EXPECT_NO_THROW(Transform(world, shapeId, GetIdentity<Mat22>() * Real(2)));
    EXPECT_EQ(GetChild(GetShape(world, shapeId), 0u).GetVertex(0), location0 * Real(2));
}

TEST(WorldShape, TestPointFreeFunction)
{
    const auto shapeA = Shape{DiskShapeConf{}};
    const auto bodyCtrPos = Length2(1_m, 2_m);

    World world;
    const auto shapeId = CreateShape(world, shapeA);
    const auto bodyId = CreateBody(world, BodyConf{}.UseLocation(bodyCtrPos));
    EXPECT_TRUE(TestPoint(world, bodyId, shapeId, bodyCtrPos));
    EXPECT_FALSE(TestPoint(world, bodyId, shapeId, Length2{}));
}
