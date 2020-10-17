/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldFixture.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/ChainShapeConf.hpp>

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
            EXPECT_EQ(sizeof(Fixture), std::size_t(36));
#else
            EXPECT_EQ(sizeof(Fixture), std::size_t(64));
#endif
            break;
        case  8: EXPECT_EQ(sizeof(Fixture), std::size_t(64)); break;
        case 16: EXPECT_EQ(sizeof(Fixture), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(Fixture, CreateMatchesConf)
{
    const auto density = 2_kgpm2;
    int variable;
    const auto userData = &variable;
    const auto friction = Real(0.5);
    const auto restitution = Real(0.4);
    const auto isSensor = true;
    const auto conf = DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density);
    const auto shapeA = Shape(conf);

    auto def = FixtureConf{};
    def.userData = userData;
    def.isSensor = isSensor;

    World world;
    const auto body = world.CreateBody();
    const auto fixture = world.CreateFixture(body, shapeA, def);

    EXPECT_EQ(GetBody(world, fixture), body);
    EXPECT_EQ(GetShape(world, fixture), shapeA);
    EXPECT_EQ(GetDensity(world, fixture), density);
    EXPECT_EQ(GetFriction(world, fixture), friction);
    EXPECT_EQ(GetRestitution(world, fixture), restitution);
    EXPECT_EQ(GetUserData(world, fixture), userData);
    EXPECT_EQ(IsSensor(world, fixture), isSensor);
    EXPECT_EQ(GetProxyCount(world, fixture), ChildCounter{0});
}

TEST(Fixture, SetSensor)
{
    const auto shapeA = Shape{DiskShapeConf{}};
    const auto bodyCtrPos = Length2(1_m, 2_m);
    
    World world;
    const auto body = world.CreateBody(BodyConf{}.UseLocation(bodyCtrPos));
    const auto fixture = world.CreateFixture(body, shapeA);
    SetSensor(world, fixture, true);
    EXPECT_TRUE(IsSensor(world, fixture));
    SetSensor(world, fixture, true);
    EXPECT_TRUE(IsSensor(world, fixture));
    SetSensor(world, fixture, false);
    EXPECT_FALSE(IsSensor(world, fixture));
}

TEST(Fixture, TestPointFreeFunction)
{
    const auto shapeA = Shape{DiskShapeConf{}};
    const auto bodyCtrPos = Length2(1_m, 2_m);

    World world;
    const auto body = world.CreateBody(BodyConf{}.UseLocation(bodyCtrPos));
    const auto fixture = world.CreateFixture(body, shapeA);
    EXPECT_TRUE(TestPoint(world, fixture, bodyCtrPos));
    EXPECT_FALSE(TestPoint(world, fixture, Length2{}));
}

TEST(Fixture, SetAwakeFreeFunction)
{
    const auto shapeA = Shape{DiskShapeConf{}};

    World world;
    const auto body = world.CreateBody(BodyConf{}.UseType(BodyType::Dynamic));
    UnsetAwake(world, body);
    ASSERT_FALSE(IsAwake(world, body));
    const auto fixture = world.CreateFixture(body, shapeA);
    SetAwake(world, GetBody(world, fixture));
    EXPECT_TRUE(IsAwake(world, body));
}

TEST(Fixture, Proxies)
{
    const auto density = 2_kgpm2;
    auto variable = 0;
    const auto userData = &variable;
    const auto friction = Real(0.5);
    const auto restitution = Real(0.4);
    const auto isSensor = true;
    
    auto def = FixtureConf{};
    def.userData = userData;
    def.isSensor = isSensor;
    
    {
        const auto shape = Shape{
            DiskShapeConf{}.UseFriction(friction).UseRestitution(restitution).UseDensity(density)
        };

        auto world = World{};
        const auto body = world.CreateBody();
        const auto fixture = world.CreateFixture(body, shape, def);

        ASSERT_EQ(GetBody(world, fixture), body);
        ASSERT_EQ(GetShape(world, fixture), shape);
        ASSERT_EQ(GetDensity(world, fixture), density);
        ASSERT_EQ(GetFriction(world, fixture), friction);
        ASSERT_EQ(GetUserData(world, fixture), userData);
        ASSERT_EQ(GetRestitution(world, fixture), restitution);
        ASSERT_EQ(IsSensor(world, fixture), isSensor);
        ASSERT_EQ(GetProxyCount(world, fixture), ChildCounter{0});

        const auto stepConf = StepConf{};
        world.Step(stepConf);
        EXPECT_EQ(GetProxyCount(world, fixture), ChildCounter{1});
        EXPECT_EQ(GetProxy(world, fixture, 0), FixtureProxy{0});
    }
    
    {
        const auto shape = Shape{
            ChainShapeConf{}.Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
        };
        
        auto world = World{};
        const auto body = world.CreateBody();
        const auto fixture = world.CreateFixture(body, shape, def);

        ASSERT_EQ(GetBody(world, fixture), body);
        ASSERT_EQ(GetShape(world, fixture), shape);
        ASSERT_EQ(IsSensor(world, fixture), isSensor);
        ASSERT_EQ(GetProxyCount(world, fixture), ChildCounter{0});
        
        const auto stepConf = StepConf{};
        world.Step(stepConf);
        EXPECT_EQ(GetProxyCount(world, fixture), ChildCounter{2});
        EXPECT_EQ(GetProxy(world, fixture, 0), FixtureProxy{0});
        EXPECT_EQ(GetProxy(world, fixture, 1), FixtureProxy{1});
    }
    
    {
        const auto shape = Shape{
            ChainShapeConf{}.Add(Length2{-2_m, -3_m}).Add(Length2{-2_m, 0_m}).Add(Length2{0_m, 0_m})
            .Add(Length2{0_m, +2_m}).Add(Length2{2_m, 2_m})
        };

        auto world = World{};
        const auto body = world.CreateBody();
        const auto fixture = world.CreateFixture(body, shape, def);

        ASSERT_EQ(GetBody(world, fixture), body);
        ASSERT_EQ(GetShape(world, fixture), shape);
        ASSERT_EQ(IsSensor(world, fixture), isSensor);
        ASSERT_EQ(GetProxyCount(world, fixture), ChildCounter{0});
        
        const auto stepConf = StepConf{};
        world.Step(stepConf);
        EXPECT_EQ(GetProxyCount(world, fixture), ChildCounter{4});
        EXPECT_EQ(GetProxy(world, fixture, 0), FixtureProxy{0});
        EXPECT_EQ(GetProxy(world, fixture, 1), FixtureProxy{1});
        EXPECT_EQ(GetProxy(world, fixture, 2), FixtureProxy{3});
        EXPECT_EQ(GetProxy(world, fixture, 3), FixtureProxy{5});
    }
}
