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
    EXPECT_EQ(sizeof(FixtureConf), std::size_t(4));
}

TEST(FixtureConf, DefaultConstructor)
{
    const auto fixture = FixtureConf{};
    EXPECT_EQ(GetBody(fixture), InvalidBodyID);
    EXPECT_EQ(GetShape(fixture), InvalidShapeID);
}

TEST(FixtureConf, InitializingConstructor)
{
    const auto body = BodyID(23u);
    const auto shape = ShapeID(0);
    const auto fixture = FixtureConf{}.UseBody(body).UseShape(shape);
    EXPECT_EQ(GetBody(fixture), body);
    EXPECT_EQ(GetShape(fixture), shape);
}

TEST(FixtureConf, EqualsOperator)
{
    EXPECT_TRUE(FixtureConf() == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseShape(ShapeID(0)) == FixtureConf());
    EXPECT_FALSE(FixtureConf().UseBody(BodyID(1u)) == FixtureConf());
}

TEST(FixtureConf, NotEqualsOperator)
{
    EXPECT_FALSE(FixtureConf() != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseShape(ShapeID(0)) != FixtureConf());
    EXPECT_TRUE(FixtureConf().UseBody(BodyID(1u)) != FixtureConf());
}
