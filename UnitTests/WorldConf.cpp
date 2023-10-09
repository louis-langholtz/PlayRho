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

#include <playrho/d2/WorldConf.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(WorldConf, DefaultConstruction)
{
    const auto worldConf = WorldConf{};
    EXPECT_EQ(worldConf.upstream, WorldConf::DefaultUpstream);
    EXPECT_EQ(worldConf.vertexRadius, WorldConf::DefaultVertexRadius);
    EXPECT_EQ(worldConf.treeCapacity, WorldConf::DefaultTreeCapacity);
    EXPECT_EQ(worldConf.contactCapacity, WorldConf::DefaultContactCapacity);
    EXPECT_EQ(worldConf.proxyCapacity, WorldConf::DefaultProxyCapacity);
    EXPECT_EQ(worldConf.reserveBodyStack, WorldConf::DefaultReserveBodyStack);
    EXPECT_EQ(worldConf.reserveBodyConstraints, WorldConf::DefaultReserveBodyConstraints);
    EXPECT_EQ(worldConf.reserveDistanceConstraints, WorldConf::DefaultReserveDistanceConstraints);
    EXPECT_EQ(worldConf.reserveContactKeys, WorldConf::DefaultReserveContactKeys);
    EXPECT_EQ(worldConf.reserveBuffers, WorldConf::DefaultReserveBuffers);
    EXPECT_EQ(worldConf.doStats, WorldConf::DefaultDoStats);
}

TEST(WorldConf, UseUpstream)
{
    EXPECT_EQ(WorldConf().UseUpstream(nullptr).upstream, nullptr);
}

TEST(WorldConf, UseVertexRadius)
{
    EXPECT_EQ(WorldConf().UseVertexRadius({4.2_m, 6.3_m}).vertexRadius.GetMin(), 4.2_m);
    EXPECT_EQ(WorldConf().UseVertexRadius({4.2_m, 6.3_m}).vertexRadius.GetMax(), 6.3_m);
}

TEST(WorldConf, UseTreeCapacity)
{
    EXPECT_EQ(WorldConf().UseTreeCapacity(42u).treeCapacity, 42u);
}

TEST(WorldConf, UseContactCapacity)
{
    EXPECT_EQ(WorldConf().UseContactCapacity(42u).contactCapacity, 42u);
}

TEST(WorldConf, UseProxyCapacity)
{
    EXPECT_EQ(WorldConf().UseProxyCapacity(42u).proxyCapacity, 42u);
}
