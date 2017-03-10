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
#include <Box2D/Collision/BroadPhase.hpp>

using namespace box2d;

TEST(BroadPhase, ByteSizeIs72)
{
	EXPECT_EQ(sizeof(BroadPhase), size_t(72));
}

TEST(BroadPhase, DefaultConstruction)
{
	const auto defaultConf = BroadPhase::GetDefaultConf();
	
	BroadPhase foo;
	
	EXPECT_EQ(foo.GetPairCapacity(), defaultConf.pairCapacity);
	EXPECT_EQ(foo.GetMoveCapacity(), defaultConf.moveCapacity);

	EXPECT_EQ(foo.GetProxyCount(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeHeight(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeBalance(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetProxyCount(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetMoveCount(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeQuality(), RealNum(0));
}

TEST(BroadPhase, CreateAndDestroyProxy)
{
	const auto defaultConf = BroadPhase::GetDefaultConf();

	BroadPhase foo;
	
	ASSERT_EQ(foo.GetPairCapacity(), defaultConf.pairCapacity);
	ASSERT_EQ(foo.GetMoveCapacity(), defaultConf.moveCapacity);
	
	const auto aabb = AABB{Vec2{3, 1}, Vec2{-5, -2}};
	const auto userdata = nullptr;
	
	const auto pid = foo.CreateProxy(aabb, userdata);
	EXPECT_EQ(foo.GetProxyCount(), BroadPhase::size_type(1));
	EXPECT_EQ(foo.GetPairCapacity(), defaultConf.pairCapacity);
	EXPECT_EQ(foo.GetFatAABB(pid), aabb);
	EXPECT_EQ(foo.GetUserData(pid), userdata);
	EXPECT_EQ(foo.GetTreeHeight(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeBalance(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeQuality(), RealNum(1));
	
	foo.DestroyProxy(pid);
	EXPECT_EQ(foo.GetProxyCount(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetPairCapacity(), defaultConf.pairCapacity);
	EXPECT_EQ(foo.GetTreeHeight(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeBalance(), BroadPhase::size_type(0));
	EXPECT_EQ(foo.GetTreeQuality(), RealNum(0));
}
