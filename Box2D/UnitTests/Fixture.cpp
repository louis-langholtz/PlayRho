/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>

using namespace box2d;

TEST(Fixture, ByteSizeIs_72_88_or_112)
{
	switch (sizeof(RealNum))
	{
		case  4: EXPECT_EQ(sizeof(Fixture), size_t(72)); break;
		case  8: EXPECT_EQ(sizeof(Fixture), size_t(88)); break;
		case 16: EXPECT_EQ(sizeof(Fixture), size_t(112)); break;
		default: FAIL(); break;
	}
}

TEST(Fixture, InitializingConstructor)
{
	const auto body = reinterpret_cast<Body*>(0x1);
	const auto shapeA = std::make_shared<CircleShape>();
	const auto density = RealNum(2);
	int variable;
	const auto userData = &variable;
	const auto friction = RealNum(0.5);
	const auto restitution = RealNum(0.4);
	const auto isSensor = true;

	auto def = FixtureDef{}.UseDensity(density);
	def.friction = friction;
	def.userData = userData;
	def.restitution = restitution;
	def.isSensor = isSensor;

	Fixture f{body, def, shapeA};
	
	EXPECT_EQ(f.GetBody(), body);
	EXPECT_EQ(f.GetShape(), &(*shapeA));

	EXPECT_EQ(f.GetDensity(), density);
	EXPECT_EQ(f.GetFriction(), friction);
	EXPECT_EQ(f.GetUserData(), userData);
	EXPECT_EQ(f.GetRestitution(), restitution);
	EXPECT_EQ(f.IsSensor(), isSensor);
}
