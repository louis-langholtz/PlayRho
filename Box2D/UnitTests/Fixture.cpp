//
//  Fixture.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 9/21/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Dynamics/Fixture.h>

using namespace box2d;

TEST(Fixture, ByteSizeIs64)
{
	EXPECT_EQ(sizeof(Fixture), size_t(64));
}

TEST(Fixture, InitializingConstructor)
{
	const auto body = reinterpret_cast<Body*>(0x1);
	const auto shapeA = reinterpret_cast<Shape*>(0x3);
	const auto shapeB = reinterpret_cast<Shape*>(0x5);
	const auto density = float_t(2);
	int variable;
	const auto userData = &variable;
	const auto friction = float_t(0.5);
	const auto restitution = float_t(0.4);
	const auto isSensor = true;

	auto def = FixtureDef{shapeB, density};
	def.friction = friction;
	def.userData = userData;
	def.restitution = restitution;
	def.isSensor = isSensor;

	Fixture f{body, def, shapeA};
	
	EXPECT_EQ(f.GetBody(), body);
	EXPECT_EQ(f.GetShape(), shapeA);

	EXPECT_EQ(f.GetDensity(), density);
	EXPECT_EQ(f.GetFriction(), friction);
	EXPECT_EQ(f.GetUserData(), userData);
	EXPECT_EQ(f.GetRestitution(), restitution);
	EXPECT_EQ(f.IsSensor(), isSensor);
}