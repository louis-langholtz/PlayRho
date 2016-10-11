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
#include <Box2D/Dynamics/Contacts/VelocityConstraint.hpp>

using namespace box2d;

TEST(VelocityConstraint, ByteSizeIs176or160)
{
#if defined(BOX2D_CACHE_VC_POINT_MASSES)
	EXPECT_EQ(sizeof(VelocityConstraint), size_t(176));
#else
	EXPECT_EQ(sizeof(VelocityConstraint), size_t(160));
#endif
}

TEST(VelocityConstraint, DefaultInit)
{
	VelocityConstraint vc;
	EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{0});
	EXPECT_FALSE(IsValid(vc.GetK()));
	EXPECT_FALSE(IsValid(vc.GetNormalMass()));
	EXPECT_FALSE(IsValid(vc.normal));
	EXPECT_FALSE(IsValid(vc.GetFriction()));
	EXPECT_FALSE(IsValid(vc.GetRestitution()));
	EXPECT_FALSE(IsValid(vc.GetTangentSpeed()));
	EXPECT_FALSE(IsValid(vc.GetContactIndex()));
}

TEST(VelocityConstraint, InitializingConstructor)
{
	const auto contact_index = VelocityConstraint::index_type{3};
	const auto friction = float_t(0.432);
	const auto restitution = float_t(0.989);
	const auto tangent_speed = float_t(1.876);
	
	const VelocityConstraint vc(contact_index, friction, restitution, tangent_speed);
	EXPECT_EQ(vc.GetContactIndex(), contact_index);
	EXPECT_EQ(vc.GetFriction(), friction);
	EXPECT_EQ(vc.GetRestitution(), restitution);
	EXPECT_EQ(vc.GetTangentSpeed(), tangent_speed);
}
