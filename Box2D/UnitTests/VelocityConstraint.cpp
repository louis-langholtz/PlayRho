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
#include <Box2D/Collision/WorldManifold.hpp>

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
	EXPECT_FALSE(IsValid(vc.GetK()));
	EXPECT_FALSE(IsValid(vc.GetNormalMass()));
	EXPECT_FALSE(IsValid(vc.GetNormal()));
	EXPECT_FALSE(IsValid(vc.GetFriction()));
	EXPECT_FALSE(IsValid(vc.GetRestitution()));
	EXPECT_FALSE(IsValid(vc.GetTangentSpeed()));
	EXPECT_FALSE(IsValid(vc.GetContactIndex()));
	
	EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type{0});

	EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(0)));
	EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(0)));
	EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(0)));
	EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(0)));
	EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(0)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosA(0)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosB(0)));

	EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(1)));	
	EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosA(1)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosB(1)));
}

TEST(VelocityConstraint, InitializingConstructor)
{
	const auto contact_index = VelocityConstraint::index_type{3};
	const auto friction = float_t(0.432);
	const auto restitution = float_t(0.989);
	const auto tangent_speed = float_t(1.876);
	
	const auto bodyA = VelocityConstraint::BodyData{};
	const auto bodyB = VelocityConstraint::BodyData{};

	const VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB};

	EXPECT_EQ(vc.GetContactIndex(), contact_index);
	EXPECT_EQ(vc.GetFriction(), friction);
	EXPECT_EQ(vc.GetRestitution(), restitution);
	EXPECT_EQ(vc.GetTangentSpeed(), tangent_speed);
	EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
}

TEST(VelocityConstraint, AddPoint)
{
	const auto contact_index = VelocityConstraint::index_type{3};
	const auto friction = float_t(0.432);
	const auto restitution = float_t(0.989);
	const auto tangent_speed = float_t(1.876);

	const auto bodyA = VelocityConstraint::BodyData{};
	const auto bodyB = VelocityConstraint::BodyData{};

	VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB};

	ASSERT_EQ(vc.GetContactIndex(), contact_index);
	ASSERT_EQ(vc.GetFriction(), friction);
	ASSERT_EQ(vc.GetRestitution(), restitution);
	ASSERT_EQ(vc.GetTangentSpeed(), tangent_speed);
	ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
	
	const auto ni = float_t(1.2);
	const auto ti = float_t(0.3);
	
	vc.AddPoint(ni, ti);
	EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(1));
	
	vc.AddPoint(ni + 2, ti + 2);
	EXPECT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(2));

	EXPECT_EQ(GetNormalImpulseAtPoint(vc, 0), ni);
	EXPECT_EQ(GetTangentImpulseAtPoint(vc, 0), ti);
	EXPECT_EQ(GetNormalImpulseAtPoint(vc, 1), ni + 2);
	EXPECT_EQ(GetTangentImpulseAtPoint(vc, 1), ti + 2);
}

TEST(VelocityConstraint, Update)
{
	const auto contact_index = VelocityConstraint::index_type{3};
	const auto friction = float_t(0.432);
	const auto restitution = float_t(0.989);
	const auto tangent_speed = float_t(1.876);
	
	const auto invMass = float_t(0.1);
	const auto invI = float_t(0.02);
	const auto bodyA = VelocityConstraint::BodyData{0, invMass, invI};
	const auto bodyB = VelocityConstraint::BodyData{1, invMass, invI};

	VelocityConstraint vc{contact_index, friction, restitution, tangent_speed, bodyA, bodyB};
	ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(0));
	ASSERT_EQ(vc.GetContactIndex(), contact_index);
	ASSERT_EQ(vc.GetFriction(), friction);
	ASSERT_EQ(vc.GetRestitution(), restitution);
	ASSERT_EQ(vc.GetTangentSpeed(), tangent_speed);
	
	ASSERT_FALSE(IsValid(GetNormalMassAtPoint(vc, 0)));
	ASSERT_FALSE(IsValid(GetNormalMassAtPoint(vc, 1)));

	const auto ni = float_t(1.2);
	const auto ti = float_t(0.3);
	vc.AddPoint(ni, ti);
	ASSERT_EQ(vc.GetPointCount(), VelocityConstraint::size_type(1));
	ASSERT_EQ(vc.GetNormalImpulseAtPoint(0), ni);
	ASSERT_EQ(vc.GetTangentImpulseAtPoint(0), ti);

	const auto normal = Vec2(1, 0);
	const auto ps = WorldManifold::PointSeparation{};
	const auto worldManifold = WorldManifold{normal, ps};

	const auto posA = Vec2{1, 2};
	const auto posB = Vec2{3, 4};
	const auto velocities = {Velocity{Vec2{1, 0}, 0_deg}, Velocity{Vec2{-1, 0}, 0_deg}};

	vc.Update(worldManifold, posA, posB, velocities.begin(), false);

	EXPECT_FLOAT_EQ(vc.GetNormal().x, normal.x);
	EXPECT_FLOAT_EQ(vc.GetNormal().y, normal.y);

	EXPECT_FLOAT_EQ(vc.GetNormalImpulseAtPoint(0), ni);
	EXPECT_FLOAT_EQ(vc.GetTangentImpulseAtPoint(0), ti);
	EXPECT_FLOAT_EQ(vc.GetNormalMassAtPoint(0), float_t(1.6666666));
	EXPECT_FLOAT_EQ(vc.GetTangentMassAtPoint(0), float_t(2.5000002));
	EXPECT_FLOAT_EQ(vc.GetVelocityBiasAtPoint(0), float_t(1.978));
	EXPECT_FLOAT_EQ(vc.GetPointRelPosA(0).x, float_t(-1));
	EXPECT_FLOAT_EQ(vc.GetPointRelPosA(0).y, float_t(-2));
	EXPECT_FLOAT_EQ(vc.GetPointRelPosB(0).x, float_t(-3));
	EXPECT_FLOAT_EQ(vc.GetPointRelPosB(0).y, float_t(-4));

	EXPECT_FALSE(IsValid(vc.GetNormalImpulseAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetTangentImpulseAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetNormalMassAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetTangentMassAtPoint(1)));	
	EXPECT_FALSE(IsValid(vc.GetVelocityBiasAtPoint(1)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosA(1)));
	EXPECT_FALSE(IsValid(vc.GetPointRelPosB(1)));
}

