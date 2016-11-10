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
#include <Box2D/Collision/TimeOfImpact.h>
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

TEST(TOIOutput, DefaultConstruction)
{
	TOIOutput foo;
	EXPECT_EQ(foo.get_state(), TOIOutput::e_unknown);
}

TEST(TOIOutput, InitConstruction)
{
	const auto state = TOIOutput::e_failed;
	const auto time = float_t(0.6);
	TOIOutput::Stats stats{3, 5, 11, 10, 4};
	TOIOutput foo{state, time, stats};

	EXPECT_EQ(foo.get_state(), state);
	EXPECT_EQ(foo.get_t(), time);
	
	EXPECT_EQ(foo.get_toi_iters(), 3);
	EXPECT_EQ(foo.get_sum_dist_iters(), 5);
	EXPECT_EQ(foo.get_max_dist_iters(), 11);
	EXPECT_EQ(foo.get_sum_root_iters(), 10);
	EXPECT_EQ(foo.get_max_root_iters(), 4);
}

TEST(TimeOfImpact, Overlapped)
{
	const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};

	const auto radius = float_t(1);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{{0, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{{0, 0}, 0_deg}};
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	EXPECT_EQ(output.get_state(), TOIOutput::e_overlapped);
	EXPECT_EQ(output.get_t(), float_t(0));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Touching)
{
	const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};

	const auto radius = float_t(1.1);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{2, 0}, 0_deg}};

	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_EQ(output.get_t(), float_t(0));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Separated)
{
	const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};
	const auto radius = float_t(1);
	
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{4, 0}, 0_deg}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_EQ(output.get_t(), float_t(1));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, CollideHorizontally)
{
	//const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};
	const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};

	// Set up for two bodies moving toward each other at same speeds and each colliding
	// with the other after they have moved roughly two-thirds of their sweep.
	const auto radius = float_t(1);
	const auto x = float_t(3);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	
	// Compute the time of impact information now...
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	const auto approx_time_of_collision = ((x - radius) + limits.targetDepth / 2) / x;

	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_FLOAT_EQ(output.get_t(), approx_time_of_collision); // float_t(0.66671669)
	EXPECT_EQ(output.get_toi_iters(), 2);
}

TEST(TimeOfImpact, CollideVertically)
{
	const auto limits = TOILimits{1, float_t(0.0001) * 3, float_t(0.0001) / 4};
	const auto radius = float_t(1);
	const auto y = float_t(20);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, -y}, 0_deg}, Position{Vec2{0, +y}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{0, +y}, 0_deg}, Position{Vec2{0, -y}, 0_deg}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_FLOAT_EQ(output.get_t(), float_t(0.47500378));
	EXPECT_EQ(output.get_toi_iters(), 2);
}
