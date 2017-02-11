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
#include <Box2D/Collision/TimeOfImpact.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

TEST(TOIConf, DefaultConstruction)
{
	EXPECT_EQ(ToiConf{}.tMax, RealNum(1));
	EXPECT_EQ(ToiConf{}.maxRootIters, MaxTOIRootIterCount);
	EXPECT_EQ(ToiConf{}.maxToiIters, MaxTOIIterations);
	EXPECT_EQ(ToiConf{}.targetDepth, LinearSlop * 3);
	EXPECT_EQ(ToiConf{}.tolerance, LinearSlop / 4);	
}

TEST(TOIOutput, Types)
{
	EXPECT_GT(sizeof(TOIOutput::dist_sum_type), sizeof(TOIOutput::dist_iter_type));
	EXPECT_GT(sizeof(TOIOutput::root_sum_type), sizeof(TOIOutput::root_iter_type));
}

TEST(TOIOutput, DefaultConstruction)
{
	TOIOutput foo;
	EXPECT_EQ(foo.get_state(), TOIOutput::e_unknown);
}

TEST(TOIOutput, InitConstruction)
{
	const auto state = TOIOutput::e_failed;
	const auto time = RealNum(0.6);
	
	TOIOutput::Stats stats;
	stats.toi_iters = 3;
	stats.max_dist_iters = 11;
	stats.max_root_iters = 4;
	stats.sum_finder_iters = 0;
	stats.sum_dist_iters = 5;
	stats.sum_root_iters = 10;

	TOIOutput foo{state, time, stats};

	EXPECT_EQ(foo.get_state(), state);
	EXPECT_EQ(foo.get_t(), time);
	
	EXPECT_EQ(foo.get_toi_iters(), 3);
	EXPECT_EQ(foo.get_max_dist_iters(), 11);
	EXPECT_EQ(foo.get_max_root_iters(), 4);
	//EXPECT_EQ(foo.get_sum_finder_iters(), 0);
	EXPECT_EQ(foo.get_sum_dist_iters(), 5);
	EXPECT_EQ(foo.get_sum_root_iters(), 10);
}

TEST(TimeOfImpact, Overlapped)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);

	const auto radius = RealNum(1);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{{0, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{{0, 0}, 0_deg}};
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	EXPECT_EQ(output.get_state(), TOIOutput::e_overlapped);
	EXPECT_EQ(output.get_t(), RealNum(0));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Touching)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);

	const auto radius = RealNum(1.1);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{2, 0}, 0_deg}};

	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_EQ(output.get_t(), RealNum(0));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, Separated)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);
	const auto radius = RealNum(1);
	
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{4, 0}, 0_deg}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_EQ(output.get_t(), RealNum(1));
	EXPECT_EQ(output.get_toi_iters(), 1);
}

TEST(TimeOfImpact, CollideCirclesHorizontally)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);

	// Set up for two bodies moving toward each other at same speeds and each colliding
	// with the other after they have moved roughly two-thirds of their sweep.
	const auto radius = RealNum(1);
	const auto x = RealNum(2);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	
	// Compute the time of impact information now...
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	const auto approx_time_of_collision = ((x - radius) + limits.targetDepth / 2) / x;

	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_TRUE(almost_equal(output.get_t(), approx_time_of_collision));
	EXPECT_EQ(output.get_toi_iters(), 2);
}

TEST(TimeOfImpact, CollideCirclesVertically)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);
	const auto radius = RealNum(1);
	const auto y = RealNum(20);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, -y}, 0_deg}, Position{Vec2{0, +y}, 0_deg}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{0, +y}, 0_deg}, Position{Vec2{0, -y}, 0_deg}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_NEAR(double(output.get_t()), 0.4750375, 0.000001);
	EXPECT_EQ(output.get_toi_iters(), 2);
}

TEST(TimeOfImpact, CirclesPassingParallelSeparatedPathsDontCollide)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);
	
	// Set up for two bodies moving toward each other at same speeds and each colliding
	// with the other after they have moved roughly two-thirds of their sweep.
	const auto radius = RealNum(1);
	const auto x = RealNum(3);
	const auto y = RealNum(1);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{-x, +y}, 0_deg}, Position{Vec2{+x, +y}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, -y}, 0_deg}, Position{Vec2{-x, -y}, 0_deg}};
	
	// Compute the time of impact information now...
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_TRUE(almost_equal(output.get_t(), RealNum(1.0)));
	EXPECT_EQ(output.get_toi_iters(), 7);
}

TEST(TimeOfImpact, RodCircleMissAt360)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);
	
	// Set up for two bodies moving toward each other at same speeds and each colliding
	// with the other after they have moved roughly two-thirds of their sweep.
	const auto radius = RealNum(1);
	const auto x = RealNum(40);
	const auto proxyA = DistanceProxy{radius, Vec2{-4, 0}, Vec2{4, 0}};
	const auto sweepA = Sweep{Position{Vec2{-x, 4}, 0_deg}, Position{Vec2{+x, 4}, 360_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{-x, 0}, 0_deg}};
	
	// Compute the time of impact information now...
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_TRUE(almost_equal(output.get_t(), RealNum(1.0)));
	EXPECT_EQ(output.get_toi_iters(), 4);
}

TEST(TimeOfImpact, RodCircleHitAt180)
{
	const auto limits = ToiConf{}.UseTimeMax(1).UseTargetDepth(0.001f * 3).UseTolerance(0.001f / 4);
	
	// Set up for two bodies moving toward each other at same speeds and each colliding
	// with the other after they have moved roughly two-thirds of their sweep.
	const auto radius = RealNum(1);
	const auto x = RealNum(40);
	const auto proxyA = DistanceProxy{radius, Vec2{-4, 0}, Vec2{4, 0}};
	const auto sweepA = Sweep{Position{Vec2{-x, 4}, 0_deg}, Position{Vec2{+x, 4}, 180_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{-x, 0}, 0_deg}};
	
	// Compute the time of impact information now...
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, limits);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_NEAR(double(output.get_t()), 0.4884203672409058, 0.000001);
	EXPECT_EQ(output.get_toi_iters(), 3);
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_1)
{
	const auto radius = RealNum(1);
	const auto x = RealNum(200);
	const auto proxyA = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0_deg}, Position{Vec2{+x, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{-x, 0}, 0_deg}};
	
	const auto conf = ToiConf{}
		.UseMaxToiIters(200)
		.UseMaxRootIters(200)
		.UseTargetDepth(0.001f * 3)
		.UseTolerance(0.001f / 4);
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_NEAR(double(output.get_t()), 0.4975037276744843, 0.000001);
	EXPECT_EQ(output.get_toi_iters(), 2);
	EXPECT_EQ(output.get_max_dist_iters(), 1);
	EXPECT_EQ(output.get_max_root_iters(), 2);
	EXPECT_EQ(output.get_sum_dist_iters(), 2);
	EXPECT_EQ(output.get_sum_root_iters(), 2);
}

TEST(TimeOfImpact, SucceedsWithClosingSpeedOf800_2)
{
	const auto radius = RealNum(1);
	const auto x = RealNum(400);
	const auto proxyA = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{0, 0}, 0_deg}};
	
	const auto conf = ToiConf{}
		.UseMaxToiIters(200)
		.UseMaxRootIters(200)
		.UseTargetDepth(0.001f * 3)
		.UseTolerance(0.001f / 4);
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_NEAR(double(output.get_t()), 0.9975037574768066, 0.000001);
	EXPECT_EQ(output.get_toi_iters(), 2);
	EXPECT_EQ(output.get_max_dist_iters(), 1);
	EXPECT_EQ(output.get_max_root_iters(), 2);
	EXPECT_EQ(output.get_sum_dist_iters(), 2);
	EXPECT_EQ(output.get_sum_root_iters(), 2);

#if 0
	ASSERT_EQ(output.get_state(), TOIOutput::e_touching);

	auto touching = true;
	auto iterations = 0u;
	for (auto t = output.get_t(); t > 0; t = std::nextafter(t, 0.0f))
	{
		const auto conf2 = ToiConf{}.UseMaxToiIters(200).UseMaxRootIters(200).UseTimeMax(t);
		const auto output2 = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf2);
		
		EXPECT_LE(output2.get_t(), t);
		if (touching)
		{
			if (output2.get_state() != TOIOutput::e_touching)
			{
				std::cout << "lost touch after " << iterations << " iterations at t=" << t << std::endl;
				touching = false;
			}
		}
		else // !touching
		{
			if (output2.get_state() == TOIOutput::e_touching)
			{
				std::cout << "found additional root at t=" << t << std::endl;
				touching = true;
			}
		}
		++iterations;
	}
#endif
}

TEST(TimeOfImpact, WithClosingSpeedOf1600)
{
	const auto radius = RealNum(1);
	const auto x = RealNum(400);
	const auto proxyA = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0_deg}, Position{Vec2{+x, 0}, 0_deg}};
	const auto proxyB = DistanceProxy{radius, Vec2{0, 0}};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0_deg}, Position{Vec2{-x, 0}, 0_deg}};
	
	const auto conf = ToiConf{}
		.UseMaxToiIters(200)
		.UseMaxRootIters(200)
		.UseTargetDepth(0.001f * 3)
		.UseTolerance(0.001f / 4);
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, conf);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_NEAR(double(output.get_t()), 0.4987518787384033, 0.000001);
	//std::cout << std::setprecision(std::numeric_limits<double>::digits10 + 1) << output.get_t() << std::defaultfloat << std::endl;
	EXPECT_EQ(output.get_toi_iters(), 2);
	EXPECT_EQ(output.get_max_dist_iters(), 1);
	EXPECT_EQ(output.get_max_root_iters(), 2);
	EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
	EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}

TEST(TimeOfImpact, ForNonCollidingShapesFailsIn27)
{
	// The data for shapes and sweeps comes from Box2D/Testbed/Tests/TimeOfImpact.hpp

	auto shapeA = PolygonShape{RealNum{0.0001f * 2}};
	shapeA.SetAsBox(25.0f, 5.0f);
	auto shapeB = PolygonShape{RealNum{0.0001f * 2}};
	shapeB.SetAsBox(2.5f, 2.5f);

	const auto dpA = GetDistanceProxy(shapeA, 0);
	const auto dpB = GetDistanceProxy(shapeB, 0);

	const auto sweepA = Sweep{
		Position{Vec2{-11, 10}, 2.95000005_rad},
		Position{Vec2{-11, 10}, 2.95000005_rad}
	};
	const auto sweepB = Sweep{
		Position{Vec2{18.4742737f, 19.7474861f}, 513.36676_rad},
		Position{Vec2{19.5954781f, 18.9165268f}, 513.627808_rad}
	};
	
	const auto conf = ToiConf{}
		.UseMaxToiIters(20)
		.UseMaxRootIters(32)
		.UseTimeMax(1)
		.UseTargetDepth(3.0f / 10000)
		.UseTolerance(1.0f / 40000);
	const auto output = TimeOfImpact(dpA, sweepA, dpB, sweepB, conf);
	
	EXPECT_TRUE(output.get_state() == TOIOutput::e_failed || output.get_state() == TOIOutput::e_separated);
	switch (output.get_state())
	{
		case TOIOutput::e_failed:
			EXPECT_NEAR(double(output.get_t()), 0.863826394, 0.000001);
			EXPECT_EQ(output.get_toi_iters(), 1);
			EXPECT_EQ(output.get_max_dist_iters(), 4);
			EXPECT_EQ(output.get_max_root_iters(), 27);
			break;
		case TOIOutput::e_separated:
			EXPECT_EQ(output.get_t(), RealNum(1));
			EXPECT_EQ(output.get_toi_iters(), 2);
			EXPECT_GE(output.get_max_dist_iters(), 3);
			EXPECT_LE(output.get_max_dist_iters(), 4);
			EXPECT_EQ(output.get_max_root_iters(), 6);
			break;
		default:
			break;
	}
	EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
	EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}

TEST(TimeOfImpact, ToleranceReachedWithT1Of1)
{
	// This setup causes the TimeOfImpact function to get into the state where
	// separation has reached tolerance but t2 already equals t1.

	const auto sweepA = Sweep{
		Position{Vec2{0.0f, -0.5f}, 0_rad},
		Position{Vec2{0.0f, -0.5f}, 0_rad}
	};
	const auto sweepB = Sweep{
		Position{Vec2{14.3689661f, 0.500306308f}, 0.0000139930862_rad},
		Position{Vec2{14.3689451f, 0.500254989f}, 0.000260060915_rad}
	};
	
	const auto dpA = DistanceProxy{
		0.000199999995f,
		Span<const Vec2>{
			Vec2{14.5f, -0.5f},
			Vec2{14.5f, +0.5f},
			Vec2{13.5f, +0.5f},
			Vec2{13.5f, -0.5f}
		}
	};
	
	auto shapeB = PolygonShape{RealNum{0.0001f * 2}};
	shapeB.SetAsBox(0.5f, 0.5f);
	const auto dpB = GetDistanceProxy(shapeB, 0);
	
	const auto conf = ToiConf{}
		.UseMaxToiIters(200)
		.UseMaxRootIters(30)
		.UseTimeMax(1)
		.UseTargetDepth(3.0f / 10000)
		.UseTolerance(1.0f / 40000);

	const auto output = TimeOfImpact(dpA, sweepA, dpB, sweepB, conf);

	EXPECT_TRUE(output.get_state() == TOIOutput::e_separated || output.get_state() == TOIOutput::e_touching);
	EXPECT_TRUE(almost_equal(output.get_t(), RealNum{1.0f}));
	EXPECT_TRUE(output.get_toi_iters() == 1 || output.get_toi_iters() == 2);
	EXPECT_EQ(output.get_max_dist_iters(), 4);
	EXPECT_EQ(output.get_max_root_iters(), 0);
	EXPECT_GE(output.get_sum_dist_iters(), output.get_max_dist_iters());
	EXPECT_GE(output.get_sum_root_iters(), output.get_max_root_iters());
}
