//
//  TimeOfImpact.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/10/16.
//
//

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
	TOIOutput foo{state, 3, time};
	EXPECT_EQ(foo.get_state(), state);
	EXPECT_EQ(foo.get_iterations(), 3);
	EXPECT_EQ(foo.get_t(), time);
}

TEST(TimeOfImpact, Overlapped)
{
	const auto radius = float_t(1);
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{{0, 0}, 0}};
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{{0, 0}, 0}};
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, 1);
	EXPECT_EQ(output.get_state(), TOIOutput::e_overlapped);
	EXPECT_EQ(output.get_t(), float_t(0));
	EXPECT_EQ(output.get_iterations(), 0);
}

TEST(TimeOfImpact, Touching)
{
	const auto radius = float_t(1.1);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{2, 0}, 0}};

	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, 1);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_EQ(output.get_t(), float_t(0));
	EXPECT_EQ(output.get_iterations(), 0);
}

TEST(TimeOfImpact, Separated)
{
	const auto radius = float_t(1);
	
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, 0}, 0}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{4, 0}, 0}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, 1);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_EQ(output.get_t(), float_t(1));
	EXPECT_EQ(output.get_iterations(), 0);
}

TEST(TimeOfImpact, CollideHorizontally)
{
	const auto radius = float_t(1);
	const auto x = float_t(3);
	
	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{-x, 0}, 0}, Position{Vec2{0, 0}, 0}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{+x, 0}, 0}, Position{Vec2{0, 0}, 0}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, 1);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_FLOAT_EQ(output.get_t(), float_t(0.66671669));
	EXPECT_EQ(output.get_iterations(), 1);
}

TEST(TimeOfImpact, CollideVertically)
{
	const auto radius = float_t(1);
	const auto y = float_t(20);

	const auto proxyA = DistanceProxy{radius, Vec2_zero};
	const auto sweepA = Sweep{Position{Vec2{0, -y}, 0}, Position{Vec2{0, +y}, 0}};
	
	const auto proxyB = DistanceProxy{radius, Vec2_zero};
	const auto sweepB = Sweep{Position{Vec2{0, +y}, 0}, Position{Vec2{0, -y}, 0}};
	
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB, 1);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_touching);
	EXPECT_FLOAT_EQ(output.get_t(), float_t(0.47500378));
	EXPECT_EQ(output.get_iterations(), 1);
}
