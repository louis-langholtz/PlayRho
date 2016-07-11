//
//  TimeOfImpact.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 7/10/16.
//
//

#include "gtest/gtest.h"
#include <Box2D/Collision/TimeOfImpact.h>

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
	TOIOutput foo{state, time};
	EXPECT_EQ(foo.get_state(), state);
	EXPECT_EQ(foo.get_t(), time);
}

TEST(TimeOfImpact, Overlapped)
{
	const auto radius = float_t(1);
	const auto position = Vec2{0, 0};
	const auto proxyA = DistanceProxy{radius, position};
	const auto sweepA = Sweep{Position{{0, 0}, 0}};
	const auto proxyB = DistanceProxy{radius, position};
	const auto sweepB = Sweep{Position{{0, 0}, 0}};
	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB);
	EXPECT_EQ(output.get_state(), TOIOutput::e_overlapped);
	EXPECT_EQ(output.get_t(), float_t(0));
}

TEST(TimeOfImpact, Touching)
{
	const auto radius = float_t(1);

	const auto posA = Vec2{0, 0};
	const auto proxyA = DistanceProxy{radius, posA};
	const auto sweepA = Sweep{Position{{0, 0}, 0}};
	
	const auto posB = Vec2{2, 0};
	const auto proxyB = DistanceProxy{radius, posB};
	const auto sweepB = Sweep{Position{{0, 0}, 0}};

	const auto output = TimeOfImpact(proxyA, sweepA, proxyB, sweepB);
	
	EXPECT_EQ(output.get_state(), TOIOutput::e_separated);
	EXPECT_EQ(output.get_t(), float_t(1));
}