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
#include <Box2D/Collision/Distance.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

TEST(Distance, MatchingCircles)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{2, 2};
	DistanceProxy dp1{1, pos1};
	DistanceProxy dp2{1, pos2};

	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a, pos1);
	EXPECT_EQ(witnessPoints.b, pos1);
	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});

	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, OpposingCircles)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{-2, -2};
	DistanceProxy dp1{2, pos1};
	DistanceProxy dp2{2, pos2};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos1.x);
	EXPECT_EQ(witnessPoints.a.y, pos1.y);

	EXPECT_EQ(witnessPoints.b.x, pos2.x);
	EXPECT_EQ(witnessPoints.b.y, pos2.y);

	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, HorTouchingCircles)
{
	Simplex::Cache cache;
	
	const auto pos1 = Vec2{-2, 2};
	const auto pos2 = Vec2{+2, 2};

	const auto output = [&]() {
		Transformation xf1 = Transform_identity;
		Transformation xf2 = Transform_identity;
		DistanceProxy dp1{2, pos1};
		DistanceProxy dp2{2, pos2};
		return Distance(dp1, xf1, dp2, xf2, cache);
	}();
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos1.x);
	EXPECT_EQ(witnessPoints.a.y, pos1.y);
	
	EXPECT_EQ(witnessPoints.b.x, pos2.x);
	EXPECT_EQ(witnessPoints.b.y, pos2.y);
	
	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, OverlappingCirclesPN)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{1, 1};
	const auto pos2 = Vec2{-1, -1};
	DistanceProxy dp1{2, pos1};
	DistanceProxy dp2{2, pos2};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos1.x);
	EXPECT_EQ(witnessPoints.a.y, pos1.y);
	
	EXPECT_EQ(witnessPoints.b.x, pos2.x);
	EXPECT_EQ(witnessPoints.b.y, pos2.y);
	
	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, OverlappingCirclesNP)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{-1, -1};
	const auto pos2 = Vec2{1, 1};
	DistanceProxy dp1{2, pos1};
	DistanceProxy dp2{2, pos2};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos1.x);
	EXPECT_EQ(witnessPoints.a.y, pos1.y);
	
	EXPECT_EQ(witnessPoints.b.x, pos2.x);
	EXPECT_EQ(witnessPoints.b.y, pos2.y);
	
	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}


TEST(Distance, SeparatedCircles)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{-2, -2};
	DistanceProxy dp1{1, pos1};
	DistanceProxy dp2{1, pos2};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos1.x);
	EXPECT_EQ(witnessPoints.a.y, pos1.y);
	
	EXPECT_EQ(witnessPoints.b.x, pos2.x);
	EXPECT_EQ(witnessPoints.b.y, pos2.y);
	
	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, EdgeCircleOverlapping)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{0, 2};
	const auto pos2 = Vec2{4, 2};
	const auto pos3 = Vec2{2, 2};
	DistanceProxy dp1{float_t(0.1), pos1, pos2};
	DistanceProxy dp2{1, pos3};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos3.x);
	EXPECT_EQ(witnessPoints.a.y, pos3.y);

	EXPECT_EQ(witnessPoints.b.x, pos3.x);
	EXPECT_EQ(witnessPoints.b.y, pos3.y);
	
	EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip0 = cache.GetIndexPair(0);
	EXPECT_EQ(ip0.a, IndexPair::size_type{0});
	EXPECT_EQ(ip0.b, IndexPair::size_type{0});

	const auto ip1 = cache.GetIndexPair(1);
	EXPECT_EQ(ip1.a, IndexPair::size_type{1});
	EXPECT_EQ(ip1.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{4});
}

TEST(Distance, EdgeCircleOverlapping2)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{-3, 2};
	const auto pos2 = Vec2{7, 2};
	const auto pos3 = Vec2{2, 2};
	DistanceProxy dp1{float_t(0.1), pos1, pos2};
	DistanceProxy dp2{1, pos3};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, pos3.x);
	EXPECT_EQ(witnessPoints.a.y, pos3.y);
	
	EXPECT_EQ(witnessPoints.b.x, pos3.x);
	EXPECT_EQ(witnessPoints.b.y, pos3.y);
	
	EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip0 = cache.GetIndexPair(0);
	EXPECT_EQ(ip0.a, IndexPair::size_type{0});
	EXPECT_EQ(ip0.b, IndexPair::size_type{0});
	
	const auto ip1 = cache.GetIndexPair(1);
	EXPECT_EQ(ip1.a, IndexPair::size_type{1});
	EXPECT_EQ(ip1.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{10});
}

TEST(Distance, EdgeCircleTouching)
{
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	const auto pos1 = Vec2{0, 3};
	const auto pos2 = Vec2{4, 3};
	const auto pos3 = Vec2{2, 1};
	DistanceProxy dp1{float_t(1), pos1, pos2};
	DistanceProxy dp2{1, pos3};
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, float_t{2});
	EXPECT_EQ(witnessPoints.a.y, float_t{3});
	
	EXPECT_EQ(witnessPoints.b.x, float_t{2});
	EXPECT_EQ(witnessPoints.b.y, float_t{1});
	
	EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip0 = cache.GetIndexPair(0);
	EXPECT_EQ(ip0.a, IndexPair::size_type{0});
	EXPECT_EQ(ip0.b, IndexPair::size_type{0});
	
	const auto ip1 = cache.GetIndexPair(1);
	EXPECT_EQ(ip1.a, IndexPair::size_type{1});
	EXPECT_EQ(ip1.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{4});
}

TEST(Distance, HorEdgeSquareTouching)
{
	const auto pos1 = Vec2{1, 1};
	const auto pos2 = Vec2{1, 3};
	const auto pos3 = Vec2{3, 3};
	const auto pos4 = Vec2{3, 1};
	const auto square = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{float_t(0.5), square};

	const auto pos5 = Vec2{-2, 0};
	const auto pos6 = Vec2{6, 0};
	DistanceProxy dp2{float_t(0.5), pos5, pos6};
	
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;

	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, float_t{1});
	EXPECT_EQ(witnessPoints.a.y, float_t{1});
	
	EXPECT_EQ(witnessPoints.b.x, float_t{1});
	EXPECT_EQ(witnessPoints.b.y, float_t{0});
	
	EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip0 = cache.GetIndexPair(0);
	EXPECT_EQ(ip0.a, IndexPair::size_type{0});
	EXPECT_EQ(ip0.b, IndexPair::size_type{0});
	
	const auto ip1 = cache.GetIndexPair(1);
	EXPECT_EQ(ip1.a, IndexPair::size_type{0});
	EXPECT_EQ(ip1.b, IndexPair::size_type{1});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{8});
}

TEST(Distance, VerEdgeSquareTouching)
{
	const auto pos1 = Vec2{1, 1};
	const auto pos2 = Vec2{1, 3};
	const auto pos3 = Vec2{3, 3};
	const auto pos4 = Vec2{3, 1};
	const auto square = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{float_t(0.5), square};
	
	const auto pos5 = Vec2{4, -2};
	const auto pos6 = Vec2{4, 6};
	DistanceProxy dp2{float_t(0.5), pos5, pos6};
	
	Simplex::Cache cache;
	Transformation xf1 = Transform_identity;
	Transformation xf2 = Transform_identity;
	
	const auto output = Distance(dp1, xf1, dp2, xf2, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(Sqrt(GetLengthSquared(witnessPoints.a - witnessPoints.b)), float_t(1));
	EXPECT_EQ(witnessPoints.a.x, float_t{3});
	EXPECT_EQ(witnessPoints.a.y, float_t{2});
	
	EXPECT_EQ(witnessPoints.b.x, float_t{4});
	EXPECT_EQ(witnessPoints.b.y, float_t{2});
	
	EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip0 = cache.GetIndexPair(0);
	EXPECT_EQ(ip0.a, IndexPair::size_type{2});
	EXPECT_EQ(ip0.b, IndexPair::size_type{0});
	
	const auto ip1 = cache.GetIndexPair(1);
	EXPECT_EQ(ip1.a, IndexPair::size_type{3});
	EXPECT_EQ(ip1.b, IndexPair::size_type{1});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{10});
}

TEST(Distance, SquareTwice)
{
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{2, 4};
	const auto pos3 = Vec2{4, 4};
	const auto pos4 = Vec2{4, 2};
	const auto square = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{float_t(0.05), square};

	Simplex::Cache cache;
	Transformation xfm = Transform_identity;
	
	const auto output = Distance(dp1, xfm, dp1, xfm, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, 2);
	EXPECT_EQ(witnessPoints.a.y, 2);

	EXPECT_EQ(witnessPoints.b.x, 2);
	EXPECT_EQ(witnessPoints.b.y, 2);

	EXPECT_EQ(decltype(output.iterations){1}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}


TEST(Distance, SquareSquareTouchingVertically)
{
	const auto pos1 = Vec2{2, 2};
	const auto pos2 = Vec2{2, 4};
	const auto pos3 = Vec2{4, 4};
	const auto pos4 = Vec2{4, 2};
	const auto square1 = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{float_t(0.05), square1};
	
	const auto pos5 = Vec2{4, 2};
	const auto pos6 = Vec2{4, 4};
	const auto pos7 = Vec2{6, 4};
	const auto pos8 = Vec2{6, 2};
	const auto square2 = {pos5, pos6, pos7, pos8};
	DistanceProxy dp2{float_t(0.05), square2};

	Simplex::Cache cache;
	Transformation xfm = Transform_identity;
	
	const auto output = Distance(dp1, xfm, dp2, xfm, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, 4);
	EXPECT_EQ(witnessPoints.a.y, 3);
	
	EXPECT_EQ(witnessPoints.b.x, 4);
	EXPECT_EQ(witnessPoints.b.y, 3);
	
	EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){2});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{3});
	EXPECT_EQ(ip.b, IndexPair::size_type{1});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{4});
}

TEST(Distance, SquareSquareDiagonally)
{
	const auto pos1 = Vec2{-3, -3};
	const auto pos2 = Vec2{-3, -1};
	const auto pos3 = Vec2{-1, -1};
	const auto pos4 = Vec2{-1, -3};
	const auto square1 = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{float_t(0.05), square1};
	
	const auto pos5 = Vec2{1, 3};
	const auto pos6 = Vec2{3, 3};
	const auto pos7 = Vec2{3, 1};
	const auto pos8 = Vec2{1, 1};
	const auto square2 = {pos5, pos6, pos7, pos8};
	DistanceProxy dp2{float_t(0.05), square2};
	
	Simplex::Cache cache;
	Transformation xfm = Transform_identity;
	
	const auto output = Distance(dp1, xfm, dp2, xfm, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, -1);
	EXPECT_EQ(witnessPoints.a.y, -1);
	
	EXPECT_EQ(witnessPoints.b.x, 1);
	EXPECT_EQ(witnessPoints.b.y, 1);
	
	EXPECT_EQ(decltype(output.iterations){2}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){1});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{2});
	EXPECT_EQ(ip.b, IndexPair::size_type{3});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{0});
}

TEST(Distance, SquareSquareOverlappingDiagnally)
{
	/*
	 *  +-----1-+
	 *  |     | |
	 * -3-2-1-+-1-2-3-
	 *  |     | |
	 *  |     1 |
	 *  |     | |
	 *  |     2 |
	 *  |     | |
	 *  +-----3-+
	 */
	// Go counter-clockwise...
	const auto pos1 = Vec2{-3, 1};
	const auto pos2 = Vec2{-3, -3};
	const auto pos3 = Vec2{1, -3};
	const auto pos4 = Vec2{1, 1};
	const auto square1 = {pos1, pos2, pos3, pos4};
	DistanceProxy dp1{0, square1};
	
	/*
	 *  +-3-----+
	 *  | |     |
	 *  | 2     |
	 *  | |     |
	 *  | 1     |
	 *  | |     |
	 * -1-+-1-2-3--
	 *  | |     |
	 *  +-1-----+
	 */
	// Go counter-clockwise...
	const auto pos5 = Vec2{3, 3};
	const auto pos6 = Vec2{-1, 3};
	const auto pos7 = Vec2{-1, -1};
	const auto pos8 = Vec2{-1, 3};
	const auto square2 = {pos5, pos6, pos7, pos8};
	DistanceProxy dp2{0, square2};
	
	Simplex::Cache cache;
	Transformation xfm = Transform_identity;
	
	const auto output = Distance(dp1, xfm, dp2, xfm, cache);
	cache = Simplex::GetCache(output.simplex.GetEdges());
	const auto witnessPoints = GetWitnessPoints(output.simplex);

	EXPECT_EQ(witnessPoints.a.x, 0);
	EXPECT_EQ(witnessPoints.a.y, 0.5);
	
	EXPECT_EQ(witnessPoints.b.x, 0);
	EXPECT_EQ(witnessPoints.b.y, 0.5);
	
	EXPECT_EQ(decltype(output.iterations){3}, output.iterations);
	
	EXPECT_EQ(cache.GetNumIndices(), decltype(cache.GetNumIndices()){3});
	
	const auto ip = cache.GetIndexPair(0);
	EXPECT_EQ(ip.a, IndexPair::size_type{0});
	EXPECT_EQ(ip.b, IndexPair::size_type{0});
	
	EXPECT_EQ(true, cache.IsMetricSet());
	EXPECT_EQ(cache.GetMetric(), float_t{-64});
}
