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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

#if 0
TEST(Rot, ByteSizeIs8)
{
    EXPECT_EQ(sizeof(Rot), std::size_t(8));
}

TEST(Rot, sin)
{
    Rot rot0(RealNum{0} * Degree);
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);
    
    EXPECT_EQ(RealNum{0}, round(rot0.sin()));
    EXPECT_EQ(RealNum{1}, round(rot90.sin()));
    EXPECT_EQ(RealNum{0}, round(rot180.sin()));
    EXPECT_EQ(RealNum{-1}, round(rot270.sin()));
    EXPECT_EQ(RealNum{0}, round(rot360.sin()));
    EXPECT_EQ(round(rot0.sin()), round(rot360.sin()));
    EXPECT_EQ(0.0f, round(std::asin(rot360.sin())));
}

TEST(Rot, cos)
{
    Rot rot0(RealNum{0} * Degree);
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * 1 * Pi);
    Rot rot360(Radian * 2 * Pi);
    EXPECT_EQ(round(rot0.cos()), round(rot360.cos()));
    EXPECT_EQ(RealNum{1}, round(rot0.cos()));
    EXPECT_EQ(RealNum{-1}, round(rot180.cos()));
    EXPECT_EQ(RealNum{1}, round(rot360.cos()));

    EXPECT_EQ(RealNum{0}, round(rot90.cos()));
}

TEST(Rot, Add)
{
    Rot rot0(RealNum{0} * Degree);
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);

    EXPECT_EQ(round(RealNum(0)), round(DegreesToRadians(0)));
    EXPECT_EQ(round(Pi/2), round(DegreesToRadians(90)));
    EXPECT_EQ(round(Pi), round(DegreesToRadians(180)));
    EXPECT_EQ(round(3*Pi/2), round(DegreesToRadians(270)));
    EXPECT_EQ(round(2*Pi), round(DegreesToRadians(360)));
    
    EXPECT_EQ(rot0, rot0.Rotate(rot0));    
    EXPECT_EQ(rot90, rot0.Rotate(rot90));
    EXPECT_EQ(rot180, rot90.Rotate(rot90));
    EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot180.Rotate(rot90))));
    EXPECT_EQ(round(ToRadians(Rot(20 * Degree))), round(ToRadians(Rot(30 * Degree).Rotate(Rot(-10 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(20 * Degree))), round(ToRadians(Rot(-10 * Degree).Rotate(Rot(30 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(20 * Degree))), round(ToRadians(Rot(10 * Degree).FlipY().Rotate(Rot(30 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(20 * Degree))), round(ToRadians(Rot(30 * Degree).Rotate(Rot(10 * Degree).FlipY()))));
    EXPECT_EQ(round(ToRadians(Rot(105 * Degree))), round(ToRadians(Rot(45 * Degree).Rotate(Rot(60 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(290 * Degree))), round(ToRadians(Rot(145 * Degree).Rotate(Rot(145 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(64 * Degree))), round(ToRadians(Rot(30 * Degree).Rotate(Rot(34 * Degree)))));
}

TEST(Rot, Negate)
{
    EXPECT_EQ(round(DegreesToRadians(0)), round(ToRadians(Rot(RealNum{0} * Degree).FlipY())));
    EXPECT_EQ(round(ToRadians(Rot(360 * Degree))), round(ToRadians(Rot(RealNum{0} * Degree).FlipY())));
    EXPECT_EQ(-round(DegreesToRadians(45)), round(ToRadians(Rot(45 * Degree).FlipY())));
    EXPECT_EQ(-round(DegreesToRadians(10)), round(ToRadians(Rot(10 * Degree).FlipY())));
    EXPECT_EQ(round(ToRadians(Rot(315 * Degree))), round(ToRadians(Rot(45 * Degree).FlipY())));
    EXPECT_EQ(round(ToRadians(Rot(270 * Degree))), round(ToRadians(Rot(90 * Degree).FlipY())));
    EXPECT_EQ(round(ToRadians(Rot(260 * Degree))), round(ToRadians(Rot(100 * Degree).FlipY())));
    EXPECT_EQ(-round(ToRadians(Rot(180 * Degree))), round(ToRadians(Rot(180 * Degree).FlipY())));
    EXPECT_EQ(round(ToRadians(Rot(64 * Degree))), round(ToRadians((Rot(30 * Degree).FlipY()).Rotate(Rot(94 * Degree)))));
    EXPECT_EQ(round(ToRadians(Rot(-64 * Degree))), round(ToRadians(Rot(30 * Degree).Rotate(Rot(94 * Degree).FlipY()))));    
}

TEST(Rot, Subtract)
{
    Rot rot0(RealNum{0} * Degree);
    Rot rot90(Radian * Pi / 2);
    Rot rot180(Radian * Pi);
    Rot rot270(Radian * 3 * Pi / 2);
    Rot rot360(Radian * 2 * Pi);

    EXPECT_EQ(round(ToRadians(rot0)), round(ToRadians(rot0.Rotate(rot0.FlipY()))));
    EXPECT_EQ(round(ToRadians(rot90)), round(ToRadians(rot90.Rotate(rot0.FlipY()))));
    EXPECT_EQ(round(ToRadians(rot180)), round(ToRadians(rot180.Rotate(rot0.FlipY()))));
    EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot270.Rotate(rot0))));
    
    EXPECT_NE(round(ToRadians(rot90)), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(round(ToRadians(rot270)), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(round(ToRadians(Rot(-90 * Degree))), round(ToRadians(rot0.Rotate(rot90.FlipY()))));
    EXPECT_EQ(round(ToRadians(Rot(64 * Degree))), round(ToRadians(Rot(34 * Degree).Rotate(Rot(-30 * Degree).FlipY()))));
    EXPECT_EQ(round(ToRadians(Rot(64 * Degree))), round(ToRadians(Rot(94 * Degree).Rotate(Rot(30 * Degree).FlipY()))));
}
#endif
