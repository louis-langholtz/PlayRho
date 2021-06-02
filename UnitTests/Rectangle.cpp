/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Collision/Shapes/Rectangle.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

using namespace playrho;
using namespace playrho::shape_part;
using namespace playrho::d2;

TEST(Rectangle, ByteSize)
{
    EXPECT_EQ(sizeof(Rectangle<1, 1>), 1u);
    EXPECT_EQ(sizeof(Rectangle<1, 1, DensityIs<StaticAreaDensity<6>>>), 1u);
    EXPECT_EQ(sizeof(Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>,
                               FrictionIs<StaticTenthsFriction<3>>>{}),
              1u);
    EXPECT_EQ(sizeof(Rectangle<1, 2, FrictionIs<StaticFriction<>>>), 1u);
    switch (sizeof(Real)) {
    case 4u:
        EXPECT_EQ(
            sizeof(
                Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>, FrictionIs<DynamicFriction<4>>>{}),
            4u);
        EXPECT_EQ(
            sizeof(Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>, FrictionIs<StaticFriction<4>>,
                             RestitutionIs<DynamicRestitution<>>>{}),
            4u);
        EXPECT_EQ(sizeof(Rectangle<1, 2, RestitutionIs<DynamicRestitution<>>>), 4u);
        EXPECT_EQ(sizeof(Rectangle<1, 2, RestitutionIs<DynamicRestitution<>>,
                                   FrictionIs<DynamicFriction<>>>),
                  8u);
        break;
    case 8u:
        EXPECT_EQ(
            sizeof(
                Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>, FrictionIs<DynamicFriction<4>>>{}),
            8u);
        EXPECT_EQ(
            sizeof(Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>, FrictionIs<StaticFriction<4>>,
                             RestitutionIs<DynamicRestitution<>>>{}),
            8u);
        EXPECT_EQ(sizeof(Rectangle<1, 2, RestitutionIs<DynamicRestitution<>>>), 8u);
        EXPECT_EQ(sizeof(Rectangle<1, 2, RestitutionIs<DynamicRestitution<>>,
                                   FrictionIs<DynamicFriction<>>>),
                  16u);
        break;
    }
}

TEST(Rectangle, GetChildCount)
{
    EXPECT_EQ(GetChildCount(Rectangle<1, 1>{}), 1u);
    EXPECT_EQ(GetChildCount(Rectangle<1, 1, StaticAreaDensity<6>>{}), 1u);
}

TEST(Rectangle, GetDensity)
{
    EXPECT_EQ(GetDensity(Rectangle<1, 1, DensityIs<StaticAreaDensity<4>>>{}), 4_kgpm2);
    EXPECT_EQ(GetDensity(Rectangle<1, 1, DensityIs<StaticAreaDensity<5>>>{}), 5_kgpm2);
    EXPECT_EQ(GetDensity(Rectangle<1, 1, DensityIs<DynamicAreaDensity<6>>>{}), 6_kgpm2);
    EXPECT_EQ(GetDensity(Rectangle<1, 1, DensityIs<DynamicAreaDensity<6>>>{{2.4_kgpm2}}),
              2.4_kgpm2);
}

TEST(Rectangle, GetFriction)
{
    EXPECT_EQ(GetFriction(Rectangle<1, 1>{}), Real(0.2));
    EXPECT_EQ(GetFriction(Rectangle<1, 1, FrictionIs<StaticTenthsFriction<>>>{}), Real(0.2));
    EXPECT_EQ(GetFriction(Rectangle<1, 1, FrictionIs<StaticTenthsFriction<3>>>{}), Real(0.3));
    EXPECT_EQ(GetFriction(Rectangle<1, 1, FrictionIs<DynamicFriction<4>>>{}), Real(4));
    EXPECT_EQ(GetFriction(Rectangle<1, 1, FrictionIs<DynamicFriction<4>>>{{}, {Real(0.5)}}),
              Real(0.5));
}

TEST(Rectangle, GetRestitution)
{
    EXPECT_EQ(GetRestitution(Rectangle<1, 1>{}), Real(0));
    EXPECT_EQ(GetRestitution(Rectangle<1, 1, RestitutionIs<StaticRestitution<1>>>{}), Real(1));
    EXPECT_EQ(GetRestitution(Rectangle<1, 1, RestitutionIs<DynamicRestitution<8>>>{}), Real(8));
    EXPECT_EQ(
        GetRestitution(Rectangle<1, 1, RestitutionIs<DynamicRestitution<8>>>{{}, {}, {Real(1.2)}}),
        Real(1.2));
}

TEST(Rectangle, SetFriction)
{
    {
        auto rectangle = Rectangle<1, 1>{};
        ASSERT_EQ(rectangle.friction, Real(0.2));
        EXPECT_THROW(SetFriction(rectangle, Real(3)), InvalidArgument);
        EXPECT_EQ(rectangle.friction, Real(0.2));
    }
    {
        auto rectangle = Rectangle<1, 1, FrictionIs<DynamicFriction<>>>{};
        ASSERT_EQ(rectangle.friction, Real(0));
        EXPECT_NO_THROW(SetFriction(rectangle, Real(3)));
        EXPECT_EQ(rectangle.friction, Real(3));
    }
}

TEST(Rectangle, SetRestitution)
{
    {
        auto rectangle = Rectangle<1, 1>{};
        ASSERT_EQ(rectangle.restitution, Real(0));
        EXPECT_THROW(SetRestitution(rectangle, Real(3)), InvalidArgument);
        EXPECT_EQ(rectangle.restitution, Real(0));
    }
    {
        auto rectangle = Rectangle<1, 1, RestitutionIs<DynamicRestitution<>>>{};
        ASSERT_EQ(rectangle.restitution, Real(0));
        EXPECT_NO_THROW(SetRestitution(rectangle, Real(3)));
        EXPECT_EQ(rectangle.restitution, Real(3));
    }
}
