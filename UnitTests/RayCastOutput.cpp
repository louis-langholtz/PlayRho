/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

#include <type_traits>

using namespace playrho;
using namespace playrho::d2;

TEST(RayCastOutput, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(RayCastOutput), std::size_t(16)); break;
        case  8: EXPECT_EQ(sizeof(RayCastOutput), std::size_t(32)); break;
        case 16: EXPECT_EQ(sizeof(RayCastOutput), std::size_t(64)); break;
        default: FAIL(); break;
    }
}

TEST(RayCastOutput, DefaultConstruction)
{
    RayCastOutput foo{};
    EXPECT_FALSE(foo.has_value());
}

TEST(RayCastOutput, InitConstruction)
{
    const auto normal = UnitVec::GetLeft();
    const auto fraction = Real(0.8f);
    RayCastOutput foo{{normal, fraction}};
    EXPECT_TRUE(foo.has_value());
    EXPECT_EQ(foo->normal, normal);
    EXPECT_EQ(foo->fraction, fraction);
}

TEST(RayCastOutput, RayCastFreeFunctionHits)
{
    const auto radius = 0.1_m;
    const auto location = Length2(5_m, 2_m);
    const auto p1 = Length2(10_m, 2_m);
    const auto p2 = Length2(0_m, 2_m);
    const auto maxFraction = Real(1);
    auto input = RayCastInput{p1, p2, maxFraction};
    const auto output = RayCast(radius, location, input);
    ASSERT_TRUE(output.has_value());
    EXPECT_NEAR(static_cast<double>(output->normal.GetX()),
                static_cast<double>(UnitVec::GetRight().GetX()),
                0.02);
    EXPECT_NEAR(static_cast<double>(output->normal.GetY()),
                static_cast<double>(UnitVec::GetRight().GetY()),
                0.02);
    EXPECT_NEAR(static_cast<double>(output->fraction.get()), 0.49, 0.01);
}

TEST(RayCastOutput, RayCastLocationFreeFunctionMisses)
{
    {
        const auto radius = 0.1_m;
        const auto location = Length2(15_m, 2_m);
        const auto p1 = Length2(10_m, 2_m);
        const auto p2 = Length2(0_m, 2_m);
        const auto maxFraction = Real(1);
        auto input = RayCastInput{p1, p2, maxFraction};
        const auto output = RayCast(radius, location, input);
        EXPECT_FALSE(output.has_value());
    }
    {
        const auto radius = 0.1_m;
        const auto location = Length2(10_m, 3_m);
        const auto p1 = Length2(0_m, 2_m);
        const auto p2 = Length2(10_m, 2_m);
        const auto maxFraction = Real(1);
        auto input = RayCastInput{p1, p2, maxFraction};
        const auto output = RayCast(radius, location, input);
        EXPECT_FALSE(output.has_value());
    }
}

TEST(RayCastOutput, RayCastAabbFreeFunction)
{
    const auto p1 = Length2(10_m, 2_m);
    const auto p2 = Length2(0_m, 2_m);
    const auto maxFraction = Real(1);
    {
        const auto aabb = AABB{};
        EXPECT_FALSE(RayCast(aabb, RayCastInput{p1, p2, maxFraction}).has_value());
    }
    {
        const auto aabb = AABB{LengthInterval{9_m, 11_m}, LengthInterval{3_m, 1_m}};
        EXPECT_FALSE(RayCast(aabb, RayCastInput{p1, p2, maxFraction}).has_value());
        EXPECT_TRUE( RayCast(aabb, RayCastInput{p2, p1, maxFraction}).has_value());
    }
    {
        auto aabb = AABB{};
        aabb.ranges[0].Include(4_m).Include(5_m);
        aabb.ranges[1].Include(1_m).Include(3_m);
        
        const auto output1 = RayCast(aabb, RayCastInput{p1, p2, maxFraction});
        ASSERT_TRUE(output1.has_value());
        EXPECT_NEAR(static_cast<double>(Real{output1->fraction}), 0.5, 0.0001);
        EXPECT_EQ(output1->normal, UnitVec::GetRight());
        
        const auto output2 = RayCast(aabb, RayCastInput{p2, p1, maxFraction});
        ASSERT_TRUE(output2.has_value());
        EXPECT_NEAR(static_cast<double>(Real{output2->fraction}), 0.4, 0.0001);
        EXPECT_EQ(output2->normal, UnitVec::GetLeft());

        const auto output3 = RayCast(aabb, RayCastInput{Length2{}, Length2{5_m, 6_m}, maxFraction});
        ASSERT_FALSE(output3.has_value());
    }
}

TEST(RayCastOutput, RayCastDistanceProxyFF)
{
    const auto pos1 = Length2{3_m, 1_m}; // bottom right
    const auto pos2 = Length2{3_m, 3_m}; // top right
    const auto pos3 = Length2{1_m, 3_m}; // top left
    const auto pos4 = Length2{1_m, 1_m}; // bottom left
    const Length2 squareVerts[] = {pos1, pos2, pos3, pos4};
    const auto n1 = GetUnitVector(GetFwdPerpendicular(pos2 - pos1));
    const auto n2 = GetUnitVector(GetFwdPerpendicular(pos3 - pos2));
    const auto n3 = GetUnitVector(GetFwdPerpendicular(pos4 - pos3));
    const auto n4 = GetUnitVector(GetFwdPerpendicular(pos1 - pos4));
    const UnitVec squareNormals[] = {n1, n2, n3, n4};
    const auto radius = 0.5_m;
    DistanceProxy dp{radius, 4, squareVerts, squareNormals};

    const auto p1 = Length2(0_m, 2_m);
    const auto p2 = Length2(10_m, 2_m);
    const auto maxFraction = Real(1);
    const auto input0 = RayCastInput{p1, p2, maxFraction};
    {
        const auto output = RayCast(dp, input0, Transform_identity);
        EXPECT_TRUE(output.has_value());
        if (output.has_value())
        {
            EXPECT_EQ(output->normal, UnitVec::GetLeft());
            EXPECT_NEAR(static_cast<double>(output->fraction.get()), 0.05, 0.002);
        }
    }
    
    const auto p0 = Length2{};
    const auto input1 = RayCastInput{p0, p1, maxFraction};
    {
        const auto output = RayCast(dp, input1, Transform_identity);
        EXPECT_FALSE(output.has_value());
    }
}

TEST(RayCastOutput, RayCastShapeFF)
{
    const auto p1 = Length2{+4_m, 0_m}; // bottom right
    const auto p2 = Length2{+0_m, 0_m}; // top right
    const auto maxFraction = Real(1);
    const auto input = RayCastInput{p1, p2, maxFraction};
    const auto xfm = Transform_identity;
    const auto output = RayCast(Shape{DiskShapeConf{1_m}}, ChildCounter{0}, input, xfm);
    EXPECT_TRUE(output.has_value());
    EXPECT_EQ(output->normal, d2::UnitVec::GetRight());
    EXPECT_NEAR(static_cast<double>(Real{output->fraction}), 0.75, 0.01);
}

TEST(RayCastHit, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(RayCastHit), std::size_t(12)); break;
        case  8: EXPECT_EQ(sizeof(RayCastHit), std::size_t(24)); break;
        case 16: EXPECT_EQ(sizeof(RayCastHit), std::size_t(48)); break;
        default: FAIL(); break;
    }
}

TEST(RayCastHit, Traits)
{
    EXPECT_TRUE(std::is_default_constructible<RayCastHit>::value);
    //EXPECT_FALSE(std::is_nothrow_default_constructible<RayCastHit>::value); // on clang 4.0 or older
    //EXPECT_TRUE(std::is_nothrow_default_constructible<RayCastHit>::value); // on gcc 6
    EXPECT_FALSE(std::is_trivially_default_constructible<RayCastHit>::value);

    EXPECT_TRUE(std::is_constructible<RayCastHit>::value);
    //EXPECT_FALSE(std::is_nothrow_constructible<RayCastHit>::value); // on clang 4.0 or older
    //EXPECT_TRUE(std::is_nothrow_constructible<RayCastHit>::value); // on gcc 6
    EXPECT_FALSE(std::is_trivially_constructible<RayCastHit>::value);

    EXPECT_TRUE(std::is_copy_constructible<RayCastHit>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<RayCastHit>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<RayCastHit>::value);

    EXPECT_TRUE(std::is_copy_assignable<RayCastHit>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<RayCastHit>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<RayCastHit>::value);

    EXPECT_TRUE(std::is_destructible<RayCastHit>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<RayCastHit>::value);
    EXPECT_TRUE(std::is_trivially_destructible<RayCastHit>::value);
}

TEST(RayCastHit, DefaultConstruction)
{
    RayCastHit foo{};
    EXPECT_FALSE(IsValid(foo.normal));
}

TEST(RayCastHit, InitConstruction)
{
    const auto normal = UnitVec::GetLeft();
    const auto fraction = Real(0.8f);
    RayCastHit foo{normal, fraction};
    EXPECT_EQ(foo.normal, normal);
    EXPECT_EQ(foo.fraction, fraction);
}

