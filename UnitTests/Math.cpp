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
#include <type_traits>
#include <chrono>
#include <cmath>

using namespace box2d;

TEST(Math, Sqrt)
{
    EXPECT_EQ(Sqrt(RealNum{0}), RealNum{0});
    EXPECT_EQ(Sqrt(RealNum{4}), RealNum{2});
    EXPECT_EQ(Sqrt(RealNum{25}), RealNum{5});
    EXPECT_NE(Sqrt(std::numeric_limits<RealNum>::min()), RealNum(0));

    EXPECT_NE(std::sqrt(std::numeric_limits<double>::min()), double(0));
    EXPECT_EQ(Square(std::sqrt(std::numeric_limits<double>::min())), std::numeric_limits<double>::min());
}

TEST(Math, Square)
{
    ASSERT_NE(std::numeric_limits<float>::min() * 2, std::numeric_limits<float>::min());

    EXPECT_EQ(Square(std::numeric_limits<float>::min()), float(0));
    EXPECT_EQ(Square(std::numeric_limits<float>::min() * float(2251799947902976)), float(0));
    EXPECT_NE(Square(std::numeric_limits<float>::min() * float(2251799947902977)), float(0));

    auto low = float(0);
    auto high = float(0);
    auto value = float(0);

    low = std::numeric_limits<float>::min() * float(2251799947902976);
    high = std::numeric_limits<float>::min() * float(2251799947902977);
    do
    {
        value = (low + high) / float(2);
        if ((value == low) || (value == high))
        {
            break;
        }
        if (Square(value) != float(0))
        {
            high = value;
        }
        else
        {
            low = value;
        }
    }
    while (low < high);
    
#if 0
    std::cout << "Min float is: ";
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << std::numeric_limits<float>::min();
    std::cout << " aka ";
    std::cout << std::hexfloat;
    std::cout << std::numeric_limits<float>::min();
    std::cout << std::defaultfloat;
    std::cout << std::endl;

    std::cout << "Least float that squared isn't zero: ";
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << high;
    std::cout << " aka ";
    std::cout << std::hexfloat;
    std::cout << high;
    std::cout << std::defaultfloat;
    std::cout << std::endl;
#endif
    EXPECT_EQ(high, float(2.646978275714050648e-23));

    ASSERT_NE(Square(high), float(0));
    ASSERT_EQ(Sqrt(Square(float(1))), float(1));
#if 0
    std::cout << "Sqrt(min) is: ";
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << Sqrt(std::numeric_limits<float>::min());
    std::cout << " aka ";
    std::cout << std::hexfloat;
    std::cout << Sqrt(std::numeric_limits<float>::min());
    std::cout << std::endl;
    EXPECT_EQ(Sqrt(std::numeric_limits<float>::min()), float(0x1p-63)); // float(1.084202172485504434e-19)
#endif
    
    // What is the smallest float a for which:
    // almost_equal(sqrt(square(a)), a) and almost_equal(square(sqrt(a)), a)
    // hold true?
    
    const auto a = Sqrt(std::numeric_limits<float>::min());
    EXPECT_TRUE(almost_equal(Square(Sqrt(a)), a));
    EXPECT_TRUE(almost_equal(Sqrt(Square(a)), a));
}

TEST(Math, Atan2)
{
    EXPECT_EQ(Atan2(RealNum(0), RealNum(0)), Angle{0});
    //EXPECT_EQ(Atan2(RealNum(1), RealNum(0)), RealNum(90) * Degree);
}

TEST(Math, Span)
{
    {
        // check aggragate initialization
        const Span<const int> foo = {1, 2, 4};
        EXPECT_EQ(foo.size(), size_t(3));
        EXPECT_EQ(foo[0], 1);
        EXPECT_EQ(foo[1], 2);
        EXPECT_EQ(foo[2], 4);
    }
    {
        // check initialization from explicit initializer list
        const auto foo = Span<const int>(std::initializer_list<int>{1, 2, 4});
        EXPECT_EQ(foo.size(), size_t(3));
        EXPECT_EQ(foo[0], 1);
        EXPECT_EQ(foo[1], 2);
        EXPECT_EQ(foo[2], 4);
    }
    {
        // check initialization from non-const array
        int array[6] = {1, 2, 4, 10, -1, -33};
        auto foo = Span<int>(array);
        EXPECT_EQ(foo.size(), size_t(6));
        EXPECT_EQ(foo[0], 1);
        EXPECT_EQ(foo[1], 2);
        EXPECT_EQ(foo[2], 4);
        EXPECT_EQ(foo[3], 10);
        EXPECT_EQ(foo[4], -1);
        EXPECT_EQ(foo[5], -33);
        foo[3] = 22;
        EXPECT_EQ(foo[3], 22);
    }
    {
        float array[15];
        EXPECT_EQ(Span<float>(array).size(), size_t(15));
        EXPECT_EQ(Span<float>(array, 2).size(), size_t(2));        
        EXPECT_EQ(Span<float>(array, array + 4).size(), size_t(4));
        EXPECT_EQ(Span<float>(array + 1, array + 3).size(), size_t(2));
    }
}

TEST(Math, Average)
{
    EXPECT_EQ(Average<int>({}), 0);
    EXPECT_EQ(Average<float>({}), float(0));

    EXPECT_EQ(Average<int>({0}), 0);
    EXPECT_EQ(Average<int>({4}), 4);
    EXPECT_EQ(Average<int>({-3}), -3);
    EXPECT_EQ(Average<float>({float(-3)}), float(-3));

    EXPECT_EQ(Average<int>({0, 0}), 0);
    EXPECT_EQ(Average<int>({2, 2}), 2);
    EXPECT_EQ(Average<int>({2, 4}), 3);
    EXPECT_EQ(Average<float>({float(2), float(3)}), float(2.5));

    EXPECT_EQ(Average<int>({2, 4, 6}), 4);
    EXPECT_EQ(Average<int>({2, 4, 12}), 6);
    EXPECT_EQ(Average<double>({2.0, 4.0, 6.0}), 4.0);
    EXPECT_EQ(Average<double>({2.0, 4.0, 12.0}), 6.0);
}

TEST(Math, AverageVec2)
{
    EXPECT_EQ(Average<Vec2>({}), Vec2(0, 0));
    
    {
        const auto val = Vec2{RealNum(3.9), RealNum(-0.1)};
        EXPECT_EQ(Average<Vec2>({val}), val);
    }
    
    {
        const auto val1 = Vec2{RealNum(2.2), RealNum(-1.1)};
        const auto val2 = Vec2{RealNum(4.4), RealNum(-1.3)};
        const auto average = Average<Vec2>({val1, val2});
        const auto expected = Vec2(RealNum(3.3), RealNum(-1.2));
        EXPECT_NEAR(double(average.x), double(expected.x), 0.0001);
        EXPECT_NEAR(double(average.y), double(expected.y), 0.0001);
    }
}

TEST(Math, DotProductOfTwoVecTwoIsCommutative)
{
    const auto a = Vec2{RealNum(-3.2), RealNum(1.9)};
    const auto b = Vec2{RealNum(4.01), RealNum(-0.002)};
    EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, DotProductOfTwoVecThreeIsCommutative)
{
    const auto a = Vec3{RealNum(-3.2), RealNum(1.9), RealNum(36.01)};
    const auto b = Vec3{RealNum(4.01), RealNum(-0.002), RealNum(1.2)};
    EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, CrossProductOfTwoVecTwoIsAntiCommutative)
{
    const auto a = Vec2{RealNum(-3.2), RealNum(1.9)};
    const auto b = Vec2{RealNum(4.01), RealNum(-0.002)};
    EXPECT_EQ(Cross(a, b), -Cross(b, a));
}

TEST(Math, DotProductOfInvalidIsInvalid)
{
    EXPECT_TRUE(std::isnan(Dot(GetInvalid<Vec2>(), GetInvalid<Vec2>())));

    EXPECT_TRUE(std::isnan(Dot(Vec2(0, 0), GetInvalid<Vec2>())));
    EXPECT_TRUE(std::isnan(Dot(Vec2(0, 0), Vec2(GetInvalid<RealNum>(), 0))));
    EXPECT_TRUE(std::isnan(Dot(Vec2(0, 0), Vec2(0, GetInvalid<RealNum>()))));
    
    EXPECT_TRUE(std::isnan(Dot(GetInvalid<Vec2>(),             Vec2(0, 0))));
    EXPECT_TRUE(std::isnan(Dot(Vec2(GetInvalid<RealNum>(), 0), Vec2(0, 0))));
    EXPECT_TRUE(std::isnan(Dot(Vec2(0, GetInvalid<RealNum>()), Vec2(0, 0))));

    EXPECT_TRUE(std::isnan(Dot(GetInvalid<Vec2>(), GetInvalid<UnitVec2>())));
    EXPECT_TRUE(std::isnan(Dot(Vec2(0, 0),         GetInvalid<UnitVec2>())));
    EXPECT_TRUE(std::isnan(Dot(GetInvalid<Vec2>(), UnitVec2::GetZero())));

    EXPECT_TRUE(std::isnan(Dot(GetInvalid<UnitVec2>(), GetInvalid<Vec2>())));
    EXPECT_TRUE(std::isnan(Dot(GetInvalid<UnitVec2>(), Vec2(0, 0))));
    EXPECT_TRUE(std::isnan(Dot(UnitVec2::GetZero(),    GetInvalid<Vec2>())));
}

TEST(Math, Vec2NegationAndRotationIsOrderIndependent)
{
    {
        const auto v = Vec2{RealNum(1), RealNum(1)};
        const auto r = UnitVec2{RealNum{0} * Degree};
        EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
    }
    {
        const auto v = Vec2{RealNum(1), RealNum(1)};
        const auto r = UnitVec2{RealNum{33.0f} * Degree};
        EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
    }
    {
        const auto v = Vec2{RealNum(-3.2), RealNum(1.9)};
        const auto r = UnitVec2{RealNum{33.0f} * Degree};
        EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
    }
    {
        const auto v = Vec2{RealNum(-3.2), RealNum(-21.4)};
        for (auto angle = -RealNum{360.0f} * Degree; angle < RealNum{360.0f} * Degree; angle += RealNum{15.0f} * Degree)
        {
            const auto r = UnitVec2{angle};
            EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
        }
    }
    {
        const auto v = Vec2{RealNum(-3.2), RealNum(1.9)};
        const auto r = UnitVec2{RealNum{33.0f} * Degree};
        EXPECT_EQ(Rotate(v, r), -Rotate(-v, r));
    }
    {
        const auto v = Vec2{RealNum(-3.2), RealNum(1.9)};
        const auto r = UnitVec2{RealNum{33.0f} * Degree};
        EXPECT_EQ(Rotate(v, r), -Rotate(v, -r));
    }
}

TEST(Math, InverseRotationRevertsRotation)
{
    const auto vec_list = {Vec2{-10.7f, 5.3f}, Vec2{3.2f, 21.04f}, Vec2{-1.2f, -0.78f}};
    for (auto&& vec: vec_list) {
        for (auto angle = RealNum{0} * Degree; angle < RealNum{360.0f} * Degree; angle += RealNum{10.0f} * Degree)
        {
            const auto unit_vec = UnitVec2{angle};
            EXPECT_NEAR(double(GetX(InverseRotate(Rotate(vec, unit_vec), unit_vec))), double(GetX(vec)), 0.004);
            EXPECT_NEAR(double(GetY(InverseRotate(Rotate(vec, unit_vec), unit_vec))), double(GetY(vec)), 0.004);
        }
    }
}

TEST(Math, TransformIsRotatePlusTranslate)
{
    const auto vector = Vec2{RealNum(19), RealNum(-0.5)} * Meter;
    const auto translation = Vec2{RealNum(-3), RealNum(+5)} * Meter;
    const auto rotation = UnitVec2{RealNum{90.0f} * Degree};
    const auto transformation = Transformation{translation, rotation};
    
    const auto transformed_vector = Transform(vector, transformation);
    const auto alt = Rotate(vector, rotation) + translation;
    
    EXPECT_EQ(transformed_vector.x, alt.x);
    EXPECT_EQ(transformed_vector.y, alt.y);
}

TEST(Math, InverseTransformIsUntranslateAndInverseRotate)
{
    const auto vector = Vec2{RealNum(19), RealNum(-0.5)} * Meter;
    const auto translation = Vec2{RealNum(-3), RealNum(+5)} * Meter;
    const auto rotation = UnitVec2{RealNum{90.0f} * Degree};
    const auto transformation = Transformation{translation, rotation};
    
    const auto inv_vector = InverseTransform(vector, transformation);
    const auto alt = InverseRotate(vector - translation, rotation);
    
    EXPECT_EQ(inv_vector.x, alt.x);
    EXPECT_EQ(inv_vector.y, alt.y);
}

TEST(Math, InverseTransformTransformedIsOriginal)
{
    const auto vector = Vec2{RealNum(19), RealNum(-0.5)} * Meter;
    const auto translation = Vec2{RealNum(-3), RealNum(+5)} * Meter;
    const auto rotation = UnitVec2{RealNum{90.0f} * Degree};
    const auto transformation = Transformation{translation, rotation};

    const auto transformed_vector = Transform(vector, transformation);
    const auto inverse_transformed_vector = InverseTransform(transformed_vector, transformation);

    EXPECT_NEAR(double(RealNum{vector.x / Meter}),
                double(RealNum{inverse_transformed_vector.x / Meter}), 0.0001);
    EXPECT_NEAR(double(RealNum{vector.y / Meter}),
                double(RealNum{inverse_transformed_vector.y / Meter}), 0.0001);
}

TEST(Math, TransformInverseTransformedIsOriginal)
{
    const auto vector = Vec2{RealNum(19), RealNum(-0.5)} * Meter;
    const auto translation = Vec2{RealNum(-3), RealNum(+5)} * Meter;
    const auto rotation = UnitVec2{RealNum{90.0f} * Degree};
    const auto transformation = Transformation{translation, rotation};

    const auto inverse_transformed_vector = InverseTransform(vector, transformation);
    const auto transformed_inverse_vector = Transform(inverse_transformed_vector, transformation);
    
    EXPECT_NEAR(double(RealNum{vector.x / Meter}),
                double(RealNum{transformed_inverse_vector.x / Meter}), 0.00001);
    EXPECT_NEAR(double(RealNum{vector.y / Meter}),
                double(RealNum{transformed_inverse_vector.y / Meter}), 0.00001);
}

TEST(Math, ComputeCentroidCenteredR1)
{
    const auto hx = RealNum(1);
    const auto hy = RealNum(1);
    const auto real_center = Vec2{0, 0};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter,
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_EQ(center.x, real_center.x * Meter);
    EXPECT_EQ(center.y, real_center.y * Meter);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_EQ(average.x, center.x);
    EXPECT_EQ(average.y, center.y);
}

template <typename T>
struct Results {
    constexpr static auto expected_ctr = Vec2{0,0};
};

template <>
struct Results<Fixed32> {
    constexpr static auto expected_ctr = Vec2{0,0};
};

TEST(Math, ComputeCentroidCentered0R1000)
{
    const auto hx = RealNum(1000);
    const auto hy = RealNum(1000);
    const auto real_center = Vec2{0, 0};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter
    };
    const auto center = ComputeCentroid(vertices);
    
    EXPECT_EQ(center.x, real_center.x * Meter);
    EXPECT_EQ(center.y, real_center.y * Meter);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_EQ(average.x, center.x);
    EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight1000R1)
{
    const auto hx = RealNum(1);
    const auto hy = RealNum(1);
    const auto real_center = Vec2{1000, 1000};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_NEAR(double(RealNum{center.x / Meter}), double(real_center.x), 0.01);
    EXPECT_NEAR(double(RealNum{center.y / Meter}), double(real_center.y), 0.01);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_NEAR(double(RealNum{average.x / Meter}), double(RealNum{center.x / Meter}), 0.01);
    EXPECT_NEAR(double(RealNum{average.y / Meter}), double(RealNum{center.y / Meter}), 0.01);
}

TEST(Math, ComputeCentroidUpRight1000R100)
{
    const auto hx = RealNum(100);
    const auto hy = RealNum(100);
    const auto real_center = Vec2{1000, 1000};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_NEAR(double(RealNum{center.x / Meter}), double(real_center.x), 0.01);
    EXPECT_NEAR(double(RealNum{center.y / Meter}), double(real_center.y), 0.01);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_NEAR(double(RealNum{average.x / Meter}), double(RealNum{center.x / Meter}), 0.01);
    EXPECT_NEAR(double(RealNum{average.y / Meter}), double(RealNum{center.y / Meter}), 0.01);
}

TEST(Math, ComputeCentroidUpRight10000R01)
{
    const auto hx = RealNum(0.1);
    const auto hy = RealNum(0.1);
    const auto real_center = Vec2{10000, 10000};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_NEAR(double(RealNum{center.x / Meter}), double(real_center.x), 0.1);
    EXPECT_NEAR(double(RealNum{center.y / Meter}), double(real_center.y), 0.1);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_NEAR(double(RealNum{average.x / Meter}), double(RealNum{center.x / Meter}), 0.1);
    EXPECT_NEAR(double(RealNum{average.y / Meter}), double(RealNum{center.y / Meter}), 0.1);
}

TEST(Math, ComputeCentroidDownLeft1000R1)
{
    const auto hx = RealNum(1);
    const auto hy = RealNum(1);
    const auto real_center = Vec2{-1000, -1000};
    const auto vertices = {
        Vec2{real_center.x + hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - hy} * Meter
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_NEAR(double(RealNum{center.x / Meter}), double(real_center.x), 0.01);
    EXPECT_NEAR(double(RealNum{center.y / Meter}), double(real_center.y), 0.01);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_NEAR(double(RealNum{average.x / Meter}), double(RealNum{center.x / Meter}), 0.01);
    EXPECT_NEAR(double(RealNum{average.y / Meter}), double(RealNum{center.y / Meter}), 0.01);
}

TEST(Math, ComputeCentroidOfHexagonalVertices)
{
    const auto hx = RealNum(1);
    const auto hy = RealNum(1);
    const auto real_center = Vec2{-1000, -1000};
    const auto vertices = {
        Vec2{real_center.x + 00, real_center.y + 2 * hy} * Meter,
        Vec2{real_center.x - hx, real_center.y + 1 * hy} * Meter,
        Vec2{real_center.x - hx, real_center.y - 1 * hy} * Meter,
        Vec2{real_center.x + 00, real_center.y - 2 * hy} * Meter,
        Vec2{real_center.x + hx, real_center.y - 1 * hy} * Meter,
        Vec2{real_center.x + hx, real_center.y + 1 * hy} * Meter,
    };
    const auto center = ComputeCentroid(vertices);
    EXPECT_NEAR(double(RealNum{center.x / Meter}), double(real_center.x), 0.01);
    EXPECT_NEAR(double(RealNum{center.y / Meter}), double(real_center.y), 0.01);
    
    const auto average = Average<Length2D>(vertices);
    EXPECT_NEAR(double(RealNum{average.x / Meter}), double(RealNum{center.x / Meter}), 0.01);
    EXPECT_NEAR(double(RealNum{average.y / Meter}), double(RealNum{center.y / Meter}), 0.01);
}

TEST(Math, GetContactRelVelocity)
{
    const auto velA = Velocity{Vec2(+1, +4) * MeterPerSecond, RealNum{3.2f} * RadianPerSecond};
    const auto velB = Velocity{Vec2(+3, +1) * MeterPerSecond, RealNum{0.4f} * RadianPerSecond};
    const auto relA = Vec2(0, 0) * Meter;
    const auto relB = Vec2(0, 0) * Meter;
    const auto result = GetContactRelVelocity(velA, relA, velB, relB);
    
    EXPECT_EQ(result, velB.linear - velA.linear);
}

TEST(Math, NextPowerOfTwo)
{
    EXPECT_EQ(NextPowerOfTwo(0u), 1u);
    EXPECT_EQ(NextPowerOfTwo(1u), 2u);
    EXPECT_EQ(NextPowerOfTwo(2u), 4u);
    EXPECT_EQ(NextPowerOfTwo(3u), 4u);
    EXPECT_EQ(NextPowerOfTwo(4u), 8u);
    EXPECT_EQ(NextPowerOfTwo(5u), 8u);
    EXPECT_EQ(NextPowerOfTwo(6u), 8u);
    EXPECT_EQ(NextPowerOfTwo(7u), 8u);
    EXPECT_EQ(NextPowerOfTwo(8u), 16u);
    EXPECT_EQ(NextPowerOfTwo(9u), 16u);
    EXPECT_EQ(NextPowerOfTwo(10u), 16u);
    EXPECT_EQ(NextPowerOfTwo(11u), 16u);
    EXPECT_EQ(NextPowerOfTwo(12u), 16u);
    EXPECT_EQ(NextPowerOfTwo(13u), 16u);
    EXPECT_EQ(NextPowerOfTwo(14u), 16u);
    EXPECT_EQ(NextPowerOfTwo(15u), 16u);
    EXPECT_EQ(NextPowerOfTwo(16u), 32u);

    constexpr auto max = std::numeric_limits<std::uint32_t>::max() / 512;
    for (auto i = decltype(max){0}; i < max; ++i)
    {
        const auto next = std::pow(2, std::ceil(std::log(i + 1)/std::log(2)));
        EXPECT_EQ(NextPowerOfTwo(i), next);
    }
}

TEST(Math, Subtracting2UlpAlmostEqualNumbersNotAlmostZero)
{
    const auto a = 0.863826155f;
    const auto b = 0.863826453f;
    ASSERT_NE(a, b);
    ASSERT_TRUE(almost_equal(a, b, 2));
    ASSERT_FALSE(almost_equal(a, b, 1));
    EXPECT_FALSE(almost_zero((a >= b)? a - b: b - a));
}

TEST(Math, Subtracting1UlpAlmostEqualNumbersIsNotAlmostZero)
{
    const auto a = 0.8638264550000f;
    const auto b = 0.8638264238828f;
    ASSERT_NE(a, b);
    ASSERT_TRUE(almost_equal(a, b, 1));
    ASSERT_FALSE(almost_equal(a, b, 0));
    EXPECT_FALSE(almost_zero((a >= b)? a - b: b - a));
}

TEST(Math, nextafter)
{
    const auto a = float(0.863826394);
    const auto b = float(0.863826453);
    
    ASSERT_NE(a, b);
    ASSERT_TRUE(almost_equal(a, b, 2));

    const auto ap = std::nextafter(a, a + 1);
    
    EXPECT_NE(a, ap);
    EXPECT_EQ(ap, b);
    EXPECT_EQ((a + b) / 2, a);
}

TEST(Math, nextafter2)
{
    const auto a = 0.863826155f;
    const auto b = std::nextafter(a, 1.0f);
    ASSERT_TRUE(almost_equal(a, b, 2));
    ASSERT_TRUE(almost_equal(a, b, 1));
    ASSERT_FALSE(almost_equal(a, b, 0));
    ASSERT_TRUE(a != b);
    const auto d = b - a;
    ASSERT_FALSE(almost_zero(d));
    EXPECT_EQ(a + d, b);
    EXPECT_EQ(b - d, a);
    const auto minfloat = std::numeric_limits<float>::min();
    ASSERT_NE(minfloat, 0.0f);
    ASSERT_TRUE(minfloat > 0.0f);
    ASSERT_NE(minfloat, d);
    ASSERT_FALSE(almost_zero(minfloat));
    const auto subnormal = minfloat / 2;
    ASSERT_TRUE(almost_zero(subnormal));
    ASSERT_NE(minfloat, subnormal);
    EXPECT_EQ(a + subnormal, a);
    EXPECT_EQ(b + subnormal, b);
}

TEST(Math, GetPosition)
{
    /*
     * If GetPosition is implemented as: pos0 * (1 - beta) + pos1 * beta.
     * Then it fails the following test when RealNum is implemented via float.
     * This is due to floating point inaccuracy.
     *
     * If GetPosition is implemented as: pos0 + (pos1 - pos0) * beta.
     * Then it passes the following test when RealNum is implemented via float.
     */

    const auto x = RealNum{2.587699890136719e-02f};
    const auto y = RealNum{5.515012264251709e+00f};
    const auto value = RealNum{0.0866042823f};

    const auto oldPos = Position{Vec2{x, y} * Meter, RealNum{0.0f} * Radian};
    const auto newPos = GetPosition(oldPos, oldPos, value);
    
    EXPECT_EQ(oldPos.linear.x, newPos.linear.x);
    EXPECT_EQ(oldPos.linear.y, newPos.linear.y);
    EXPECT_EQ(oldPos.angular, newPos.angular);
}

TEST(Math, ToiTolerance)
{
    // What is the max vr for which the following still holds true?
    //   vr + DefaultLinearSlop / 4 > vr
    // The max vr for which (std::nextafter(vr, MaxFloat) - vr) <= DefaultLinearSlop / 4.
    // I.e. the max vr for which (std::nextafter(vr, MaxFloat) - vr) <= 0.000025

    const auto linearSlop = 0.0001f;
    const auto tolerance = linearSlop / 4;
    {
        const auto vr = 511.0f;
        EXPECT_GT(vr + tolerance, vr);
    }
    {
        const auto vr = 512.0f;
        EXPECT_EQ(vr + tolerance, vr);
    }
}

struct Coords {
    float x, y;
};

TEST(Math, LengthFasterThanHypot)
{
    constexpr auto iterations = unsigned(5000000);
    
    std::chrono::duration<double> elapsed_secs_length;
    std::chrono::duration<double> elapsed_secs_hypot;
    
    const auto v1 = Coords{10.8f, 99.02f};
    const auto v2 = Coords{-6.01f, 31.2f};
    const auto v3 = Coords{409183.2f, 0.00023f};
    const auto v4 = Coords{-0.004f, 0.001f};
    const auto v5 = Coords{-432.1f, -9121.0f};
    const auto v6 = Coords{32.1f, -21.0f};
    const auto v7 = Coords{12088.032f, 7612.823f};
    const auto v8 = Coords{7612.823f, -7612.823f};

    auto totalLength = 0.0f;
    auto totalHypot = 0.0f;

    {
        // Time the "length" algorithm: sqrt(x^2 + y^2).
        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        start = std::chrono::high_resolution_clock::now();
        for (auto i = decltype(iterations){0}; i < iterations; ++i)
        {
            const auto l1 = std::sqrt(Square(v1.x * i) + Square(v1.y * i));
            const auto l2 = std::sqrt(Square(v2.x * i) + Square(v2.y * i));
            const auto l3 = std::sqrt(Square(v3.x * i) + Square(v3.y * i));
            const auto l4 = std::sqrt(Square(v4.x * i) + Square(v4.y * i));
            const auto l5 = std::sqrt(Square(v5.x * i) + Square(v5.y * i));
            const auto l6 = std::sqrt(Square(v6.x * i) + Square(v6.y * i));
            const auto l7 = std::sqrt(Square(v7.x * i) + Square(v7.y * i));
            const auto l8 = std::sqrt(Square(v8.x * i) + Square(v8.y * i));
            totalLength += l1 + l2 + l3 + l4 + l5 + l6 + l7 + l8;
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed_secs_length = end - start;
    }
    
    {
        // Time the "hypot" algorithm: hypot(x, y).
        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        start = std::chrono::high_resolution_clock::now();
        for (auto i = decltype(iterations){0}; i < iterations; ++i)
        {
            const auto l1 = std::hypot(v1.x * i, v1.y * i);
            const auto l2 = std::hypot(v2.x * i, v2.y * i);
            const auto l3 = std::hypot(v3.x * i, v3.y * i);
            const auto l4 = std::hypot(v4.x * i, v4.y * i);
            const auto l5 = std::hypot(v5.x * i, v5.y * i);
            const auto l6 = std::hypot(v6.x * i, v6.y * i);
            const auto l7 = std::hypot(v7.x * i, v7.y * i);
            const auto l8 = std::hypot(v8.x * i, v8.y * i);
            totalHypot += l1 + l2 + l3 + l4 + l5 + l6 + l7 + l8;
        }
        end = std::chrono::high_resolution_clock::now();
        elapsed_secs_hypot = end - start;
    }
    
    EXPECT_LT(elapsed_secs_length.count(), elapsed_secs_hypot.count());
    EXPECT_EQ(totalLength, totalHypot);
}
