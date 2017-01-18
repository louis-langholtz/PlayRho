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
#include <Box2D/Common/Math.hpp>

using namespace box2d;

TEST(Math, Sqrt)
{
	EXPECT_EQ(Sqrt(0.0), 0.0);
	EXPECT_NE(Sqrt(std::numeric_limits<float>::min()), float(0));
	EXPECT_NE(Sqrt(std::numeric_limits<double>::min()), double(0));
	EXPECT_EQ(Square(Sqrt(std::numeric_limits<double>::min())), std::numeric_limits<double>::min());
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

	EXPECT_EQ(high, float(2.646978275714050648e-23));

	ASSERT_NE(Square(high), float(0));
	ASSERT_EQ(Sqrt(Square(float(1))), float(1));

	std::cout << "Sqrt(min) is: ";
	std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << Sqrt(std::numeric_limits<float>::min());
	std::cout << " aka ";
	std::cout << std::hexfloat;
	std::cout << Sqrt(std::numeric_limits<float>::min());
	std::cout << std::endl;
	EXPECT_EQ(Sqrt(std::numeric_limits<float>::min()), float(0x1p-63)); // float(1.084202172485504434e-19)
	
	// What is the smallest float a for which:
	// almost_equal(sqrt(square(a)), a) and almost_equal(square(sqrt(a)), a)
	// hold true?
	
	const auto a = Sqrt(std::numeric_limits<float>::min());
	EXPECT_TRUE(almost_equal(Square(Sqrt(a)), a));
	EXPECT_TRUE(almost_equal(Sqrt(Square(a)), a));
}

TEST(Math, Atan2)
{
	EXPECT_EQ(Atan2(0, 0), 0);
	EXPECT_EQ(Atan2(0.0, 0.0), 0.0);
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
		const auto val = Vec2{realnum(3.9), realnum(-0.1)};
		EXPECT_EQ(Average<Vec2>({val}), val);
	}
	
	{
		const auto val1 = Vec2{realnum(2.2), realnum(-1.1)};
		const auto val2 = Vec2{realnum(4.4), realnum(-1.3)};
		const auto average = Average<Vec2>({val1, val2});
		const auto expected = Vec2(realnum(3.3), realnum(-1.2));
		EXPECT_FLOAT_EQ(average.x, expected.x);
		EXPECT_FLOAT_EQ(average.y, expected.y);
	}
}

TEST(Math, DotProductOfTwoVecTwoIsCommutative)
{
	const auto a = Vec2{realnum(-3.2), realnum(1.9)};
	const auto b = Vec2{realnum(4.01), realnum(-0.002)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, DotProductOfTwoVecThreeIsCommutative)
{
	const auto a = Vec3{realnum(-3.2), realnum(1.9), realnum(36.01)};
	const auto b = Vec3{realnum(4.01), realnum(-0.002), realnum(1.2)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, CrossProductOfTwoVecTwoIsAntiCommutative)
{
	const auto a = Vec2{realnum(-3.2), realnum(1.9)};
	const auto b = Vec2{realnum(4.01), realnum(-0.002)};
	EXPECT_EQ(Cross(a, b), -Cross(b, a));
}

TEST(Math, Vec2NegationAndRotationIsOrderIndependent)
{
	{
		const auto v = Vec2{realnum(1), realnum(1)};
		const auto r = UnitVec2{0_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{realnum(1), realnum(1)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{realnum(-3.2), realnum(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{realnum(-3.2), realnum(-21.4)};
		for (auto angle = -360_deg; angle < 360_deg; angle += 15_deg)
		{
			const auto r = UnitVec2{angle};
			EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
		}
	}
	{
		const auto v = Vec2{realnum(-3.2), realnum(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(v, r), -Rotate(-v, r));
	}
	{
		const auto v = Vec2{realnum(-3.2), realnum(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(v, r), -Rotate(v, -r));
	}
}

TEST(Math, TransformIsRotatePlusTranslate)
{
	const auto vector = Vec2{realnum(19), realnum(-0.5)};
	const auto translation = Vec2{realnum(-3), realnum(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};
	
	const auto transformed_vector = Transform(vector, transformation);
	const auto alt = Rotate(vector, rotation) + translation;
	
	EXPECT_EQ(transformed_vector.x, alt.x);
	EXPECT_EQ(transformed_vector.y, alt.y);
}

TEST(Math, InverseTransformIsUntranslateAndInverseRotate)
{
	const auto vector = Vec2{realnum(19), realnum(-0.5)};
	const auto translation = Vec2{realnum(-3), realnum(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};
	
	const auto inv_vector = InverseTransform(vector, transformation);
	const auto alt = InverseRotate(vector - translation, rotation);
	
	EXPECT_EQ(inv_vector.x, alt.x);
	EXPECT_EQ(inv_vector.y, alt.y);
}

TEST(Math, InverseTransformTransformedIsOriginal)
{
	const auto vector = Vec2{realnum(19), realnum(-0.5)};
	const auto translation = Vec2{realnum(-3), realnum(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};

	const auto transformed_vector = Transform(vector, transformation);
	const auto inverse_transformed_vector = InverseTransform(transformed_vector, transformation);

	EXPECT_FLOAT_EQ(vector.x, inverse_transformed_vector.x);
	EXPECT_FLOAT_EQ(vector.y, inverse_transformed_vector.y);
}

TEST(Math, TransformInverseTransformedIsOriginal)
{
	const auto vector = Vec2{realnum(19), realnum(-0.5)};
	const auto translation = Vec2{realnum(-3), realnum(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};

	const auto inverse_transformed_vector = InverseTransform(vector, transformation);
	const auto transformed_inverse_vector = Transform(inverse_transformed_vector, transformation);
	
	EXPECT_FLOAT_EQ(vector.x, transformed_inverse_vector.x);
	EXPECT_FLOAT_EQ(vector.y, transformed_inverse_vector.y);
}

TEST(Math, ComputeCentroidCenteredR1)
{
	const auto hx = realnum(1);
	const auto hy = realnum(1);
	const auto real_center = Vec2{0, 0};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidCentered0R1000)
{
	const auto hx = realnum(1000);
	const auto hy = realnum(1000);
	const auto real_center = Vec2{0, 0};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight1000R1)
{
	const auto hx = realnum(1);
	const auto hy = realnum(1);
	const auto real_center = Vec2{1000, 1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight1000R100)
{
	const auto hx = realnum(100);
	const auto hy = realnum(100);
	const auto real_center = Vec2{1000, 1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidUpRight10000R01)
{
	const auto hx = realnum(0.1);
	const auto hy = realnum(0.1);
	const auto real_center = Vec2{10000, 10000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidDownLeft1000R1)
{
	const auto hx = realnum(1);
	const auto hy = realnum(1);
	const auto real_center = Vec2{-1000, -1000};
	const auto vertices = {
		Vec2{real_center.x + hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y + hy},
		Vec2{real_center.x - hx, real_center.y - hy},
		Vec2{real_center.x + hx, real_center.y - hy}
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, ComputeCentroidOfHexagonalVertices)
{
	const auto hx = realnum(1);
	const auto hy = realnum(1);
	const auto real_center = Vec2{-1000, -1000};
	const auto vertices = {
		Vec2{real_center.x + 00, real_center.y + 2 * hy},
		Vec2{real_center.x - hx, real_center.y + 1 * hy},
		Vec2{real_center.x - hx, real_center.y - 1 * hy},
		Vec2{real_center.x + 00, real_center.y - 2 * hy},
		Vec2{real_center.x + hx, real_center.y - 1 * hy},
		Vec2{real_center.x + hx, real_center.y + 1 * hy},
	};
	const auto center = ComputeCentroid(vertices);
	EXPECT_EQ(center.x, real_center.x);
	EXPECT_EQ(center.y, real_center.y);
	
	const auto average = Average<Vec2>(vertices);
	EXPECT_EQ(average.x, center.x);
	EXPECT_EQ(average.y, center.y);
}

TEST(Math, GetContactRelVelocity)
{
	const auto velA = Velocity{Vec2(+1, +4), 3.2_rad};
	const auto velB = Velocity{Vec2(+3, +1), 0.4_rad};
	const auto relA = Vec2(0, 0);
	const auto relB = Vec2(0, 0);
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

	constexpr auto max = std::numeric_limits<uint32>::max() / 512;
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

TEST(Math, BiggerFloatsIncreasinglyInaccurate)
{
	// This test is meant to demonstrate the increasing inaccuracy of the float type and help
	// recognize the problems that using this type can cause. Note that the double suffers the
	// same way except more slowly. This increasing inaccuracy is inherent to how floating point
	// types are designed.
	//
	// A way to avoid this problem, is to use fixed-point calculations (instead of floating-point
	// calculations).

	auto last_delta = float(0);
	auto val = float(1);
	for (auto i = 0; i < 20; ++i)
	{
		const auto next = std::nextafter(val, MaxFloat);
		const auto delta = next - val;
		ASSERT_EQ(val + (delta / 2), val);

		// For  0x1p+0, delta of next value is 0x1p-23: ie. at      1, delta is 0.0000001192092895508
		// For  0x1p+1, delta of next value is 0x1p-22: ie. at      2, delta is 0.0000002384185791016
		// For  0x1p+2, delta of next value is 0x1p-21: ie. at      4, delta is 0.0000004768371582031
		// For  0x1p+3, delta of next value is 0x1p-20: ie. at      8, delta is 0.0000009536743164062
		// For  0x1p+4, delta of next value is 0x1p-19: ie. at     16, delta is 0.0000019073486328125
		// For  0x1p+5, delta of next value is 0x1p-18: ie. at     32, delta is 0.0000038146972656250
		// For  0x1p+6, delta of next value is 0x1p-17: ie. at     64, delta is 0.0000076293945312500
		// For  0x1p+7, delta of next value is 0x1p-16: ie. at    128, delta is 0.0000152587890625000
		// For  0x1p+8, delta of next value is 0x1p-15: ie. at    256, delta is 0.0000305175781250000
		// For  0x1p+9, delta of next value is 0x1p-14: ie. at    512, delta is 0.0000610351562500000
		// For 0x1p+10, delta of next value is 0x1p-13: ie. at   1024, delta is 0.0001220703125000000
		// For 0x1p+11, delta of next value is 0x1p-12: ie. at   2048, delta is 0.0002441406250000000
		// For 0x1p+12, delta of next value is 0x1p-11: ie. at   4096, delta is 0.0004882812500000000
		// For 0x1p+13, delta of next value is 0x1p-10: ie. at   8192, delta is 0.0009765625000000000
		// For 0x1p+14, delta of next value is  0x1p-9: ie. at  16384, delta is 0.0019531250000000000
		// For 0x1p+15, delta of next value is  0x1p-8: ie. at  32768, delta is 0.0039062500000000000
		// For 0x1p+16, delta of next value is  0x1p-7: ie. at  65536, delta is 0.0078125000000000000
		// For 0x1p+17, delta of next value is  0x1p-6: ie. at 131072, delta is 0.0156250000000000000
		// For 0x1p+18, delta of next value is  0x1p-5: ie. at 262144, delta is 0.0312500000000000000
		// For 0x1p+19, delta of next value is  0x1p-4: ie. at 524288, delta is 0.0625000000000000000
		//
		// If a floating-point type is used in the implementation of the simulation then, these
		// deltas mean that:
		// - The farther bodies get out from the origin (0, 0) the less accurately they can be moved.
		// - The larger shape vertex radiuses get, the less accurately time of impact can be
		//   calculated for those shapes.
#if 0
		std::cout << std::hexfloat;
		std::cout << "For " << std::setw(7) << val << ", delta of next value is " << std::setw(7) << delta;
		std::cout << std::defaultfloat;
		std::cout << ": ie. at " << std::setw(6) << val;
		std::cout << std::fixed;
		std::cout << ", delta is " << delta;
		std::cout << std::endl;
#endif
		val *= 2;
		EXPECT_GT(delta, last_delta);
		last_delta = delta;
	}
}

TEST(Math, ToiTolerance)
{
	// What is the max vr for which the following still holds true?
	//   vr + LinearSlop / 4 > vr
	// The max vr for which (std::nextafter(vr, MaxFloat) - vr) <= LinearSlop / 4.
	// I.e. the max vr for which (std::nextafter(vr, MaxFloat) - vr) <= 0.000025

	const auto tolerance = LinearSlop / 4;
	{
		const auto vr = 511.0f;
		EXPECT_GT(vr + tolerance, vr);
	}
	{
		const auto vr = 512.0f;
		EXPECT_EQ(vr + tolerance, vr);
	}
}
