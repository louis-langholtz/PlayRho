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
		const auto val = Vec2{float_t(3.9), float_t(-0.1)};
		EXPECT_EQ(Average<Vec2>({val}), val);
	}
	
	{
		const auto val1 = Vec2{float_t(2.2), float_t(-1.1)};
		const auto val2 = Vec2{float_t(4.4), float_t(-1.3)};
		const auto average = Average<Vec2>({val1, val2});
		const auto expected = Vec2(float_t(3.3), float_t(-1.2));
		EXPECT_FLOAT_EQ(average.x, expected.x);
		EXPECT_FLOAT_EQ(average.y, expected.y);
	}
}

TEST(Math, DotProductOfTwoVecTwoIsCommutative)
{
	const auto a = Vec2{float_t(-3.2), float_t(1.9)};
	const auto b = Vec2{float_t(4.01), float_t(-0.002)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, DotProductOfTwoVecThreeIsCommutative)
{
	const auto a = Vec3{float_t(-3.2), float_t(1.9), float_t(36.01)};
	const auto b = Vec3{float_t(4.01), float_t(-0.002), float_t(1.2)};
	EXPECT_EQ(Dot(a, b), Dot(b, a));
}

TEST(Math, CrossProductOfTwoVecTwoIsAntiCommutative)
{
	const auto a = Vec2{float_t(-3.2), float_t(1.9)};
	const auto b = Vec2{float_t(4.01), float_t(-0.002)};
	EXPECT_EQ(Cross(a, b), -Cross(b, a));
}

TEST(Math, Vec2NegationAndRotationIsOrderIndependent)
{
	{
		const auto v = Vec2{float_t(1), float_t(1)};
		const auto r = UnitVec2{0_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{float_t(1), float_t(1)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{float_t(-3.2), float_t(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
	}
	{
		const auto v = Vec2{float_t(-3.2), float_t(-21.4)};
		for (auto angle = -360_deg; angle < 360_deg; angle += 15_deg)
		{
			const auto r = UnitVec2{angle};
			EXPECT_EQ(Rotate(-v, r), -Rotate(v, r));
		}
	}
	{
		const auto v = Vec2{float_t(-3.2), float_t(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(v, r), -Rotate(-v, r));
	}
	{
		const auto v = Vec2{float_t(-3.2), float_t(1.9)};
		const auto r = UnitVec2{33_deg};
		EXPECT_EQ(Rotate(v, r), -Rotate(v, -r));
	}
}

TEST(Math, TransformIsRotatePlusTranslate)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};
	
	const auto transformed_vector = Transform(vector, transformation);
	const auto alt = Rotate(vector, rotation) + translation;
	
	EXPECT_EQ(transformed_vector.x, alt.x);
	EXPECT_EQ(transformed_vector.y, alt.y);
}

TEST(Math, InverseTransformIsUntranslateAndInverseRotate)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};
	
	const auto inv_vector = InverseTransform(vector, transformation);
	const auto alt = InverseRotate(vector - translation, rotation);
	
	EXPECT_EQ(inv_vector.x, alt.x);
	EXPECT_EQ(inv_vector.y, alt.y);
}

TEST(Math, InverseTransformTransformedIsOriginal)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};

	const auto transformed_vector = Transform(vector, transformation);
	const auto inverse_transformed_vector = InverseTransform(transformed_vector, transformation);

	EXPECT_FLOAT_EQ(vector.x, inverse_transformed_vector.x);
	EXPECT_FLOAT_EQ(vector.y, inverse_transformed_vector.y);
}

TEST(Math, TransformInverseTransformedIsOriginal)
{
	const auto vector = Vec2{float_t(19), float_t(-0.5)};
	const auto translation = Vec2{float_t(-3), float_t(+5)};
	const auto rotation = UnitVec2{90_deg};
	const auto transformation = Transformation{translation, rotation};

	const auto inverse_transformed_vector = InverseTransform(vector, transformation);
	const auto transformed_inverse_vector = Transform(inverse_transformed_vector, transformation);
	
	EXPECT_FLOAT_EQ(vector.x, transformed_inverse_vector.x);
	EXPECT_FLOAT_EQ(vector.y, transformed_inverse_vector.y);
}

TEST(Math, ComputeCentroidCenteredR1)
{
	const auto hx = float_t(1);
	const auto hy = float_t(1);
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
	const auto hx = float_t(1000);
	const auto hy = float_t(1000);
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
	const auto hx = float_t(1);
	const auto hy = float_t(1);
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
	const auto hx = float_t(100);
	const auto hy = float_t(100);
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
	const auto hx = float_t(0.1);
	const auto hy = float_t(0.1);
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
	const auto hx = float_t(1);
	const auto hy = float_t(1);
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
	const auto hx = float_t(1);
	const auto hy = float_t(1);
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
