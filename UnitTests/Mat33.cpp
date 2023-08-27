/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Math.hpp>

using namespace playrho;

TEST(Mat33, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:  EXPECT_EQ(sizeof(Mat33), std::size_t(36)); break;
        case  8:  EXPECT_EQ(sizeof(Mat33), std::size_t(72)); break;
        case 16: EXPECT_EQ(sizeof(Mat33), std::size_t(144)); break;
        default: FAIL(); break;
    }
}

TEST(Mat33, Init)
{
    Vec3 c1{1, 1, 1};
    Vec3 c2{2, 2, 2};
    Vec3 c3{3, 3, 3};
    Mat33 foo{c1, c2, c3};
    EXPECT_EQ(c1, GetX(foo));
    EXPECT_EQ(c2, GetY(foo));
    EXPECT_EQ(c3, GetZ(foo));
}

TEST(Mat33, IsMatrix)
{
    ASSERT_FALSE(IsMatrix<int>::value);
    ASSERT_TRUE((IsMatrix<Matrix<int, 2, 3>>::value));
    EXPECT_TRUE(IsMatrix<Mat33>::value);
}

TEST(Mat33, IsSquareMatrix)
{
    ASSERT_FALSE(IsSquareMatrix<int>::value);
    ASSERT_FALSE((IsSquareMatrix<Matrix<int, 3, 2>>::value));
    EXPECT_TRUE(IsSquareMatrix<Mat33>::value);
}

TEST(Mat33, GetInverse)
{
    Vec3 c1{1, 1, 1};
    Vec3 c2{2, 2, 2};
    Vec3 c3{3, 3, 3};
    const Mat33 foo{c1, c2, c3};
    
    const auto a = GetX(GetX(foo)), b = GetX(GetY(foo)), c = GetY(GetX(foo)), d = GetY(GetY(foo));
    auto det = (a * d) - (b * c);
    if (det != Real{0})
    {
        det = Real{1} / det;
    }

    Mat33 boo{c1, c2, c3};
    boo = GetInverse22(foo);

    EXPECT_EQ(Real{0}, GetX(GetZ(boo)));
    EXPECT_EQ(Real{0}, GetY(GetZ(boo)));
    EXPECT_EQ(Real{0}, GetZ(GetZ(boo)));
    
    EXPECT_EQ(Real{0}, GetZ(GetY(boo)));
    EXPECT_EQ(Real{0}, GetZ(GetX(boo)));
    
    EXPECT_EQ(GetX(GetX(boo)), det * d);
    EXPECT_EQ(GetY(GetX(boo)), -det * c);
    EXPECT_EQ(GetX(GetY(boo)), -det * b);
    EXPECT_EQ(GetY(GetY(boo)), det * a);
}

TEST(Mat33, GetSymInverse33)
{
    Vec3 c1{1, 1, 1};
    Vec3 c2{2, 2, 2};
    Vec3 c3{3, 3, 3};
    const Mat33 foo{c1, c2, c3};
    
    auto det = Dot(GetX(foo), Cross(GetY(foo), GetZ(foo)));
    if (det != Real{0})
    {
        det = Real{1} / det;
    }
    
    const auto a11 = GetX(GetX(foo)), a12 = GetX(GetY(foo)), a13 = GetX(GetZ(foo));
    const auto a22 = GetY(GetY(foo)), a23 = GetY(GetZ(foo));
    const auto a33 = GetZ(GetZ(foo));
    const auto ex_y = det * (a13 * a23 - a12 * a33);
    const auto ex_z = det * (a12 * a23 - a13 * a22);
    const auto ey_z = det * (a13 * a12 - a11 * a23);

    Mat33 boo{c1, c2, c3};
    boo = GetSymInverse33(foo);

    EXPECT_EQ(GetX(GetX(boo)), det * (a22 * a33 - a23 * a23));
    EXPECT_EQ(GetY(GetX(boo)), ex_y);
    EXPECT_EQ(GetZ(GetX(boo)), ex_z);
    
    EXPECT_EQ(GetX(GetY(boo)), ex_y);
    EXPECT_EQ(GetY(GetY(boo)), det * (a11 * a33 - a13 * a13));
    EXPECT_EQ(GetZ(GetY(boo)), ey_z);
    
    EXPECT_EQ(GetX(GetZ(boo)), ex_z);
    EXPECT_EQ(GetY(GetZ(boo)), ey_z);
    EXPECT_EQ(GetZ(GetZ(boo)), det * (a11 * a22 - a12 * a12));
}
