/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Common/Math.hpp>

using namespace playrho;

TEST(Mat33, ByteSizeIs_36_72_or_144)
{
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
    EXPECT_EQ(c1, foo.ex);
    EXPECT_EQ(c2, foo.ey);
    EXPECT_EQ(c3, foo.ez);
}

TEST(Mat33, GetInverse)
{
    Vec3 c1{1, 1, 1};
    Vec3 c2{2, 2, 2};
    Vec3 c3{3, 3, 3};
    const Mat33 foo{c1, c2, c3};
    
    const auto a = GetX(foo.ex), b = GetX(foo.ey), c = GetY(foo.ex), d = GetY(foo.ey);
    auto det = (a * d) - (b * c);
    if (det != Real{0})
    {
        det = Real{1} / det;
    }

    Mat33 boo{c1, c2, c3};
    boo = GetInverse22(foo);

    EXPECT_EQ(Real{0}, GetX(boo.ez));
    EXPECT_EQ(Real{0}, GetY(boo.ez));
    EXPECT_EQ(Real{0}, GetZ(boo.ez));
    
    EXPECT_EQ(Real{0}, GetZ(boo.ey));
    EXPECT_EQ(Real{0}, GetZ(boo.ex));
    
    EXPECT_EQ(GetX(boo.ex), det * d);
    EXPECT_EQ(GetY(boo.ex), -det * c);
    EXPECT_EQ(GetX(boo.ey), -det * b);
    EXPECT_EQ(GetY(boo.ey), det * a);
}

TEST(Mat33, GetSymInverse33)
{
    Vec3 c1{1, 1, 1};
    Vec3 c2{2, 2, 2};
    Vec3 c3{3, 3, 3};
    const Mat33 foo{c1, c2, c3};
    
    auto det = Dot(foo.ex, Cross(foo.ey, foo.ez));
    if (det != Real{0})
    {
        det = Real{1} / det;
    }
    
    const auto a11 = GetX(foo.ex), a12 = GetX(foo.ey), a13 = GetX(foo.ez);
    const auto a22 = GetY(foo.ey), a23 = GetY(foo.ez);
    const auto a33 = GetZ(foo.ez);
    const auto ex_y = det * (a13 * a23 - a12 * a33);
    const auto ex_z = det * (a12 * a23 - a13 * a22);
    const auto ey_z = det * (a13 * a12 - a11 * a23);

    Mat33 boo{c1, c2, c3};
    boo = GetSymInverse33(foo);

    EXPECT_EQ(GetX(boo.ex), det * (a22 * a33 - a23 * a23));
    EXPECT_EQ(GetY(boo.ex), ex_y);
    EXPECT_EQ(GetZ(boo.ex), ex_z);
    
    EXPECT_EQ(GetX(boo.ey), ex_y);
    EXPECT_EQ(GetY(boo.ey), det * (a11 * a33 - a13 * a13));
    EXPECT_EQ(GetZ(boo.ey), ey_z);
    
    EXPECT_EQ(GetX(boo.ez), ex_z);
    EXPECT_EQ(GetY(boo.ez), ey_z);
    EXPECT_EQ(GetZ(boo.ez), det * (a11 * a22 - a12 * a12));
}
