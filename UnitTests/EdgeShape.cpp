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

#include <PlayRho/Collision/Shapes/EdgeShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(EdgeShapeConf, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real)) {
    case 4:
        EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(56));
        break;
    case 8:
        EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(104));
        break;
    case 16:
        EXPECT_EQ(sizeof(EdgeShapeConf), std::size_t(208));
        break;
    default:
        FAIL();
        break;
    }
}

TEST(EdgeShapeConf, IsValidShapeType)
{
    EXPECT_TRUE(IsValidShapeType<EdgeShapeConf>::value);
}

TEST(EdgeShapeConf, GetInvalidChildThrows)
{
    const auto foo = EdgeShapeConf{};

    ASSERT_EQ(GetChildCount(foo), ChildCounter{1});
    EXPECT_NO_THROW(GetChild(foo, 0));
    EXPECT_THROW(GetChild(foo, 1), InvalidArgument);
}

TEST(EdgeShapeConf, TranslateFF)
{
    {
        auto foo = EdgeShapeConf{};
        auto tmp = foo;
        Translate(foo, Length2{});
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = EdgeShapeConf{v1, v2};
        auto tmp = foo;
        const auto value = Length2{1_m, 2_m};
        Translate(foo, value);
        EXPECT_NE(foo, tmp);
        EXPECT_EQ(foo.GetVertexA(), v1 + value);
        EXPECT_EQ(foo.GetVertexB(), v2 + value);
    }
}

TEST(EdgeShapeConf, ScaleFF)
{
    {
        auto foo = EdgeShapeConf{};
        auto tmp = foo;
        Scale(foo, Vec2{Real(1), Real(1)});
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = EdgeShapeConf{v1, v2};
        auto tmp = foo;
        const auto value = Vec2{Real(2), Real(4)};
        EXPECT_NO_THROW(Scale(foo, value));
        EXPECT_NE(foo, tmp);
        EXPECT_EQ(foo.GetVertexA(), Length2(GetX(v1) * GetX(value), GetY(v1) * GetY(value)));
        EXPECT_EQ(foo.GetVertexB(), Length2(GetX(v2) * GetX(value), GetY(v2) * GetY(value)));
    }
}

TEST(EdgeShapeConf, RotateFF)
{
    {
        auto foo = EdgeShapeConf{};
        auto tmp = foo;
        Rotate(foo, UnitVec::GetRight());
        EXPECT_EQ(foo, tmp);
    }
    {
        const auto v1 = Length2{1_m, 2_m};
        const auto v2 = Length2{3_m, 4_m};
        auto foo = EdgeShapeConf{v1, v2};
        auto tmp = foo;
        const auto value = UnitVec::GetTop();
        EXPECT_NO_THROW(Rotate(foo, value));
        EXPECT_NE(foo, tmp);
        EXPECT_EQ(foo.GetVertexA(), Rotate(v1, value));
        EXPECT_EQ(foo.GetVertexB(), Rotate(v2, value));
    }
}

TEST(EdgeShapeConf, TypeInfo)
{
    const auto foo = EdgeShapeConf{};
    const auto shape = Shape(foo);
    EXPECT_EQ(GetType(shape), GetTypeID<EdgeShapeConf>());
    auto copy = EdgeShapeConf{};
    EXPECT_NO_THROW(copy = TypeCast<EdgeShapeConf>(shape));
    EXPECT_THROW(TypeCast<int>(shape), std::bad_cast);
}

TEST(EdgeShapeConf, Equality)
{
    EXPECT_TRUE(EdgeShapeConf() == EdgeShapeConf());

    EXPECT_FALSE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) ==
                EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)));

    EXPECT_FALSE(EdgeShapeConf().UseVertexRadius(10_m) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseVertexRadius(10_m) == EdgeShapeConf().UseVertexRadius(10_m));

    EXPECT_FALSE(EdgeShapeConf().UseDensity(10_kgpm2) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseDensity(10_kgpm2) == EdgeShapeConf().UseDensity(10_kgpm2));

    EXPECT_FALSE(EdgeShapeConf().UseFriction(Real(10)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseFriction(Real(10)) == EdgeShapeConf().UseFriction(Real(10)));

    EXPECT_FALSE(EdgeShapeConf().UseRestitution(Real(10)) == EdgeShapeConf());
    EXPECT_TRUE(EdgeShapeConf().UseRestitution(Real(10)) ==
                EdgeShapeConf().UseRestitution(Real(10)));
}

TEST(EdgeShapeConf, Inequality)
{
    EXPECT_FALSE(EdgeShapeConf() != EdgeShapeConf());

    EXPECT_TRUE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)) !=
                 EdgeShapeConf().Set(Length2(1_m, 2_m), Length2(3_m, 4_m)));

    EXPECT_TRUE(EdgeShapeConf().UseVertexRadius(10_m) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseVertexRadius(10_m) != EdgeShapeConf().UseVertexRadius(10_m));

    EXPECT_TRUE(EdgeShapeConf().UseDensity(10_kgpm2) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseDensity(10_kgpm2) != EdgeShapeConf().UseDensity(10_kgpm2));

    EXPECT_TRUE(EdgeShapeConf().UseFriction(Real(10)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseFriction(Real(10)) != EdgeShapeConf().UseFriction(Real(10)));

    EXPECT_TRUE(EdgeShapeConf().UseRestitution(Real(10)) != EdgeShapeConf());
    EXPECT_FALSE(EdgeShapeConf().UseRestitution(Real(10)) !=
                 EdgeShapeConf().UseRestitution(Real(10)));
}
