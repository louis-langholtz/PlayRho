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

#include <playrho/ContactFeature.hpp>

#include <sstream>

using namespace playrho;

TEST(ContactFeature, ByteSizeIs4)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    EXPECT_EQ(sizeof(ContactFeature), std::size_t(4));
}

TEST(ContactFeature, Init)
{
    const auto typeA = ContactFeature::e_vertex;
    const auto typeB = ContactFeature::e_face;
    const auto indexA = ContactFeature::Index{1};
    const auto indexB = ContactFeature::Index{2};
    ContactFeature foo{typeA, indexA, typeB, indexB};
    
    EXPECT_EQ(foo.typeA, typeA);
    EXPECT_EQ(foo.typeB, typeB);
    EXPECT_EQ(foo.indexA, indexA);
    EXPECT_EQ(foo.indexB, indexB);
}

TEST(ContactFeature, Flip)
{
    const auto typeA = ContactFeature::e_vertex;
    const auto typeB = ContactFeature::e_face;
    const auto indexA = ContactFeature::Index{1};
    const auto indexB = ContactFeature::Index{2};
    const auto foo = ContactFeature{typeA, indexA, typeB, indexB};
    const auto bar = Flip(foo);
    EXPECT_EQ(bar.typeA, typeB);
    EXPECT_EQ(bar.typeB, typeA);
    EXPECT_EQ(bar.indexA, indexB);
    EXPECT_EQ(bar.indexB, indexA);    
}

TEST(ContactFeature, Equals)
{
    const auto typeA = ContactFeature::e_vertex;
    const auto typeB = ContactFeature::e_face;
    const auto indexA = ContactFeature::Index{1};
    const auto indexB = ContactFeature::Index{2};
    const auto foo = ContactFeature{typeA, indexA, typeB, indexB};
    EXPECT_EQ(foo, foo);
}

TEST(ContactFeature, NotEquals)
{
    {
        const auto cf1 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
        const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 0};
        EXPECT_NE(cf1, cf2);
    }
    {
        const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
        const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
        EXPECT_NE(cf1, cf2);
    }
    {
        const auto cf1 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 0};
        const auto cf2 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 0};
        EXPECT_NE(cf1, cf2);
    }
    {
        const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 0};
        const auto cf2 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
        EXPECT_NE(cf1, cf2);
    }
    {
        const auto cf1 = ContactFeature{ContactFeature::e_vertex, 0, ContactFeature::e_face, 1};
        const auto cf2 = ContactFeature{ContactFeature::e_face, 0, ContactFeature::e_face, 1};
        EXPECT_NE(cf1, cf2);
    }
    {
        const auto cf1 = ContactFeature{ContactFeature::e_face, 1, ContactFeature::e_face, 1};
        const auto cf2 = ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_face, 1};
        EXPECT_NE(cf1, cf2);
    }
}

TEST(ContactFeature, GetName)
{
    EXPECT_STREQ(GetName(ContactFeature::e_face), "face");
    EXPECT_STREQ(GetName(ContactFeature::e_vertex), "vertex");
    EXPECT_STREQ(GetName(static_cast<ContactFeature::Type>(100)), "unknown");
}

TEST(ContactFeature, StreamOutput)
{
    std::ostringstream os;
    os << ContactFeature{ContactFeature::e_vertex, 1, ContactFeature::e_face, 2};
    const auto result = os.str();
    EXPECT_STREQ(result.c_str(), "{vertex,1,face,2}");
}
