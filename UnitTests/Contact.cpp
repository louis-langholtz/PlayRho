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

#include <PlayRho/Dynamics/Contacts/Contact.hpp>

using namespace playrho;
using namespace playrho::d2;

TEST(Contact, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
    switch (sizeof(Real))
    {
        case  4:
            EXPECT_EQ(alignof(Contact), 4u);
            EXPECT_EQ(sizeof(Contact), std::size_t(36));
            break;
        case  8:
            EXPECT_EQ(alignof(Contact), 8u);
            EXPECT_EQ(sizeof(Contact), std::size_t(56));
            break;
        case 16:
            EXPECT_EQ(sizeof(Contact), std::size_t(96));
            break;
        default:
            FAIL();
            break;
    }
}

TEST(Contact, Enabled)
{
    const auto bA = BodyID(0u);
    const auto bB = BodyID(1u);
    const auto fA = FixtureID(0u);
    const auto fB = FixtureID(1u);
    auto c = Contact{bA, fA, 0u, bB, fB, 0u};
    EXPECT_TRUE(c.IsEnabled());
    c.UnsetEnabled();
    EXPECT_FALSE(c.IsEnabled());
    c.SetEnabled();
    EXPECT_TRUE(c.IsEnabled());
}
