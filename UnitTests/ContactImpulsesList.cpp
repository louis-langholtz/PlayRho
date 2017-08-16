/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "gtest/gtest.h"
#include <PlayRho/Dynamics/WorldCallbacks.hpp>

using namespace playrho;

TEST(ContactImpulsesList, ByteSize)
{
    switch (sizeof(Real))
    {
        case  4: EXPECT_EQ(sizeof(ContactImpulsesList), std::size_t(20)); break;
        case  8: EXPECT_EQ(sizeof(ContactImpulsesList), std::size_t(40)); break;
        case 16: EXPECT_EQ(sizeof(ContactImpulsesList), std::size_t(80)); break;
        default: FAIL(); break;
    }
}

TEST(ContactImpulsesList, DefaultConstruction)
{
    const auto v = ContactImpulsesList{};
    EXPECT_EQ(v.GetCount(), ContactImpulsesList::count_t(0));
}

TEST(ContactImpulsesList, AddEntry)
{
    auto v = ContactImpulsesList{};
    EXPECT_EQ(v.GetCount(), ContactImpulsesList::count_t(0));
    
    const auto normalMomentum = Momentum{Real(3) * Kilogram * MeterPerSecond};
    const auto tangentMomentum = Momentum{Real(1) * Kilogram * MeterPerSecond};

    v.AddEntry(normalMomentum, tangentMomentum);
    EXPECT_EQ(v.GetCount(), ContactImpulsesList::count_t(1));
    EXPECT_EQ(v.GetEntryNormal(0), normalMomentum);
    EXPECT_EQ(v.GetEntryTanget(0), tangentMomentum);
    
    v.AddEntry(normalMomentum * Real(2), tangentMomentum * Real(2));
    EXPECT_EQ(v.GetCount(), ContactImpulsesList::count_t(2));
    EXPECT_EQ(v.GetEntryNormal(0), normalMomentum);
    EXPECT_EQ(v.GetEntryTanget(0), tangentMomentum);
    EXPECT_EQ(v.GetEntryNormal(1), normalMomentum * Real(2));
    EXPECT_EQ(v.GetEntryTanget(1), tangentMomentum * Real(2));
}
