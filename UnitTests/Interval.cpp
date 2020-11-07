/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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
#include <PlayRho/Common/Interval.hpp>
#include <limits>

class Interval: public ::testing::Test
{
protected:
    using type = playrho::Interval<int>;

    virtual void SetUp()
    {
        // add in lexicographical order from lowest to highest
        m_ranges.push_back(type{-87});
        m_ranges.push_back(type{-5});
        m_ranges.push_back(type{-5, 4});
        m_ranges.push_back(type{-5, 5});
        m_ranges.push_back(type{-5, 371});
        m_ranges.push_back(type{-5, 372});
        m_ranges.push_back(type{-4, 4});
        m_ranges.push_back(type{-4, 5});
        m_ranges.push_back(type{-4, 370});
        m_ranges.push_back(type{0});
        m_ranges.push_back(type{1});
        m_ranges.push_back(type{1, 2});
        m_ranges.push_back(type{2});
        m_ranges.push_back(type{ 4, 5});
        m_ranges.push_back(type{ 4, 57871});
        m_ranges.push_back(type{875});
    }
    
    std::vector<type> m_ranges;
};

TEST_F(Interval, Equality)
{
    for (auto& v: m_ranges)
    {
        // Establish reflexivity.
        // See: https://en.wikipedia.org/wiki/Reflexive_relation
        EXPECT_TRUE(v == v);
    }
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v == w || w == v)
            {
                // Establish symmetry.
                // See: https://en.wikipedia.org/wiki/Symmetric_relation
                EXPECT_TRUE(w == v && v == w);
            }
        }
    }
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                // Establish transivity.
                // See: https://en.wikipedia.org/wiki/Transitive_relation
                if (v == w && w == x)
                {
                    EXPECT_TRUE(v == x);
                }
            }
        }
    }
}

TEST_F(Interval, Inequality)
{
    for (auto& v: m_ranges)
    {
        // Establish reflexivity.
        // See: https://en.wikipedia.org/wiki/Reflexive_relation
        EXPECT_FALSE(v != v);
    }
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v == w || w == v)
            {
                // Establish symmetry.
                // See: https://en.wikipedia.org/wiki/Symmetric_relation
                EXPECT_FALSE(w != v || v != w);
            }
        }
    }
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                // Establish transivity.
                // See: https://en.wikipedia.org/wiki/Transitive_relation
                if (v == w && w == x)
                {
                    EXPECT_FALSE(v != x);
                }
            }
        }
    }
}

TEST_F(Interval, LessThan)
{
    // Establish irreflexivity.
    // See: https://en.wikipedia.org/wiki/Reflexive_relation
    for (auto& v: m_ranges)
    {
        EXPECT_FALSE(v < v);
    }

    // Establish asymmetry.
    // See: https://en.wikipedia.org/wiki/Symmetric_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v < w)
            {
                EXPECT_FALSE(w < v);
            }
        }
    }

    // Establish transivity.
    // See: https://en.wikipedia.org/wiki/Transitive_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                if (v < w && w < x)
                {
                    EXPECT_TRUE(v < x);
                }
                if (v < w)
                {
                    EXPECT_TRUE(v < x || x < w);
                }
            }
        }
    }
}

TEST_F(Interval, GreaterThan)
{
    // Establish irreflexivity.
    // See: https://en.wikipedia.org/wiki/Reflexive_relation
    for (auto& v: m_ranges)
    {
        EXPECT_FALSE(v > v);
    }
    
    // Establish asymmetry.
    // See: https://en.wikipedia.org/wiki/Symmetric_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v > w)
            {
                EXPECT_FALSE(w > v);
            }
        }
    }
    
    // Establish transivity.
    // See: https://en.wikipedia.org/wiki/Transitive_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                if (v > w && w > x)
                {
                    EXPECT_TRUE(v > x);
                }
                if (v > w)
                {
                    EXPECT_TRUE(v > x || x > w);
                }
            }
        }
    }
}

TEST_F(Interval, LessThanOrEqualTo)
{
    {
        type last = type{std::numeric_limits<int>::lowest()};
        for (auto& v: m_ranges)
        {
            //std::cout << v << "\n";
            EXPECT_LE(last, v);
            last = v;
        }
    }
    
    // Establish reflexivity.
    // See: https://en.wikipedia.org/wiki/Reflexive_relation
    for (auto& v: m_ranges)
    {
        EXPECT_TRUE(v <= v);
    }
    
    // Establish symmetry.
    // See: https://en.wikipedia.org/wiki/Symmetric_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v <= w)
            {
                EXPECT_TRUE(v != w || w <= v);
            }
        }
    }
    
    // Establish transivity.
    // See: https://en.wikipedia.org/wiki/Transitive_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                if (v <= w && w <= x)
                {
                    EXPECT_TRUE(v <= x);
                }
                if (v <= w)
                {
                    EXPECT_TRUE(v <= x || x <= w);
                }
            }
        }
    }
}

TEST_F(Interval, GreaterThanOrEqualTo)
{
    {
        type last = type{std::numeric_limits<int>::lowest()};
        for (auto& v: m_ranges)
        {
            //std::cout << v << "\n";
            EXPECT_GE(v, last);
            last = v;
        }
    }
    
    // Establish reflexivity.
    // See: https://en.wikipedia.org/wiki/Reflexive_relation
    for (auto& v: m_ranges)
    {
        EXPECT_TRUE(v >= v);
    }
    
    // Establish symmetry.
    // See: https://en.wikipedia.org/wiki/Symmetric_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            if (v >= w)
            {
                EXPECT_TRUE(v != w || w >= v);
            }
        }
    }
    
    // Establish transivity.
    // See: https://en.wikipedia.org/wiki/Transitive_relation
    for (auto& v: m_ranges)
    {
        for (auto& w: m_ranges)
        {
            for (auto& x: m_ranges)
            {
                if (v >= w && w >= x)
                {
                    EXPECT_TRUE(v >= x);
                }
                if (v >= w)
                {
                    EXPECT_TRUE(v >= x || x >= w);
                }
            }
        }
    }
}

TEST_F(Interval, GetLowest)
{
    EXPECT_EQ(playrho::Interval<int>::GetLowest(), std::numeric_limits<int>::lowest());
    EXPECT_EQ(playrho::Interval<float>::GetLowest(), -std::numeric_limits<float>::infinity());
}

TEST_F(Interval, GetHighest)
{
    EXPECT_EQ(playrho::Interval<int>::GetHighest(), std::numeric_limits<int>::max());
    EXPECT_EQ(playrho::Interval<float>::GetHighest(), std::numeric_limits<float>::infinity());
}

TEST_F(Interval, DefaultConstruction)
{
    EXPECT_EQ(playrho::Interval<int>{}, playrho::Interval<int>{});
    EXPECT_EQ(playrho::Interval<int>{}.GetMin(), std::numeric_limits<int>::max());
    EXPECT_EQ(playrho::Interval<int>{}.GetMax(), std::numeric_limits<int>::lowest());
    EXPECT_EQ(playrho::Interval<float>{}, playrho::Interval<float>{});
    EXPECT_EQ(playrho::Interval<float>{}.GetMin(),  std::numeric_limits<float>::infinity());
    EXPECT_EQ(playrho::Interval<float>{}.GetMax(), -std::numeric_limits<float>::infinity());
}
