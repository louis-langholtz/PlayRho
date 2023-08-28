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

#include <playrho/Interval.hpp>
#include <playrho/TypeInfo.hpp> // for TypeNameAsString
#include <playrho/Units.hpp>

using namespace playrho;

TEST(Interval, GetLowest)
{
    EXPECT_EQ(Interval<int>::GetLowest(), std::numeric_limits<int>::lowest());
    EXPECT_EQ(Interval<float>::GetLowest(), -std::numeric_limits<float>::infinity());
    EXPECT_EQ(Interval<Length>::GetLowest(), -std::numeric_limits<Length>::infinity());
}

TEST(Interval, GetHighest)
{
    EXPECT_EQ(Interval<int>::GetHighest(), std::numeric_limits<int>::max());
    EXPECT_EQ(Interval<float>::GetHighest(), std::numeric_limits<float>::infinity());
    EXPECT_EQ(Interval<Length>::GetHighest(), std::numeric_limits<Length>::infinity());
}

template <class T>
auto DefaultConstructionChecks() -> void
{
    SCOPED_TRACE(detail::TypeNameAsString<T>() + " type...");
    EXPECT_EQ(Interval<T>{}, Interval<T>{});
    EXPECT_EQ(Interval<T>{}.GetMin(), Interval<T>::GetHighest());
    EXPECT_EQ(Interval<T>{}.GetMax(), Interval<T>::GetLowest());
}

TEST(Interval, DefaultConstruction)
{
    DefaultConstructionChecks<int>();
    DefaultConstructionChecks<unsigned>();
    DefaultConstructionChecks<float>();
    DefaultConstructionChecks<double>();
    DefaultConstructionChecks<Length>();
}

TEST(Interval, MoveInvalidDoesNothing)
{
    EXPECT_EQ(Interval<int>().Move(3), Interval<int>());
    EXPECT_EQ(Interval<float>().Move(3), Interval<float>());
}

TEST(Interval, ExpandInvalidDoesNothing)
{
    EXPECT_EQ(Interval<int>().Expand(3), Interval<int>());
    EXPECT_EQ(Interval<float>().Expand(3), Interval<float>());
}

TEST(Interval, ExpandEquallyInvalidDoesNothing)
{
    EXPECT_EQ(Interval<int>().ExpandEqually(3), Interval<int>());
    EXPECT_EQ(Interval<float>().ExpandEqually(3), Interval<float>());
}

TEST(Interval, IncludeValueInvalid)
{
    constexpr auto v = 42;
    EXPECT_EQ(Interval<int>().Include(v), Interval<int>(v));
    EXPECT_EQ(Interval<int>(v).Include(Interval<int>()), Interval<int>(v));
    EXPECT_EQ(Interval<float>().Include(v), Interval<float>(v));
    EXPECT_EQ(Interval<float>(v).Include(Interval<float>()), Interval<float>(v));
}

TEST(Interval, IncludeIntervalInvalid)
{
    {
        constexpr auto v = Interval<int>(42);
        EXPECT_EQ(Interval<int>().Include(v), Interval<int>(v));
        EXPECT_EQ(Interval<int>(v).Include(Interval<int>()), Interval<int>(v));
    }
    {
        constexpr auto v = Interval<float>(8.0f);
        EXPECT_EQ(Interval<float>().Include(v), Interval<float>(v));
        EXPECT_EQ(Interval<float>(v).Include(Interval<float>()), Interval<float>(v));
    }
}

TEST(Interval, IntersectWithInvalidAlwaysInvalid)
{
    constexpr auto v = 42;
    EXPECT_EQ(Interval<int>().Intersect(Interval<int>()), Interval<int>());
    EXPECT_EQ(Interval<int>().Intersect(Interval<int>(v)), Interval<int>());
    EXPECT_EQ(Interval<int>(v).Intersect(Interval<int>()), Interval<int>());
    EXPECT_EQ(Interval<float>().Intersect(Interval<float>()), Interval<float>());
    EXPECT_EQ(Interval<float>().Intersect(Interval<float>(v)), Interval<float>());
    EXPECT_EQ(Interval<float>(v).Intersect(Interval<float>()), Interval<float>());
}

TEST(Interval, GetSize)
{
    EXPECT_EQ(GetSize(Interval<int>(0)), 0);
    EXPECT_EQ(GetSize(Interval<int>(-1, +1)), 2);
    EXPECT_EQ(GetSize(Interval<int>(0, 4)), 4);
    EXPECT_EQ(GetSize(Interval<float>(0)), 0.0f);
    EXPECT_EQ(GetSize(Interval<float>(-1, +1)), 2.0f);
    EXPECT_EQ(GetSize(Interval<float>()), -std::numeric_limits<float>::infinity());
    EXPECT_EQ(GetSize(Interval<float>(Interval<float>::GetLowest(), Interval<float>::GetHighest())),
              +std::numeric_limits<float>::infinity());
}

TEST(Interval, GetCenter)
{
    EXPECT_EQ(GetCenter(Interval<int>(0)), 0);
    EXPECT_EQ(GetCenter(Interval<int>(-1, +1)), 0);
    EXPECT_EQ(GetCenter(Interval<int>(0, 4)), 2);
    EXPECT_EQ(GetCenter(Interval<float>(0)), 0.0f);
    EXPECT_EQ(GetCenter(Interval<float>(-1, +1)), 0.0f);
    EXPECT_EQ(GetCenter(Interval<float>(0, 4)), 2.0f);
    EXPECT_TRUE(std::isnan(GetCenter(Interval<float>())));
    EXPECT_TRUE(std::isnan(GetCenter(Interval<float>(Interval<float>::GetLowest(), Interval<float>::GetHighest()))));
}

class IntervalFixture: public ::testing::Test
{
protected:
    using type = Interval<int>;

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

TEST_F(IntervalFixture, Equality)
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

TEST_F(IntervalFixture, Inequality)
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

TEST_F(IntervalFixture, LessThan)
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

TEST_F(IntervalFixture, GreaterThan)
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

TEST_F(IntervalFixture, LessThanOrEqualTo)
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

TEST_F(IntervalFixture, GreaterThanOrEqualTo)
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
