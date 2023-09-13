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

#include <playrho/NonNegative.hpp>
#include <playrho/NonPositive.hpp>
#include <playrho/NonZero.hpp>
#include <playrho/UnitInterval.hpp>
#include <playrho/Finite.hpp>
#include <playrho/Positive.hpp>
#include <playrho/Negative.hpp>

#include <limits>
#include <cmath>
#include <type_traits>

using namespace playrho;

TEST(CheckedValue, NonNegativeFloatTraits)
{
    using type = NonNegative<float>;

    EXPECT_TRUE(std::is_default_constructible<type>::value);
    EXPECT_FALSE(std::is_nothrow_default_constructible<type>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<type>::value);

    EXPECT_TRUE((std::is_constructible<type, type::value_type>::value));
    EXPECT_FALSE((std::is_nothrow_constructible<type, type::value_type>::value));
    EXPECT_FALSE((std::is_trivially_constructible<type, type::value_type>::value));

    EXPECT_TRUE(std::is_copy_constructible<type>::value);
    EXPECT_TRUE(std::is_nothrow_copy_constructible<type>::value);
    EXPECT_TRUE(std::is_trivially_copy_constructible<type>::value);

    EXPECT_TRUE(std::is_copy_assignable<type>::value);
    EXPECT_TRUE(std::is_nothrow_copy_assignable<type>::value);
    EXPECT_TRUE(std::is_trivially_copy_assignable<type>::value);

    EXPECT_TRUE(std::is_destructible<type>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<type>::value);
    EXPECT_TRUE(std::is_trivially_destructible<type>::value);

    EXPECT_TRUE((std::is_convertible<type, type::value_type>::value));
    EXPECT_TRUE((std::is_convertible<type::value_type, type>::value));
}

#ifndef NDEBUG
TEST(CheckedValue_DeathTest, NonNegativeFfTerminates)
{
    using type = Checked<float, NonNegativeChecker<float>, true>;
    constexpr auto exit_value = 42;
    constexpr auto exit_message = "terminate called for invalid_argument\n";
    type foo;
    EXPECT_EXIT({
        std::set_terminate([](){
            const auto ex = std::current_exception();
            try {
                if (ex) {
                    std::rethrow_exception(ex);
                }
                std::cerr << "terminate called without exception\n";
            }
            catch (const std::invalid_argument& ex) {
                std::cerr << exit_message;
            }
            catch (...) {
                std::cerr << "terminate called for unexpected exception\n";
            }
            std::cerr << std::flush;
            _exit(exit_value);
        });
        foo = -2.0f;
        std::exit(0);
    }, testing::ExitedWithCode(exit_value), exit_message);
}
#endif

TEST(CheckedValue, NegativeFloat)
{
    EXPECT_EQ(float{Negative<float>(-1.0f)}, -1.0f);
    EXPECT_EQ(float(Negative<float>(-1.0f)), float(Negative<float>(-1.0f)));
    EXPECT_EQ(float(Negative<float>(-std::numeric_limits<float>::infinity())),
              -std::numeric_limits<float>::infinity());

    EXPECT_THROW(Negative<float>(-0.0f), Negative<float>::exception_type);
    EXPECT_THROW(Negative<float>{+0.00001f}, Negative<float>::exception_type);
    EXPECT_THROW(Negative<float>{+1.4f}, Negative<float>::exception_type);
    EXPECT_THROW(Negative<float>{+std::numeric_limits<float>::infinity()},
                 Negative<float>::exception_type);
    EXPECT_THROW(Negative<float>{std::numeric_limits<float>::quiet_NaN()},
                 Negative<float>::exception_type);

    {
        auto os = std::ostringstream{};
        os << Negative<float>(-1.0f);
        EXPECT_STREQ(os.str().c_str(), "-1");
    }
}

TEST(CheckedValue, NonNegativeFloat)
{
    EXPECT_EQ(float{NonNegative<float>(1.0f)}, 1.0f);
    EXPECT_EQ(float(NonNegative<float>(1.0f)), float(NonNegative<float>(1.0f)));
    EXPECT_EQ(float(NonNegative<float>(0.0f)), 0.0f);
    EXPECT_EQ(float(NonNegative<float>(std::numeric_limits<float>::infinity())),
              std::numeric_limits<float>::infinity());

    EXPECT_THROW(NonNegative<float>{-0.00001f}, NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{-1.4f}, NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{-std::numeric_limits<float>::infinity()},
                 NonNegative<float>::exception_type);
    EXPECT_THROW(NonNegative<float>{std::numeric_limits<float>::quiet_NaN()},
                 NonNegative<float>::exception_type);
}

TEST(CheckedValue, NonNegativeDouble)
{
    EXPECT_EQ(double{NonNegative<double>(1.0f)}, 1.0f);
    EXPECT_EQ(double(NonNegative<double>(1.0f)), double(NonNegative<double>(1.0f)));
    EXPECT_EQ(double(NonNegative<double>(0.0f)), 0.0f);
    EXPECT_EQ(double(NonNegative<double>(std::numeric_limits<double>::infinity())),
              std::numeric_limits<double>::infinity());

    EXPECT_THROW(NonNegative<double>{-0.00001f}, NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{-1.4f}, NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{-std::numeric_limits<double>::infinity()},
                 NonNegative<double>::exception_type);
    EXPECT_THROW(NonNegative<double>{std::numeric_limits<double>::quiet_NaN()},
                 NonNegative<double>::exception_type);
}

TEST(CheckedValue, NonNegativeInt)
{
    EXPECT_EQ(int{NonNegative<int>(1)}, 1);
    EXPECT_EQ(int(NonNegative<int>(1)), int(NonNegative<int>(1)));
    EXPECT_EQ(int(NonNegative<int>(0)), 0);

    EXPECT_THROW(NonNegative<int>{-1}, NonNegative<int>::exception_type);
    EXPECT_THROW(NonNegative<int>{-2}, NonNegative<int>::exception_type);

    {
        auto os = std::ostringstream{};
        os << NonNegative<int>(2);
        EXPECT_STREQ(os.str().c_str(), "2");
    }
}

TEST(CheckedValue, PositiveFloat)
{
    EXPECT_EQ(float(Positive<float>(+1.0f)), +1.0f);
    EXPECT_EQ(float(Positive<float>(+1.0f)), float(Positive<float>(+1.0f)));
    EXPECT_EQ(float(Positive<float>(+std::numeric_limits<float>::infinity())),
              +std::numeric_limits<float>::infinity());

    EXPECT_THROW(Positive<float>(+0.0f), Positive<float>::exception_type);
    EXPECT_THROW(Positive<float>{-0.00001f}, Positive<float>::exception_type);
    EXPECT_THROW(Positive<float>{-1.4f}, Positive<float>::exception_type);
    EXPECT_THROW(Positive<float>{-std::numeric_limits<float>::infinity()},
                 Positive<float>::exception_type);
    EXPECT_THROW(Positive<float>{std::numeric_limits<float>::quiet_NaN()},
                 Positive<float>::exception_type);

    {
        auto os = std::ostringstream{};
        os << Positive<float>(1.0f);
        EXPECT_STREQ(os.str().c_str(), "1");
    }
}

TEST(CheckedValue, NonPositiveFloat)
{
    EXPECT_EQ(float(NonPositive<float>(-1.0f)), -1.0f);
    EXPECT_EQ(float(NonPositive<float>(-1.0f)), float(NonPositive<float>(-1.0f)));
    EXPECT_EQ(float(NonPositive<float>(0.0f)), 0.0f);
    EXPECT_EQ(float(NonPositive<float>(-std::numeric_limits<float>::infinity())),
              -std::numeric_limits<float>::infinity());

    EXPECT_THROW(NonPositive<float>{0.00001f}, NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{1.4f}, NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{std::numeric_limits<float>::infinity()},
                 NonPositive<float>::exception_type);
    EXPECT_THROW(NonPositive<float>{std::numeric_limits<float>::quiet_NaN()},
                 NonPositive<float>::exception_type);
}

TEST(CheckedValue, NonPositiveDouble)
{
    EXPECT_EQ(double(NonPositive<double>(-1.0f)), -1.0f);
    EXPECT_EQ(double(NonPositive<double>(-1.0f)), double(NonPositive<double>(-1.0f)));
    EXPECT_EQ(double(NonPositive<double>(0.0f)), 0.0f);
    EXPECT_EQ(double(NonPositive<double>(-std::numeric_limits<double>::infinity())),
              -std::numeric_limits<double>::infinity());

    EXPECT_THROW(NonPositive<double>{0.00001f}, NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{1.4f}, NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{std::numeric_limits<double>::infinity()},
                 NonPositive<double>::exception_type);
    EXPECT_THROW(NonPositive<double>{std::numeric_limits<double>::quiet_NaN()},
                 NonPositive<double>::exception_type);
}

TEST(CheckedValue, NonPositiveInt)
{
    EXPECT_EQ(int(NonPositive<int>(-1)), -1);
    EXPECT_EQ(int(NonPositive<int>(-1)), int(NonPositive<int>(-1)));
    EXPECT_EQ(int(NonPositive<int>(0)), 0);

    EXPECT_THROW(NonPositive<int>{1}, NonPositive<int>::exception_type);
    EXPECT_THROW(NonPositive<int>{2}, NonPositive<int>::exception_type);
}

TEST(CheckedValue, FiniteDouble)
{
    EXPECT_EQ(double(Finite<double>(0)), 0.0);
    EXPECT_EQ(double(Finite<double>(-1.0)), -1.0);
    EXPECT_EQ(double(Finite<double>(+1.0)), +1.0);

    EXPECT_THROW(Finite<double>{std::numeric_limits<double>::infinity()},
                 Finite<double>::exception_type);
    EXPECT_THROW(Finite<double>{std::numeric_limits<double>::quiet_NaN()},
                 Finite<double>::exception_type);
}

TEST(CheckedValue, FloatUnitInterval)
{
    EXPECT_NO_THROW(UnitInterval<float>(0.0f));
    EXPECT_NO_THROW(UnitInterval<float>(0.01f));
    EXPECT_NO_THROW(UnitInterval<float>(0.5f));
    EXPECT_NO_THROW(UnitInterval<float>(0.9999f));
    EXPECT_NO_THROW(UnitInterval<float>(1.0f));

    EXPECT_EQ(float(UnitInterval<float>(0.0f)), 0.0f);
    EXPECT_EQ(float(UnitInterval<float>(0.01f)), 0.01f);
    EXPECT_EQ(float(UnitInterval<float>(0.5f)), 0.5f);
    EXPECT_EQ(float(UnitInterval<float>(0.9999f)), 0.9999f);
    EXPECT_EQ(float(UnitInterval<float>(1.0f)), 1.0f);

    EXPECT_THROW(UnitInterval<float>(2.0f), UnitInterval<float>::exception_type);
    EXPECT_THROW(UnitInterval<float>(-1.0f), UnitInterval<float>::exception_type);
    EXPECT_THROW(UnitInterval<float>(1.00001f), UnitInterval<float>::exception_type);
    EXPECT_THROW(UnitInterval<float>(-0.00001f), UnitInterval<float>::exception_type);
    EXPECT_THROW(UnitInterval<float>{std::numeric_limits<float>::infinity()},
                 UnitInterval<float>::exception_type);
}

TEST(CheckedValue, IntUnitInterval)
{
    EXPECT_EQ(int(UnitInterval<int>(0)), 0);
    EXPECT_EQ(int(UnitInterval<int>(1)), 1);

    EXPECT_THROW(UnitInterval<int>(2), UnitInterval<int>::exception_type);
    EXPECT_THROW(UnitInterval<int>(-1), UnitInterval<int>::exception_type);
}

namespace playrho {
class Body;
}

TEST(CheckedValue, NonZero)
{
    EXPECT_THROW(NonZero<int>(0), NonZero<int>::exception_type);
    EXPECT_NO_THROW(NonZero<int>(1));
}

TEST(CheckedValue, NonNull)
{
    EXPECT_THROW(NonNull<Body*>(nullptr), NonNull<Body*>::exception_type);
    EXPECT_NO_THROW(NonNull<Body*>(reinterpret_cast<Body*>(1)));

    const int a = 5;
    const auto foo = NonNull<const int*>(&a);
    EXPECT_EQ(*foo, a);

    struct B {
        int field1 = 6;
        double field2 = 1.6;
        const char* field3 = "foo";
    };
    auto b = B{};
    auto boo = NonNull<B*>(&b);
    EXPECT_EQ(boo->field2, 1.6);
    EXPECT_EQ((*boo).field2, 1.6);
    EXPECT_EQ(boo->field1, 6);
    boo->field1 = 5;
    EXPECT_EQ(boo->field1, 5);
    EXPECT_EQ((*boo).field1, 5);
    EXPECT_EQ(b.field1, 5);
    (*boo).field1 = 44;
    EXPECT_EQ(b.field1, 44);
}

template <typename T>
struct CountingChecker {
    static unsigned numChecks;

    auto operator()(const T&) noexcept -> const char*
    {
        ++numChecks;
        return nullptr;
    }
};

template <typename T>
unsigned CountingChecker<T>::numChecks = 0u;

TEST(CheckedValue, CopyFromDiffNoExcept)
{
    CountingChecker<float>::numChecks = 0;
    using TrueType = Checked<float, CountingChecker<float>, true>;
    using FalseType = Checked<float, CountingChecker<float>, false>;
    constexpr auto vA = 42.0f;
    constexpr auto vB = 88.0f;
    TrueType a{vA};
    FalseType b{vB};
    ASSERT_NE(vA, vB);
    ASSERT_EQ(b.get(), vB);
    const auto beforeChecks = CountingChecker<float>::numChecks;
#ifdef NDEBUG
    EXPECT_EQ(beforeChecks, 1u);
#else
    EXPECT_EQ(beforeChecks, 2u);
#endif
    const auto copyB = b;
    b = a;
    EXPECT_EQ(beforeChecks, CountingChecker<float>::numChecks);
    EXPECT_EQ(b.get(), vA);
    a = copyB;
    EXPECT_EQ(beforeChecks, CountingChecker<float>::numChecks);
    EXPECT_EQ(a.get(), vB);
    CountingChecker<float>::numChecks = 0;
}
