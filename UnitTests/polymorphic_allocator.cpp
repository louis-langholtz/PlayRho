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

#include <cstddef> // for std::byte

#include <PlayRho/Common/MemoryResource.hpp>

using namespace playrho;

TEST(polymorphic_allocator, ResourceNonNullOnDefaultConstruction)
{
    pmr::polymorphic_allocator<std::byte> allocator;
    EXPECT_TRUE(allocator.resource());
}

TEST(polymorphic_allocator, ResourceGivenOnConstruction)
{
    const auto resource = pmr::null_memory_resource();
    pmr::polymorphic_allocator<std::byte> allocator{resource};
    EXPECT_EQ(allocator.resource(), resource);
}

TEST(polymorphic_allocator, allocate_too_big)
{
    pmr::polymorphic_allocator<double> allocator{};
    auto p = static_cast<double*>(nullptr);
    EXPECT_THROW(p = allocator.allocate(std::numeric_limits<std::size_t>::max()),
                 std::bad_array_new_length);
    EXPECT_EQ(p, nullptr);
}

TEST(polymorphic_allocator, equality)
{
    pmr::polymorphic_allocator<std::byte> a0;
    pmr::polymorphic_allocator<std::byte> a1;
    pmr::polymorphic_allocator<double> b0;
    pmr::polymorphic_allocator<double> b1{pmr::null_memory_resource()};
    EXPECT_TRUE(a0 == a0);
    EXPECT_TRUE(a0 == a1);
    EXPECT_TRUE(a0 == b0);
    EXPECT_FALSE(a0 == b1);
}
