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

#include <PlayRho/Common/StatsResource.hpp>

using namespace playrho;

TEST(StatsResource, DefaultConstruction)
{
    const pmr::StatsResource resource;
    EXPECT_EQ(resource.upstream_resource(), pmr::new_delete_resource());
    EXPECT_EQ(resource.GetStats().blocksAllocated, 0u);
    EXPECT_EQ(resource.GetStats().bytesAllocated, 0u);
    EXPECT_EQ(resource.GetStats().maxBlocksAllocated, 0u);
    EXPECT_EQ(resource.GetStats().maxBytesAllocated, 0u);
    EXPECT_EQ(resource.GetStats().maxBytes, 0u);
    EXPECT_EQ(resource.GetStats().maxAlignment, 0u);
    EXPECT_TRUE(resource.is_equal(resource));
    EXPECT_FALSE(resource.is_equal(pmr::StatsResource()));
}

TEST(StatsResource, ConstructorSetsUpstream)
{
    pmr::StatsResource upstream;
    pmr::StatsResource resource{&upstream};
    EXPECT_EQ(resource.upstream_resource(), &upstream);
}

TEST(StatsResource, allocate_deallocate)
{
    pmr::StatsResource resource;
    using type = double;
    constexpr auto bytes = sizeof(type);
    constexpr auto alignment = alignof(type);

    auto p0 = static_cast<void*>(nullptr);
    EXPECT_NO_THROW(p0 = resource.allocate(bytes, alignment));
    EXPECT_NE(p0, nullptr);
    EXPECT_EQ(resource.GetStats().blocksAllocated, 1u);
    EXPECT_EQ(resource.GetStats().bytesAllocated, bytes);
    EXPECT_EQ(resource.GetStats().maxBlocksAllocated, 1u);
    EXPECT_EQ(resource.GetStats().maxBytesAllocated, bytes);
    EXPECT_EQ(resource.GetStats().maxBytes, bytes);
    EXPECT_EQ(resource.GetStats().maxAlignment, alignment);

    auto p1 = static_cast<void*>(nullptr);
    EXPECT_NO_THROW(p1 = resource.allocate(bytes * 2u, alignment * 2u));
    EXPECT_NE(p1, nullptr);
    EXPECT_EQ(resource.GetStats().blocksAllocated, 2u);
    EXPECT_EQ(resource.GetStats().bytesAllocated, bytes + bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxBlocksAllocated, 2u);
    EXPECT_EQ(resource.GetStats().maxBytesAllocated, bytes + bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxBytes, bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxAlignment, alignment * 2u);

    EXPECT_NO_THROW(resource.deallocate(p0, bytes, alignment));
    EXPECT_EQ(resource.GetStats().blocksAllocated, 1u);
    EXPECT_EQ(resource.GetStats().bytesAllocated, bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxBlocksAllocated, 2u);
    EXPECT_EQ(resource.GetStats().maxBytesAllocated, bytes + bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxBytes, bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxAlignment, alignment * 2u);

    EXPECT_NO_THROW(resource.deallocate(p1, bytes * 2u, alignment * 2u));
    EXPECT_EQ(resource.GetStats().blocksAllocated, 0u);
    EXPECT_EQ(resource.GetStats().bytesAllocated, 0u);
    EXPECT_EQ(resource.GetStats().maxBlocksAllocated, 2u);
    EXPECT_EQ(resource.GetStats().maxBytesAllocated, bytes + bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxBytes, bytes * 2u);
    EXPECT_EQ(resource.GetStats().maxAlignment, alignment * 2u);
}
