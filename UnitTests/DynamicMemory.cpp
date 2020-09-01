/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 *   claim that you wrote the original software. If you use this software
 *   in a product, an acknowledgment in the product documentation would be
 *   appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *   misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "UnitTests.hpp"

#include <PlayRho/Common/DynamicMemory.hpp>

#include <new>

using namespace playrho;

TEST(DynamicMemory, Alloc)
{
    auto ptr = static_cast<void *>(nullptr);
    EXPECT_NO_THROW(ptr = Alloc(0));
    EXPECT_EQ(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Alloc(1));
    EXPECT_NE(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));

    // Presumably no system can allocate max size.
    ptr = nullptr;
    EXPECT_THROW(ptr = Alloc(std::numeric_limits<std::size_t>::max()), std::bad_alloc);
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));
}

TEST(DynamicMemory, AllocArray)
{
    using ElementType = int;

    auto ptr = static_cast<ElementType*>(nullptr);
    EXPECT_NO_THROW(ptr = Alloc<ElementType>(0));
    EXPECT_EQ(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Alloc<ElementType>(1));
    EXPECT_NE(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));

    // Presumably no system can allocate max size.
    ptr = nullptr;
    EXPECT_THROW(ptr = Alloc<ElementType>(std::numeric_limits<std::size_t>::max()), std::bad_alloc);
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));
}

TEST(DynamicMemory, Realloc)
{
    auto ptr = static_cast<void *>(nullptr);
    EXPECT_NO_THROW(ptr = Realloc(nullptr, 0));
    EXPECT_EQ(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc(nullptr, 1));
    EXPECT_NE(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc(ptr, 1));
    EXPECT_NE(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc(ptr, 0));
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));

    // Presumably no system can allocate max size.
    ptr = nullptr;
    EXPECT_THROW(ptr = Realloc(ptr, std::numeric_limits<std::size_t>::max()), std::bad_alloc);
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));
}

TEST(DynamicMemory, ReallocArray)
{
    using ElementType = int;

    auto ptr = static_cast<ElementType *>(nullptr);
    EXPECT_NO_THROW(ptr = Realloc<ElementType>(nullptr, 0));
    EXPECT_EQ(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc<ElementType>(nullptr, 1));
    EXPECT_NE(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc<ElementType>(ptr, 1));
    EXPECT_NE(ptr, nullptr);
    EXPECT_NO_THROW(ptr = Realloc<ElementType>(ptr, 0));
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));

    ptr = nullptr;
    EXPECT_THROW(ptr = Realloc<ElementType>(ptr, std::numeric_limits<std::size_t>::max()), std::bad_array_new_length);
    EXPECT_EQ(ptr, nullptr);

    // Presumably no system can allocate max size.
    ptr = nullptr;
    EXPECT_THROW(ptr = Realloc<ElementType>(ptr, std::numeric_limits<std::size_t>::max()/sizeof(ElementType)), std::bad_alloc);
    EXPECT_EQ(ptr, nullptr);
    ASSERT_NO_THROW(Free(ptr));
}
