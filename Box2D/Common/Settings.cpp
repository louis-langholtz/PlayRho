/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <Box2D/Common/Settings.hpp>

#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <typeinfo>
#include <sstream>

namespace box2d {

// Memory allocators. Modify these to use your own allocator.
void* alloc(size_t size)
{
    return std::malloc(size);
}

void* realloc(void* ptr, size_t new_size)
{
    return std::realloc(ptr, new_size);
}

void free(void* mem)
{
    std::free(mem);
}

Version GetVersion() noexcept
{
    return Version{3, 0, 0};
}

template <typename T>
const char* GetTypeName() noexcept
{
    // No gaurantee of what the following returns. Could be mangled!
    // See http://en.cppreference.com/w/cpp/types/type_info/name
    return typeid(T).name();
}

template <>
const char* GetTypeName<float>() noexcept
{
    return "float";
}

template <>
const char* GetTypeName<double>() noexcept
{
    return "double";
}

template <>
const char* GetTypeName<long double>() noexcept
{
    return "long-double";
}

template <>
const char* GetTypeName<Fixed32>() noexcept
{
    return "Fixed32";
}

#ifndef _WIN32
template <>
const char* GetTypeName<Fixed64>() noexcept
{
    return "Fixed64";
}
#endif

std::string GetBuildDetails() noexcept
{
    std::stringstream stream;
    stream << "asserts=";
#ifdef NDEBUG
    stream << "off";
#else
    stream << "on";
#endif
    stream << ", RealNum=" << GetTypeName<RealNum>();
    return stream.str();
}

} // namespace box2d
