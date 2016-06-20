/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Settings.h>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

namespace box2d {

// Memory allocators. Modify these to use your own allocator.
void* alloc(size_t size)
{
	assert(size >= 0);
	return std::malloc(size);
}

void* realloc(void* ptr, size_t new_size)
{
	assert(new_size >= 0);
	return std::realloc(ptr, new_size);
}

void free(void* mem)
{
	std::free(mem);
}

// You can modify this to use your logging facility.
void log(const char* string, ...)
{
	va_list args;
	va_start(args, string);
	std::vprintf(string, args);
	va_end(args);
}

} // namespace box2d