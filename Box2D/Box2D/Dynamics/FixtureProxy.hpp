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

#ifndef FixtureProxy_hpp
#define FixtureProxy_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Collision/AABB.hpp>

namespace box2d
{
	class Fixture;

	/// Fixture proxy.
	/// @detail This proxy is used internally to connect fixtures to the broad-phase.
	/// @note This data structure is 32-bytes large (on at least one 64-bit platform).
	struct FixtureProxy
	{
		using size_type = std::remove_const<decltype(MaxContacts)>::type;
		
		FixtureProxy() = default;
		
		FixtureProxy(const FixtureProxy& copy) = default;
		
		FixtureProxy(const AABB& bb, size_type pid, Fixture* f, child_count_t ci):
			aabb{bb}, fixture{f}, proxyId{pid}, childIndex{ci} {}
		
		AABB aabb; ///< Axis Aligned Bounding Box. 16-bytes.
		Fixture* fixture; ///< Fixture. 8-bytes.
		size_type proxyId; ///< Proxy ID. 4-bytes.
		child_count_t childIndex; ///< Child index. 4-bytes.
	};
	
} // namespace box2d

#endif /* FixtureProxy_hpp */
