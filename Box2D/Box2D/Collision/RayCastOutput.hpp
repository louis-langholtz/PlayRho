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

#ifndef RayCastOutput_hpp
#define RayCastOutput_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	struct RayCastInput;
	class AABB;
	class Fixture;

	/// Ray-cast output data.
	/// @details The ray hits at p1 + fraction * (p2 - p1), where p1 and p2 come from RayCastInput.
	struct RayCastOutput
	{
		RayCastOutput() = default;
		
		constexpr RayCastOutput(UnitVec2 n, RealNum f, bool h = true) noexcept: normal{n}, fraction{f}, hit{h} {}
		
		UnitVec2 normal;
		RealNum fraction = 0;
		bool hit = false;
	};

	RayCastOutput RayCast(const AABB& aabb, const RayCastInput& input);

	/// Cast a ray against the shape of the given fixture.
	/// @param f Fixture.
	/// @param input the ray-cast input parameters.
	/// @param childIndex Child index.
	RayCastOutput RayCast(const Fixture& f, const RayCastInput& input, child_count_t childIndex);

} // namespace box2d

#endif /* RayCastOutput_hpp */
