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

#include <Box2D/Collision/ReferenceFace.hpp>
#include <Box2D/Collision/EdgeInfo.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

using namespace box2d;

#if 0

ReferenceFace box2d::GetReferenceFace(const EdgeInfo& edgeInfo)
{
	constexpr auto idx0 = ReferenceFace::index_type{0};
	constexpr auto idx1 = ReferenceFace::index_type{1};
	return (edgeInfo.IsFront())?
		ReferenceFace{idx0, edgeInfo.GetVertex1(), idx1, edgeInfo.GetVertex2(), edgeInfo.GetNormal1()}:
		ReferenceFace{idx1, edgeInfo.GetVertex2(), idx0, edgeInfo.GetVertex1(), -edgeInfo.GetNormal1()};
}

ReferenceFace box2d::GetReferenceFace(const PolygonShape& localShapeB, const ReferenceFace::index_type index)
{
	const auto i1 = index;
	const auto i2 = GetModuloNext(index, localShapeB.GetVertexCount());
	return ReferenceFace{i1, localShapeB.GetVertex(i1), i2, localShapeB.GetVertex(i2), localShapeB.GetNormal(i1)};
}

#endif
