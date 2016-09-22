/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/ChainShape.h>
#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <Box2D/Common/BlockAllocator.h>
#include <new>
#include <string.h>

using namespace box2d;

ChainShape::ChainShape(const ChainShape& other):
	Shape{e_chain, PolygonRadius}
{
	*this = other;
}

ChainShape& ChainShape::operator=(const ChainShape& other)
{
	if (&other != this)
	{
		Clear();

		m_count = other.m_count;

		m_vertices = alloc<Vec2>(other.m_count);
		memcpy(m_vertices, other.m_vertices, other.m_count * sizeof(Vec2));

		m_prevVertex = other.m_prevVertex;		
		m_nextVertex = other.m_nextVertex;
	}
	return *this;
}

ChainShape::~ChainShape()
{
	Clear();
}

void ChainShape::Clear()
{
	free(m_vertices);
	m_vertices = nullptr;
	m_count = 0;
}

void ChainShape::CreateLoop(const Vec2* vertices, child_count_t count)
{
	assert(vertices != nullptr);
	assert(count >= 3);
	assert(IsValid(vertices[count - 1]));
	assert(IsValid(vertices[1]));
	
	assert(m_vertices == nullptr && m_count == 0);
	
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		// If the code crashes here, it means your vertices are too close together.
		assert(LengthSquared(vertices[i-1] - vertices[i]) > Square(LinearSlop));
	}

	m_count = count + 1;
	m_vertices = alloc<Vec2>(m_count);
	memcpy(m_vertices, vertices, count * sizeof(Vec2));
	m_vertices[count] = m_vertices[0];
	m_prevVertex = m_vertices[m_count - 2];
	m_nextVertex = m_vertices[1];
}

void ChainShape::CreateChain(const Vec2* vertices, child_count_t count)
{
	assert((m_vertices == nullptr) && (m_count == 0));
	assert(count >= 2);
	for (auto i = decltype(count){1}; i < count; ++i)
	{
		// If the code crashes here, it means your vertices are too close together.
		assert(LengthSquared(vertices[i-1] - vertices[i]) > Square(LinearSlop));
	}

	m_count = count;
	m_vertices = alloc<Vec2>(count);
	memcpy(m_vertices, vertices, m_count * sizeof(Vec2));

	m_prevVertex = Vec2_invalid;
	m_nextVertex = Vec2_invalid;
}

void ChainShape::SetPrevVertex(const Vec2& prevVertex) noexcept
{
	m_prevVertex = prevVertex;
}

void ChainShape::SetNextVertex(const Vec2& nextVertex) noexcept
{
	m_nextVertex = nextVertex;
}

void ChainShape::GetChildEdge(EdgeShape* edge, child_count_t index) const
{
	assert((0 <= index) && (index < (m_count - 1)));
	edge->SetRadius(GetRadius());

	edge->Set(m_vertices[index + 0], m_vertices[index + 1]);

	if (index > 0)
	{
		edge->SetVertex0(m_vertices[index - 1]);
	}
	else if (HasPrevVertex())
	{
		edge->SetVertex0(m_prevVertex);
	}

	assert(m_count >= 2);
	if (index < (m_count - 2))
	{
		edge->SetVertex3(m_vertices[index + 2]);
	}
	else if (HasNextVertex())
	{
		edge->SetVertex3(m_nextVertex);
	}
}

child_count_t box2d::GetChildCount(const ChainShape& shape)
{
	// edge count = vertex count - 1
	assert(shape.GetVertexCount() > 0);
	return shape.GetVertexCount() - 1;
}

bool box2d::TestPoint(const ChainShape& shape, const Transformation& xf, const Vec2& p)
{
	BOX2D_NOT_USED(xf);
	BOX2D_NOT_USED(p);
	return false;
}

bool box2d::RayCast(const ChainShape& shape, RayCastOutput* output, const RayCastInput& input,
			 const Transformation& xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());

	const auto i1 = childIndex;
	const auto i2 = shape.GetNextIndex(childIndex);
	const auto edgeShape = EdgeShape(shape.GetVertex(i1), shape.GetVertex(i2));
	return RayCast(edgeShape, output, input, xf, 0);
}

AABB box2d::ComputeAABB(const ChainShape& shape, const Transformation& xf, child_count_t childIndex)
{
	assert(childIndex < shape.GetVertexCount());

	const auto i1 = childIndex;
	const auto i2 = shape.GetNextIndex(childIndex);
	const auto v1 = Transform(shape.GetVertex(i1), xf);
	const auto v2 = Transform(shape.GetVertex(i2), xf);
	return AABB{v1, v2};
}

MassData box2d::ComputeMass(const ChainShape& shape, float_t density)
{
	BOX2D_NOT_USED(density);

	return MassData{float_t{0}, Vec2_zero, float_t{0}};
}
