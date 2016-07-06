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
#include <new>
#include <string.h>

using namespace box2d;

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
	m_hasPrevVertex = true;
	m_hasNextVertex = true;
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

	m_hasPrevVertex = false;
	m_hasNextVertex = false;

	m_prevVertex = Vec2_zero;
	m_nextVertex = Vec2_zero;
}

void ChainShape::SetPrevVertex(const Vec2& prevVertex) noexcept
{
	m_prevVertex = prevVertex;
	m_hasPrevVertex = true;
}

void ChainShape::SetNextVertex(const Vec2& nextVertex) noexcept
{
	m_nextVertex = nextVertex;
	m_hasNextVertex = true;
}

Shape* ChainShape::Clone(BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(ChainShape));
	const auto clone = new (mem) ChainShape;
	clone->CreateChain(m_vertices, m_count);
	clone->m_prevVertex = m_prevVertex;
	clone->m_nextVertex = m_nextVertex;
	clone->m_hasPrevVertex = m_hasPrevVertex;
	clone->m_hasNextVertex = m_hasNextVertex;
	return clone;
}

child_count_t ChainShape::GetChildCount() const
{
	// edge count = vertex count - 1
	assert(m_count > 0);
	return m_count - 1;
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
	else if (m_hasPrevVertex)
	{
		edge->SetVertex0(m_prevVertex);
	}

	assert(m_count >= 2);
	if (index < (m_count - 2))
	{
		edge->SetVertex3(m_vertices[index + 2]);
	}
	else if (m_hasNextVertex)
	{
		edge->SetVertex3(m_nextVertex);
	}
}

bool ChainShape::TestPoint(const Transformation& xf, const Vec2& p) const
{
	BOX2D_NOT_USED(xf);
	BOX2D_NOT_USED(p);
	return false;
}

bool ChainShape::RayCast(RayCastOutput* output, const RayCastInput& input,
							const Transformation& xf, child_count_t childIndex) const
{
	assert(childIndex < m_count);

	const auto i1 = childIndex;
	const auto i2 = GetNextIndex(childIndex);
	const auto edgeShape = EdgeShape(m_vertices[i1], m_vertices[i2]);
	return edgeShape.RayCast(output, input, xf, 0);
}

AABB ChainShape::ComputeAABB(const Transformation& xf, child_count_t childIndex) const
{
	assert(childIndex < m_count);

	const auto i1 = childIndex;
	const auto i2 = GetNextIndex(childIndex);
	const auto v1 = Transform(m_vertices[i1], xf);
	const auto v2 = Transform(m_vertices[i2], xf);
	return AABB{v1, v2};
}

MassData ChainShape::ComputeMass(float_t density) const
{
	BOX2D_NOT_USED(density);

	return MassData{float_t{0}, Vec2_zero, float_t{0}};
}
