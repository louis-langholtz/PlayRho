/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>

#include <cstring>

using namespace box2d;

namespace {
	
	inline bool IsEachVertexFarEnoughApart(Span<const Length2D> vertices)
	{
		for (auto i = decltype(vertices.size()){1}; i < vertices.size(); ++i)
		{
			const auto delta = vertices[i-1] - vertices[i];
			
			// XXX not quite right unit-wise but this works well enough.
			if (GetLengthSquared(StripUnits(delta)) * Meter <= DefaultLinearSlop)
			{
				return false;
			}
		}
		return true;
	}
	
} // anonymous namespace

ChainShape::ChainShape(const ChainShape& other):
	Shape{e_chain, Conf{}.UseVertexRadius(other.GetVertexRadius())}
{
	*this = other;
}

ChainShape& ChainShape::operator=(const ChainShape& other)
{
	if (&other != this)
	{
		Clear();

		m_count = other.m_count;

		m_vertices = alloc<Length2D>(other.m_count);
		std::memcpy(m_vertices, other.m_vertices, other.m_count * sizeof(Length2D));
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

void ChainShape::CreateLoop(Span<const Length2D> vertices)
{
	assert(vertices.begin() != nullptr);
	assert(vertices.size() >= 3);
	assert(IsEachVertexFarEnoughApart(vertices));
	assert(m_vertices == nullptr && m_count == 0);
	
	m_count = static_cast<child_count_t>(vertices.size() + 1);
	m_vertices = alloc<Length2D>(m_count);
	std::memcpy(m_vertices, vertices.begin(), vertices.size() * sizeof(Vec2));
	m_vertices[vertices.size()] = m_vertices[0];
}

void ChainShape::CreateChain(Span<const Length2D> vertices)
{
	assert(vertices.begin() != nullptr);
	assert(vertices.size() >= 2);
	assert(IsEachVertexFarEnoughApart(vertices));
	assert((m_vertices == nullptr) && (m_count == 0));

	m_count = static_cast<child_count_t>(vertices.size());
	m_vertices = alloc<Length2D>(vertices.size());
	std::memcpy(m_vertices, vertices.begin(), m_count * sizeof(Vec2));
}

EdgeShape ChainShape::GetChildEdge(child_count_t index) const
{
	assert(index + 1 < m_count);

	const auto isLooped = ::IsLooped(*this);
	const auto v0 = (index > 0)? m_vertices[index - 1]: isLooped? m_vertices[m_count - 2]: GetInvalid<Length2D>();
	const auto v3 = (index < (m_count - 2))? m_vertices[index + 2]: isLooped? m_vertices[1]: GetInvalid<Length2D>();
	auto conf = EdgeShape::Conf{};
	conf.UseVertexRadius(GetVertexRadius());
	//conf.v0 = v0;
	//conf.v3 = v3;
	return EdgeShape{m_vertices[index + 0], m_vertices[index + 1], conf};
}

child_count_t box2d::GetChildCount(const ChainShape& shape)
{
	// edge count = vertex count - 1
	const auto count = shape.GetVertexCount();
	return (count > 1)? count - 1: 0;
}

bool box2d::TestPoint(const ChainShape& shape, const Transformation& xf, const Length2D p)
{
	NOT_USED(shape);
	NOT_USED(xf);
	NOT_USED(p);
	return false;
}
