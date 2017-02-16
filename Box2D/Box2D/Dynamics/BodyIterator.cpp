//
//  BodyIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#include <Box2D/Dynamics/BodyIterator.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

BodyIterator::pointer BodyIterator::next(pointer q) const noexcept
{
	return q->m_next;
}
