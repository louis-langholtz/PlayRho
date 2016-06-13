//
//  FixtureIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#include <Box2D/Dynamics/FixtureIterator.hpp>
#include <Box2D/Dynamics/Fixture.h>

using namespace box2d;

FixtureIterator::pointer* FixtureIterator::next(pointer* q) const noexcept
{
	return &((*q)->m_next);
}