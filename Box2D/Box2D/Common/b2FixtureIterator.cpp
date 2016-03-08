//
//  b2FixtureIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#include <Box2D/Common/b2FixtureIterator.hpp>
#include <Box2D/Dynamics/b2Fixture.h>

b2FixtureIterator::pointer b2FixtureIterator::next(pointer q) const noexcept
{
	return p->GetNext();
}