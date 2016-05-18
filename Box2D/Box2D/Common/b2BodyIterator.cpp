//
//  b2BodyIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#include <Box2D/Common/b2BodyIterator.hpp>
#include <Box2D/Dynamics/b2Body.h>

using namespace box2d;

b2BodyIterator::pointer b2BodyIterator::next(pointer q) const noexcept
{
	return p->GetNext();
}