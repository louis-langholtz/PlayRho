//
//  b2ConstBodyIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#include <Box2D/Common/b2ConstBodyIterator.hpp>
#include <Box2D/Dynamics/b2Body.h>

using namespace box2d;

b2ConstBodyIterator::pointer b2ConstBodyIterator::next(pointer q) const noexcept
{
	return p->GetNext();
}