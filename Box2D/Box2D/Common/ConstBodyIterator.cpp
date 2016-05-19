//
//  ConstBodyIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#include <Box2D/Common/ConstBodyIterator.hpp>
#include <Box2D/Dynamics/Body.h>

using namespace box2d;

ConstBodyIterator::pointer ConstBodyIterator::next(pointer q) const noexcept
{
	return p->GetNext();
}