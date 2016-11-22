//
//  JointEdgeIterator.cpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#include <Box2D/Dynamics/Joints/JointEdgeIterator.hpp>
#include <Box2D/Dynamics/Joints/Joint.hpp>

using namespace box2d;

JointEdgeIterator::pointer JointEdgeIterator::next(pointer q) const noexcept
{
	return p->next;
}
