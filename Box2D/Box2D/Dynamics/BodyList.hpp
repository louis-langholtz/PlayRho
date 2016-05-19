//
//  BodyList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef BodyList_hpp
#define BodyList_hpp

#include <Box2D/Common/b2BodyIterator.hpp>
#include <Box2D/Common/b2ConstBodyIterator.hpp>

namespace box2d {

class Body;

class BodyList
{
public:
	using iterator = BodyIterator;
	using const_iterator = ConstBodyIterator;

	BodyList() = default;
	BodyList(Body* b) noexcept: p(b) {}
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	Body* p = nullptr;
};

} // namespace box2d

#endif /* BodyList_hpp */
