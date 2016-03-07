//
//  b2BodyList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef b2BodyList_hpp
#define b2BodyList_hpp

#include <Box2D/Common/b2BodyIterator.hpp>
#include <Box2D/Common/b2ConstBodyIterator.hpp>

class b2Body;

class b2BodyList
{
public:
	using iterator = b2BodyIterator;
	using const_iterator = b2ConstBodyIterator;

	b2BodyList() = default;
	b2BodyList(b2Body* b): p(b) {}
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	b2Body* p = nullptr;
};

#endif /* b2BodyList_hpp */
