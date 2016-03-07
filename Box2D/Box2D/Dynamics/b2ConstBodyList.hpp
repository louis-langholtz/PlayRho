//
//  b2ConstBodyList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef b2ConstBodyList_hpp
#define b2ConstBodyList_hpp

#include <Box2D/Common/b2ConstBodyIterator.hpp>

class b2Body;

class b2ConstBodyList
{
public:
	using const_iterator = b2ConstBodyIterator;
	
	b2ConstBodyList() = default;
	b2ConstBodyList(const b2Body* b): p(b) {}
	
	const_iterator begin() noexcept { return const_iterator(p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	const b2Body* p = nullptr;
};

#endif /* b2ConstBodyList_hpp */
