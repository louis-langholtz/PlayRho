//
//  b2ConstFixtureList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef b2ConstFixtureList_hpp
#define b2ConstFixtureList_hpp

#include <Box2D/Common/b2ConstFixtureIterator.hpp>

class b2Fixture;

class b2ConstFixtureList
{
public:
	using const_iterator = b2ConstFixtureIterator;
	
	b2ConstFixtureList() = default;
	b2ConstFixtureList(const b2Fixture* b): p(b) {}
	
	const_iterator begin() noexcept { return const_iterator(p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	const b2Fixture* p = nullptr;
};

#endif /* b2ConstFixtureList_hpp */
