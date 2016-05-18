//
//  b2FixtureList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef b2FixtureList_hpp
#define b2FixtureList_hpp

#include <Box2D/Common/b2FixtureIterator.hpp>
#include <Box2D/Common/b2ConstFixtureIterator.hpp>

namespace box2d {

class b2Fixture;

class b2FixtureList
{
public:
	using iterator = b2FixtureIterator;
	using const_iterator = b2ConstFixtureIterator;
	
	b2FixtureList() = default;
	b2FixtureList(b2Fixture* b): p(b) {}
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	b2Fixture* p = nullptr;
};

} // namespace box2d

#endif /* b2FixtureList_hpp */
