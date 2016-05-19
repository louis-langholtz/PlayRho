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

class Fixture;

class FixtureList
{
public:
	using iterator = FixtureIterator;
	using const_iterator = b2ConstFixtureIterator;
	
	FixtureList() = default;
	FixtureList(Fixture* b): p(b) {}
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	Fixture* p = nullptr;
};

} // namespace box2d

#endif /* b2FixtureList_hpp */
