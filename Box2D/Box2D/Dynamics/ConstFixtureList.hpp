//
//  ConstFixtureList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef ConstFixtureList_hpp
#define ConstFixtureList_hpp

#include <Box2D/Common/b2ConstFixtureIterator.hpp>

namespace box2d {

class Fixture;

class ConstFixtureList
{
public:
	using const_iterator = ConstFixtureIterator;
	
	ConstFixtureList() = default;
	ConstFixtureList(const Fixture* b): p(b) {}
	
	const_iterator begin() noexcept { return const_iterator(p); }
	const_iterator end() noexcept { return const_iterator(nullptr); }
	
	const_iterator begin() const noexcept { return const_iterator(p); }
	const_iterator end() const noexcept { return const_iterator(nullptr); }
	
private:
	const Fixture* p = nullptr;
};

} // namespace box2d

#endif /* ConstFixtureList_hpp */
