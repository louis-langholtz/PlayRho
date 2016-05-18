//
//  b2ConstFixtureIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef b2ConstFixtureIterator_hpp
#define b2ConstFixtureIterator_hpp

#include <iterator>

namespace box2d {

class b2Fixture;

class b2ConstFixtureIterator: public std::iterator<std::forward_iterator_tag, const b2Fixture>
{
public:
	b2ConstFixtureIterator(pointer b) noexcept: p(b) {}
	b2ConstFixtureIterator(const b2ConstFixtureIterator& it) noexcept: p(it.p) {}
	
	b2ConstFixtureIterator& operator++() noexcept { p = next(p); return *this; }
	b2ConstFixtureIterator operator++(int) { b2ConstFixtureIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const b2ConstFixtureIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const b2ConstFixtureIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};
	
} // namespace box2d

#endif /* b2ConstFixtureIterator_hpp */
