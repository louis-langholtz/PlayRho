//
//  b2FixtureIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef b2FixtureIterator_hpp
#define b2FixtureIterator_hpp

#include <iterator>

class b2Fixture;

class b2FixtureIterator: public std::iterator<std::forward_iterator_tag, b2Fixture>
{
public:
	b2FixtureIterator(pointer b) noexcept: p(b) {}
	b2FixtureIterator(const b2FixtureIterator& it) noexcept: p(it.p) {}
	
	b2FixtureIterator& operator++() noexcept { p = next(p); return *this; }
	b2FixtureIterator operator++(int) { b2FixtureIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const b2FixtureIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const b2FixtureIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};
#endif /* b2FixtureIterator_hpp */
