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

namespace box2d {

class Fixture;

class FixtureIterator: public std::iterator<std::forward_iterator_tag, Fixture>
{
public:
	FixtureIterator(pointer b) noexcept: p(b) {}
	FixtureIterator(const FixtureIterator& it) noexcept: p(it.p) {}
	
	FixtureIterator& operator++() noexcept { p = next(p); return *this; }
	FixtureIterator operator++(int) { FixtureIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const FixtureIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const FixtureIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};
	
} // namespace box2d

#endif /* b2FixtureIterator_hpp */
