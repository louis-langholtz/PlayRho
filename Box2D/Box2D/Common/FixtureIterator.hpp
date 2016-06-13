//
//  FixtureIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef FixtureIterator_hpp
#define FixtureIterator_hpp

#include <iterator>

namespace box2d {

class Fixture;

class FixtureIterator: public std::iterator<std::forward_iterator_tag, Fixture>
{
public:
	constexpr explicit FixtureIterator(pointer* f) noexcept: p{f} {}
	constexpr FixtureIterator(const FixtureIterator& it) noexcept: p{it.p} {}
	
	FixtureIterator& operator++() noexcept { p = next(p); return *this; }
	FixtureIterator operator++(int) { FixtureIterator tmp{*this}; operator++(); return tmp; }
	
	constexpr bool operator==(const FixtureIterator& rhs) const noexcept {return *p == *rhs.p; }
	constexpr bool operator!=(const FixtureIterator& rhs) const noexcept {return *p != *rhs.p; }
	
	reference operator*() const noexcept { return **p; }
	
private:
	friend class FixtureList;

	pointer* next(pointer* q) const noexcept;
	
	pointer* p = nullptr;
};
	
} // namespace box2d

#endif /* FixtureIterator_hpp */
