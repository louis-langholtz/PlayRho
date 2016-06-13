//
//  ConstFixtureIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/7/16.
//
//

#ifndef ConstFixtureIterator_hpp
#define ConstFixtureIterator_hpp

#include <iterator>

namespace box2d {

class Fixture;

class ConstFixtureIterator: public std::iterator<std::forward_iterator_tag, const Fixture>
{
public:
	constexpr explicit ConstFixtureIterator(pointer const * f) noexcept: p{f} {}
	constexpr ConstFixtureIterator(const ConstFixtureIterator& it) noexcept: p{it.p} {}
	
	ConstFixtureIterator& operator++() noexcept { p = next(p); return *this; }
	ConstFixtureIterator operator++(int) { ConstFixtureIterator tmp(*this); operator++(); return tmp; }
	
	constexpr bool operator==(const ConstFixtureIterator& rhs) const noexcept {return *p == *rhs.p; }
	constexpr bool operator!=(const ConstFixtureIterator& rhs) const noexcept {return *p != *rhs.p; }
	
	reference operator*() const noexcept { return **p; }
	
private:
	friend class FixtureList;

	pointer const * next(pointer const * q) const noexcept;

	pointer const * p = nullptr;
};
	
} // namespace box2d

#endif /* ConstFixtureIterator_hpp */
