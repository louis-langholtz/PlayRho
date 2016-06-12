//
//  ConstBodyIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef ConstBodyIterator_hpp
#define ConstBodyIterator_hpp

#include <iterator>

namespace box2d {

class Body;

class ConstBodyIterator: public std::iterator<std::forward_iterator_tag, const Body>
{
public:
	constexpr explicit ConstBodyIterator(pointer b) noexcept: p{b} {}
	constexpr ConstBodyIterator(const ConstBodyIterator& it) noexcept: p{it.p} {}
	
	ConstBodyIterator& operator++() noexcept { p = next(p); return *this; }
	ConstBodyIterator operator++(int) { ConstBodyIterator tmp(*this); operator++(); return tmp; }
	
	constexpr bool operator==(const ConstBodyIterator& rhs) const noexcept {return p == rhs.p; }
	constexpr bool operator!=(const ConstBodyIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};

} // namespace box2d

#endif /* ConstBodyIterator_hpp */
