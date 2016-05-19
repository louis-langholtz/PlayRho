//
//  BodyIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef BodyIterator_hpp
#define BodyIterator_hpp

#include <iterator>

namespace box2d {

class Body;

class BodyIterator: public std::iterator<std::forward_iterator_tag, Body>
{
public:
	BodyIterator(pointer b) noexcept: p(b) {}
	BodyIterator(const BodyIterator& it) noexcept: p(it.p) {}
	
	BodyIterator& operator++() noexcept { p = next(p); return *this; }
	BodyIterator operator++(int) { BodyIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const BodyIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const BodyIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};

} // namespace box2d

#endif /* BodyIterator_hpp */
