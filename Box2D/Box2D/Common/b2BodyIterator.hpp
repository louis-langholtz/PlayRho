//
//  b2BodyIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef b2BodyIterator_hpp
#define b2BodyIterator_hpp

#include <iterator>

namespace box2d {

class b2Body;

class b2BodyIterator: public std::iterator<std::forward_iterator_tag, b2Body>
{
public:
	b2BodyIterator(pointer b) noexcept: p(b) {}
	b2BodyIterator(const b2BodyIterator& it) noexcept: p(it.p) {}
	
	b2BodyIterator& operator++() noexcept { p = next(p); return *this; }
	b2BodyIterator operator++(int) { b2BodyIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const b2BodyIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const b2BodyIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};

} // namespace box2d

#endif /* b2BodyIterator_hpp */
