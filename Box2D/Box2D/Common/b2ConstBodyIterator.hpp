//
//  b2ConstBodyIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 3/6/16.
//
//

#ifndef b2ConstBodyIterator_hpp
#define b2ConstBodyIterator_hpp

#include <iterator>

class b2Body;

class b2ConstBodyIterator: public std::iterator<std::forward_iterator_tag, const b2Body>
{
public:
	b2ConstBodyIterator(pointer b) noexcept: p(b) {}
	b2ConstBodyIterator(const b2ConstBodyIterator& it) noexcept: p(it.p) {}
	
	b2ConstBodyIterator& operator++() noexcept { p = next(p); return *this; }
	b2ConstBodyIterator operator++(int) { b2ConstBodyIterator tmp(*this); operator++(); return tmp; }
	
	bool operator==(const b2ConstBodyIterator& rhs) const noexcept {return p == rhs.p; }
	bool operator!=(const b2ConstBodyIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	pointer next(pointer q) const noexcept;
	
	pointer p;
};

#endif /* b2ConstBodyIterator_hpp */
