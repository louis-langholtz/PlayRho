//
//  JointIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#ifndef JointIterator_hpp
#define JointIterator_hpp

#include <iterator>

namespace box2d {
	
class Joint;

class JointIterator: public std::iterator<std::forward_iterator_tag, Joint>
{
public:
	constexpr explicit JointIterator(pointer b) noexcept: p{b} {}
	constexpr JointIterator(const JointIterator& it) noexcept: p{it.p} {}
	
	JointIterator& operator++() noexcept { p = next(p); return *this; }
	JointIterator operator++(int) { JointIterator tmp(*this); operator++(); return tmp; }
	
	constexpr bool operator==(const JointIterator& rhs) const noexcept {return p == rhs.p; }
	constexpr bool operator!=(const JointIterator& rhs) const noexcept {return p != rhs.p; }
	
	reference operator*() const noexcept { return *p; }
	
private:
	friend class JointList;
	
	pointer next(pointer q) const noexcept;
	
	pointer p;
};
	
} // namespace box2d

#endif /* JointIterator_hpp */
