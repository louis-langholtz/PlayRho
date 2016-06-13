//
//  ConstJointIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#ifndef ConstJointIterator_hpp
#define ConstJointIterator_hpp

#include <iterator>

namespace box2d {
	
	class Joint;
	
	class ConstJointIterator: public std::iterator<std::forward_iterator_tag, const Joint>
	{
	public:
		constexpr explicit ConstJointIterator(pointer b) noexcept: p{b} {}
		constexpr ConstJointIterator(const ConstJointIterator& it) noexcept: p{it.p} {}
		
		ConstJointIterator& operator++() noexcept { p = next(p); return *this; }
		ConstJointIterator operator++(int) { ConstJointIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ConstJointIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ConstJointIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ConstJointIterator_hpp */
