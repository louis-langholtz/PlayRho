//
//  ConstJointEdgeIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#ifndef ConstJointEdgeIterator_hpp
#define ConstJointEdgeIterator_hpp

#include <iterator>

namespace box2d {
	
	class JointEdge;
	
	class ConstJointEdgeIterator: public std::iterator<std::forward_iterator_tag, const JointEdge>
	{
	public:
		constexpr explicit ConstJointEdgeIterator(pointer b) noexcept: p{b} {}
		constexpr ConstJointEdgeIterator(const ConstJointEdgeIterator& it) noexcept: p{it.p} {}
		
		ConstJointEdgeIterator& operator++() noexcept { p = next(p); return *this; }
		ConstJointEdgeIterator operator++(int) { ConstJointEdgeIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const ConstJointEdgeIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const ConstJointEdgeIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* ConstJointEdgeIterator_hpp */
