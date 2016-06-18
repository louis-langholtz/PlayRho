//
//  JointEdgeIterator.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#ifndef JointEdgeIterator_hpp
#define JointEdgeIterator_hpp

#include <iterator>

namespace box2d {
	
	class JointEdge;
	
	class JointEdgeIterator: public std::iterator<std::forward_iterator_tag, JointEdge>
	{
	public:
		constexpr explicit JointEdgeIterator(pointer b) noexcept: p{b} {}
		constexpr JointEdgeIterator(const JointEdgeIterator& it) noexcept: p{it.p} {}
		
		JointEdgeIterator& operator++() noexcept { p = next(p); return *this; }
		JointEdgeIterator operator++(int) { JointEdgeIterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const JointEdgeIterator& rhs) const noexcept {return p == rhs.p; }
		constexpr bool operator!=(const JointEdgeIterator& rhs) const noexcept {return p != rhs.p; }
		
		reference operator*() const noexcept { return *p; }
		
	private:
		friend class JointEdgeList;
		
		pointer next(pointer q) const noexcept;
		
		pointer p;
	};
	
} // namespace box2d

#endif /* JointEdgeIterator_hpp */
