//
//  JointEdgeList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/18/16.
//
//

#ifndef JointEdgeList_hpp
#define JointEdgeList_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Dynamics/Joints/JointEdgeIterator.hpp>
#include <Box2D/Dynamics/Joints/ConstJointEdgeIterator.hpp>

namespace box2d {
	
	class JointEdge;
	
	class JointEdgeList
	{
	public:
		using iterator = JointEdgeIterator;
		using const_iterator = ConstJointEdgeIterator;
		using pointer = JointEdge*;
		using reference = JointEdge&;
		
		JointEdgeList() = default;
		constexpr JointEdgeList(const JointEdgeList& copy) = delete;
		
		JointEdgeList& operator= (const JointEdgeList& rhs) = delete;
		
		iterator begin() noexcept { return iterator(p); }
		iterator end() noexcept { return iterator(nullptr); }
		
		const_iterator begin() const noexcept { return const_iterator{p}; }
		const_iterator end() const noexcept { return const_iterator{nullptr}; }
		
		constexpr bool empty() const noexcept { return p == nullptr; }
		
		constexpr bool operator== (const JointEdgeList& rhs) const noexcept { return p == rhs.p; }
		constexpr bool operator!= (const JointEdgeList& rhs) const noexcept { return p != rhs.p; }
		
		reference front() noexcept { return *p; }
		
	private:
		friend class Body;
		friend class World;

		void push_front(pointer value) noexcept;
		void pop_front() noexcept;
		iterator erase(iterator pos);
		
		pointer p = nullptr;
	};
	
}; // namespace box2d

#endif /* JointEdgeList_hpp */
