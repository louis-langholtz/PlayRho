//
//  JointList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#ifndef JointList_hpp
#define JointList_hpp

#include <Box2D/Common/Settings.h>
#include <Box2D/Dynamics/JointIterator.hpp>
// #include <Box2D/Dynamics/ConstJointIterator.hpp>

namespace box2d {

class Joint;

class JointList
{
public:
	using iterator = JointIterator;
	using pointer = Joint*;
	using size_type = joint_count_t;

	JointList() = default;
	constexpr JointList(const JointList& copy) noexcept: p{copy.p} {}
	constexpr JointList(pointer b) noexcept: p{b} {}

	JointList& operator= (const JointList& rhs) noexcept { p = rhs.p; return *this; }
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }

	constexpr bool empty() const noexcept { return p == nullptr; }
	size_type size() const noexcept { return n; }
	constexpr size_type max_size() const noexcept { return MaxJoints; }

	constexpr bool operator== (const JointList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const JointList& rhs) const noexcept { return p != rhs.p; }

	void push_front(pointer value) noexcept;
	iterator erase(iterator pos);

private:
	pointer p = nullptr;
	joint_count_t n = 0;
};

}; // namespace box2d

#endif /* JointList_hpp */
