//
//  JointList.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/13/16.
//
//

#ifndef JointList_hpp
#define JointList_hpp

#include <Box2D/Common/Settings.hpp>
#include <Box2D/Dynamics/JointIterator.hpp>
#include <Box2D/Dynamics/ConstJointIterator.hpp>

namespace box2d {

class Joint;

class JointList
{
public:
	using iterator = JointIterator;
	using const_iterator = ConstJointIterator;

	using pointer = Joint*;
	using size_type = joint_count_t;

	JointList() = default;
	constexpr JointList(const JointList& copy) = delete;

	JointList& operator= (const JointList& rhs) = delete;
	
	iterator begin() noexcept { return iterator(p); }
	iterator end() noexcept { return iterator(nullptr); }

	const_iterator begin() const noexcept { return const_iterator{p}; }
	const_iterator end() const noexcept { return const_iterator{nullptr}; }

	constexpr bool empty() const noexcept { return p == nullptr; }
	size_type size() const noexcept { return n; }
	constexpr size_type max_size() const noexcept { return MaxJoints; }

	constexpr bool operator== (const JointList& rhs) const noexcept { return p == rhs.p; }
	constexpr bool operator!= (const JointList& rhs) const noexcept { return p != rhs.p; }

private:
	friend class World;

	void push_front(pointer value) noexcept;
	iterator erase(iterator pos);

	pointer p = nullptr;
	joint_count_t n = 0;
};

}; // namespace box2d

#endif /* JointList_hpp */
