//
//  AllocatedArray.hpp
//  Box2D
//
//  Created by Louis D. Langholtz on 6/15/16.
//
//

#ifndef Array_hpp
#define Array_hpp

#include <Box2D/Common/Settings.h>

#include <memory>
#include <iterator>

namespace box2d {

template <typename T, typename Deleter = std::default_delete<T> >
class AllocatedArray
{
public:
	using size_type = size_t;
	using value_type = T;
	using const_value_type = const value_type;
	using reference = value_type&;
	using const_reference = const value_type&;
	using pointer = value_type*;
	using const_pointer = const value_type*;
	using difference_type = std::ptrdiff_t;
	using deleter_type = Deleter;

	class iterator: public std::iterator<std::random_access_iterator_tag, value_type>
	{
	public:
		iterator() = default;
		iterator(pointer p): m_p{p} {}

		iterator& operator++() noexcept { ++m_p; return *this; }
		iterator operator++(int) { iterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const iterator& rhs) const noexcept {return m_p == rhs.m_p; }
		constexpr bool operator!=(const iterator& rhs) const noexcept {return m_p != rhs.m_p; }
		
		reference operator*() const noexcept { return *m_p; }

	private:
		pointer m_p = nullptr;
	};

	class const_iterator: public std::iterator<std::random_access_iterator_tag, const_value_type>
	{
	public:
		const_iterator() = default;
		const_iterator(const_pointer p): m_p{p} {}

		const_iterator& operator++() noexcept { ++m_p; return *this; }
		const_iterator operator++(int) { iterator tmp(*this); operator++(); return tmp; }
		
		constexpr bool operator==(const const_iterator& rhs) const noexcept {return m_p == rhs.m_p; }
		constexpr bool operator!=(const const_iterator& rhs) const noexcept {return m_p != rhs.m_p; }
		
		reference operator*() const noexcept { return *m_p; }

	private:
		const_pointer m_p = nullptr;
	};

	AllocatedArray(size_type capacity, pointer data, deleter_type deleter = nullptr):
		m_capacity{capacity}, m_data{data}, m_deleter{deleter}
	{}
	
	~AllocatedArray() noexcept
	{
		m_deleter(m_data);
	}

	size_type size() const noexcept { return m_size; }
	size_type max_size() const noexcept { return m_capacity; }
	bool empty() const noexcept { return size() == 0; }

	pointer data() const noexcept { return m_data; }

	reference operator[](size_type i)
	{
		assert(i < m_size);
		return m_data[i];
	}

	const_reference operator[](size_type i) const
	{
		assert(i < m_size);
		return m_data[i];
	}

	iterator begin() { return iterator{&m_data[0]}; }
	iterator end() { return iterator{&m_data[0] + size()}; }
	const_iterator begin() const { return const_iterator{&m_data[0]}; }
	const_iterator end() const { return const_iterator{&m_data[0] + size()}; }
	
	const_iterator cbegin() const { return const_iterator{&m_data[0]}; }
	const_iterator cend() const { return const_iterator{&m_data[0] + size()}; }

	reference back() noexcept
	{
		assert(m_size > 0);
		return m_data[m_size - 1];		
	}

	const_reference back() const noexcept
	{
		assert(m_size > 0);
		return m_data[m_size - 1];
	}

	void clear() noexcept
	{
		m_size = 0;
	}

	void push_back(const_reference value)
	{
		assert(m_size < m_capacity);
		m_data[m_size] = value;
		++m_size;
	}
	
	void pop_back() noexcept
	{
		assert(m_size > 0);
		--m_size;
	}

private:
	static void deletor(void *) {}

	size_type m_capacity = 0;
	size_type m_size = 0;
	pointer m_data = nullptr;
	deleter_type m_deleter;
};

}; // namespace box2d

#endif /* AllocatedArray_hpp */
