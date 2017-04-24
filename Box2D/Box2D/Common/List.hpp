/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef List_hpp
#define List_hpp

#include <iterator>
#include <Box2D/Common/Settings.hpp>

namespace box2d {

	template <typename T>
	struct ListNode
	{
		using pointer = ListNode<T>*;
		using reference = ListNode<T>&;
		using const_pointer = const ListNode<T>*;
		using const_reference = const ListNode<T>&;

		/// Iterator.
		/// @details
		/// Conforming to the bidirectional iterator concept.
		/// @sa http://en.cppreference.com/w/cpp/concept/BidirectionalIterator
		class iterator
		{
		public:
			iterator() = default;
			
			constexpr iterator(const iterator& it) noexcept: p{it.p} {}
			
			constexpr explicit iterator(pointer b) noexcept: p{b} {}
			
			iterator& operator++() noexcept { p = p->next; return *this; }
			iterator operator++(int) { iterator tmp(*this); operator++(); return tmp; }
			
			iterator& operator--() noexcept { p = p->prev; return *this; }
			iterator operator--(int) { iterator tmp(*this); operator--(); return tmp; }
			
			constexpr bool operator==(const iterator& rhs) const noexcept {return p == rhs.p; }
			constexpr bool operator!=(const iterator& rhs) const noexcept {return p != rhs.p; }
			
			reference operator*() const noexcept { return *p; }
			pointer operator->() const noexcept { return p; }
			
		private:
			pointer p;
		};
		
		class const_iterator
		{
		public:
	
			const_iterator() = default;
			
			constexpr const_iterator(const const_iterator& it) noexcept: p{it.p} {}
			
			constexpr explicit const_iterator(const pointer b) noexcept: p{b} {}
			
			const_iterator& operator++() noexcept { p = p->next; return *this; }
			const_iterator operator++(int) { const_iterator tmp(*this); operator++(); return tmp; }
			
			const_iterator& operator--() noexcept { p = p->prev; return *this; }
			const_iterator operator--(int) { const_iterator tmp(*this); operator--(); return tmp; }
			
			constexpr bool operator==(const const_iterator& rhs) const noexcept {return p == rhs.p; }
			constexpr bool operator!=(const const_iterator& rhs) const noexcept {return p != rhs.p; }
			
			const_reference operator*() const noexcept { return *p; }
			const_pointer operator->() const noexcept { return p; }
			
		private:
			const_pointer p;
		};
		
		static inline void push(pointer& p, pointer elem)
		{
			elem->prev = nullptr;
			elem->next = p;
			if (p)
			{
				p->prev = elem;
			}
			p = elem;
		}

		static inline void remove(pointer p)
		{
			if (p->prev)
			{
				p->prev->next = p->next;
			}
			if (p->next)
			{
				p->next->prev = p->prev;
			}
		}

		static inline void erase(pointer& p, pointer elem)
		{
			remove(elem);
			if (p == elem)
			{
				p = elem->next;
			}
		}

		static inline void pop(pointer& p)
		{
			remove(p);
			p = p->next;
		}

		pointer prev;
		pointer next;
		T data;
	};

	template <typename T>
	class InternalList
	{
	public:
						
		using pointer = ListNode<T>*;
		using reference = ListNode<T>&;
		using const_reference = const ListNode<T>&;

		using size_type = std::size_t;

		using iterator = typename ListNode<T>::iterator;
		using const_iterator = typename ListNode<T>::const_iterator;
		
		InternalList() = default;
		
		constexpr InternalList(const InternalList& copy) = delete;
		
		InternalList& operator= (const InternalList& rhs) = delete;

		InternalList& operator= (const InternalList&& rhs) = delete;
		
		iterator begin() noexcept { return iterator{p}; }
		iterator end() noexcept { return iterator{nullptr}; }

		const_iterator begin() const noexcept { return const_iterator{p}; }
		const_iterator end() const noexcept { return const_iterator{nullptr}; }
		
		const_iterator cbegin() const noexcept { return const_iterator{p}; }
		const_iterator cend() const noexcept { return const_iterator{nullptr}; }
		
		constexpr bool empty() const noexcept { return p == nullptr; }
		size_type size() const noexcept { return n; }

		constexpr size_type max_size() const noexcept { return max_list_size<T>(); }
		
		constexpr bool operator== (const InternalList& rhs) const noexcept { return p == rhs.p; }
		constexpr bool operator!= (const InternalList& rhs) const noexcept { return p != rhs.p; }
		
		reference front() noexcept { return *p; }
		const_reference front() const noexcept { return *p; }

		InternalList(InternalList&& other): p{std::move(other.p)}, n{std::move(other.n)}
		{
			other.p = nullptr;
			other.n = 0;
		}
		
		void push_front(pointer value) noexcept
		{
			assert(n < max_size());
			assert(value->prev == nullptr);
			assert(value->next == nullptr);
			if (n < max_size())
			{
				ListNode<T>::push(p, value);
				++n;
			}
		}
		
		void pop_front() noexcept
		{
			assert(n > 0);
			if (n > 0)
			{
				ListNode<T>::pop(p);
				--n;
			}
		}
		
		iterator erase(iterator pos)
		{
			assert(n > 0);
			if (n > 0)
			{		
				const auto next = pos.p->next;
				ListNode<T>::erase(p, pos.p);
				--n;
				return iterator{next};
			}
			return pos;
		}

	private:
		pointer p = nullptr;
		size_type n = 0;
	};
	
	template <typename T>
	class List
	{
	public:
		using pointer = T*;
		using reference = T&;
		using const_reference = const T&;
		
		using size_type = std::size_t;

		/// Iterator.
		/// @details
		/// Conforming to the bidirectional iterator concept.
		/// @sa http://en.cppreference.com/w/cpp/concept/BidirectionalIterator
		class iterator
		{
		public:
			using pointer = T*;
			using reference = T&;
			
			iterator() = default;
			
			constexpr iterator(const iterator& it) noexcept: p{it.p} {}
			
			constexpr explicit iterator(typename ListNode<T>::iterator b) noexcept: p{b} {}
			
			iterator& operator++() noexcept { ++p; return *this; }
			iterator operator++(int) { iterator tmp(*this); ++p; return tmp; }
			
			iterator& operator--() noexcept { --p; return *this; }
			iterator operator--(int) { iterator tmp(*this); --p; return tmp; }
			
			constexpr bool operator==(const iterator& rhs) const noexcept {return p == rhs.p; }
			constexpr bool operator!=(const iterator& rhs) const noexcept {return p != rhs.p; }
			
			reference operator*() const noexcept { return p->data; }
			pointer operator->() const noexcept { return &(p->data); }
			
		private:
			typename ListNode<T>::iterator p;
		};
		
		class const_iterator
		{
		public:
			using pointer = const T*;
			using reference = const T&;
			
			const_iterator() = default;
			
			constexpr const_iterator(const const_iterator& it) noexcept: p{it.p} {}
			
			constexpr explicit const_iterator(typename ListNode<T>::const_iterator b) noexcept: p{b} {}
			
			const_iterator& operator++() noexcept { ++p; return *this; }
			const_iterator operator++(int) { const_iterator tmp(*this); ++p; return tmp; }
			
			const_iterator& operator--() noexcept { --p; return *this; }
			const_iterator operator--(int) { const_iterator tmp(*this); --p; return tmp; }
			
			constexpr bool operator==(const const_iterator& rhs) const noexcept {return p == rhs.p; }
			constexpr bool operator!=(const const_iterator& rhs) const noexcept {return p != rhs.p; }
			
			reference operator*() const noexcept { return p->data; }
			pointer operator->() const noexcept { return &(p->data); }
			
		private:
			typename ListNode<T>::const_iterator p;
		};
		
		List(InternalList<T>& list): m_list{list} {}

		constexpr List(const List& copy) = delete;
		
		List& operator= (const List& rhs) = delete;
		
		List& operator= (const List&& rhs) = delete;
		
		iterator begin() noexcept { return iterator{m_list.begin()}; }
		iterator end() noexcept { return iterator{m_list.end()}; }
		
		const_iterator begin() const noexcept { return iterator{m_list.begin()}; }
		const_iterator end() const noexcept { return iterator{m_list.end()}; }
		
		const_iterator cbegin() const noexcept { return const_iterator{m_list.cbegin()}; }
		const_iterator cend() const noexcept { return const_iterator{m_list.cend()}; }
		
		constexpr bool empty() const noexcept { return m_list.empty(); }
		size_type size() const noexcept { return m_list.size(); }
		
		constexpr size_type max_size() const noexcept { return max_list_size<T>(); }
		
		constexpr bool operator== (const List& rhs) const noexcept { return m_list == rhs.m_list; }
		constexpr bool operator!= (const List& rhs) const noexcept { return m_list != rhs.m_list; }
		
		reference front() noexcept { return m_list.front().data; }
		const_reference front() const noexcept { return m_list.front().data; }

	private:
		InternalList<T>& m_list;
	};
	
}; // namespace box2d

#endif /* List_hpp */
