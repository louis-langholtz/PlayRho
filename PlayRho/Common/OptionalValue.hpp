/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef OptionalValue_hpp
#define OptionalValue_hpp

#include <cassert>

namespace playrho {
    
    /// @brief Optional value template class.
    /// @details An implementation of the optional value type idea.
    /// @note This is meant to be functionally compatible with C++17's std::optional.
    /// @note Use of this type directly is discouraged. Use the Optional type alias instead.
    template<typename T>
    class OptionalValue
    {
    public:
        using value_type = T;
        
        constexpr OptionalValue() = default;
        constexpr OptionalValue(const OptionalValue& other) = default;
        constexpr OptionalValue(const T v);
        
        constexpr const T& operator*() const;
        constexpr T& operator*();
        constexpr const T* operator->() const;
        constexpr T* operator->();

        constexpr explicit operator bool() const noexcept;
        constexpr bool has_value() const noexcept;
        
        OptionalValue& operator=(const OptionalValue& other) = default;
        OptionalValue& operator=(const T v);

        constexpr T& value();
        constexpr const T& value() const;
        
        constexpr T value_or(const T& alt) const;
        
    private:
        value_type m_value = value_type{};
        bool m_set = false;
    };
    
    template<typename T>
    constexpr OptionalValue<T>::OptionalValue(const T v): m_value{v}, m_set{true} {}
    
    template<typename T>
    constexpr bool OptionalValue<T>::has_value() const noexcept
    {
        return m_set;
    }
    
    template<typename T>
    constexpr OptionalValue<T>::operator bool() const noexcept
    {
        return m_set;
    }
    
    template<typename T>
    OptionalValue<T>& OptionalValue<T>::operator=(const T v)
    {
        m_value = v;
        m_set = true;
        return *this;
    }
    
    template<typename T>
    constexpr const T* OptionalValue<T>::operator->() const
    {
        assert(m_set);
        return &m_value;
    }
    
    template<typename T>
    constexpr T* OptionalValue<T>::operator->()
    {
        assert(m_set);
        return &m_value;
    }

    template<typename T>
    constexpr const T& OptionalValue<T>::operator*() const
    {
        assert(m_set);
        return m_value;
    }
    
    template<typename T>
    constexpr T& OptionalValue<T>::operator*()
    {
        assert(m_set);
        return m_value;
    }

    template<typename T>
    constexpr T& OptionalValue<T>::value()
    {
        return m_value;
    }
    
    template<typename T>
    constexpr const T& OptionalValue<T>::value() const
    {
        return m_value;
    }

    template<typename T>
    constexpr T OptionalValue<T>::value_or(const T& alt) const
    {
        return m_set? m_value: alt;
    }
    
    /// @brief Optional type alias.
    /// @details An alias setup to facilitate switching between implementations of the
    ///   optional type idea.
    /// @note This is meant to be used directly for optional values.
    template <typename T>
    using Optional = OptionalValue<T>;
    
} // namespace playrho

#endif /* OptionalValue_hpp */
