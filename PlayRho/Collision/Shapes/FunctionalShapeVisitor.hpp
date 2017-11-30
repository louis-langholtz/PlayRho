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

#ifndef PLAYRHO_COLLISION_SHAPES_FUNCTIONALSHAPEVISITOR_HPP
#define PLAYRHO_COLLISION_SHAPES_FUNCTIONALSHAPEVISITOR_HPP

#include <PlayRho/Collision/Shapes/ShapeVisitor.hpp>
#include <functional>
#include <tuple>

namespace playrho {
    
    /// @brief Functional shape visitor class.
    /// @note This class is intended to provide an alternate interface for visiting shapes
    ///   via the use of lamdas instead of having to subclass <code>ShapeVisitor</code>.
    /// @sa ShapeVisitor
    class FunctionalShapeVisitor: public ShapeVisitor
    {
    public:
        /// @brief Procedure alias.
        template <class T>
        using Proc = std::function<void(const T&)>;

        /// @brief Tuple alias.
        using Tuple = std::tuple<
            Proc<DiskShape>,
            Proc<EdgeShape>,
            Proc<PolygonShape>,
            Proc<ChainShape>,
            Proc<MultiShape>
        >;
        
        Tuple procs; ///< Procedures.

        /// @brief Uses given procedure.
        /// @note Provide a builder pattern mutator method.
        template <class T>
        FunctionalShapeVisitor& Use(const Proc<T>& proc) noexcept
        {
            std::get<Proc<T>>(procs) = proc;
            return *this;
        }

        // Overrides of all the base class's Visit methods...
        // Uses decltype to ensure the correctly typed invocation of the Handle method.
        void Visit(const DiskShape& arg) override { Handle<decltype(arg)>(arg); }
        void Visit(const EdgeShape& arg) override { Handle<decltype(arg)>(arg); }
        void Visit(const PolygonShape& arg) override { Handle<decltype(arg)>(arg); }
        void Visit(const ChainShape& arg) override { Handle<decltype(arg)>(arg); }
        void Visit(const MultiShape& arg) override { Handle<decltype(arg)>(arg); }

    private:
        
        /// @brief Handles the joint through the established function.
        template <class T>
        inline void Handle(T arg) const
        {
            const auto& proc = std::get<Proc<T>>(procs);
            if (proc)
            {
                proc(arg);
            }
        }
    };
    
} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_FUNCTIONALSHAPEVISITOR_HPP

