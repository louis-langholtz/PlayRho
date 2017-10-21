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

#ifndef PLAYRHO_COLLISION_SHAPES_SHAPEVISITOR_HPP
#define PLAYRHO_COLLISION_SHAPES_SHAPEVISITOR_HPP

namespace playrho {

class DiskShape;
class EdgeShape;
class PolygonShape;
class ChainShape;
class MultiShape;

/// @brief Vistor interface for Shape instances.
///
/// @details Interface to inherit from for objects wishing to "visit" shapes.
///   This uses the visitor design pattern.
/// @sa https://en.wikipedia.org/wiki/Visitor_pattern .
///
class ShapeVisitor
{
public:
    virtual ~ShapeVisitor() = default;
    
    /// @brief Visits a DiskShape.
    virtual void Visit(const DiskShape& shape) = 0;
    
    /// @brief Visits an EdgeShape.
    virtual void Visit(const EdgeShape& shape) = 0;
    
    /// @brief Visits a PolygonShape.
    virtual void Visit(const PolygonShape& shape) = 0;
    
    /// @brief Visits a ChainShape.
    virtual void Visit(const ChainShape& shape) = 0;
    
    /// @brief Visits a MultiShape.
    virtual void Visit(const MultiShape& shape) = 0;
    
protected:
    ShapeVisitor() = default;

    /// @brief Copy constructor.
    ShapeVisitor(const ShapeVisitor& other) = default;

    /// @brief Move constructor.
    ShapeVisitor(ShapeVisitor&& other) = default;
    
    /// @brief Copy assignment operator.
    ShapeVisitor& operator= (const ShapeVisitor& other) = default;

    /// @brief Move assignment operator.
    ShapeVisitor& operator= (ShapeVisitor&& other) = default;
};

/// @brief A ShapeVisitor implementation that reports whether it's been visited.
/// @sa ShapeVisitor
class IsVisitedShapeVisitor: public ShapeVisitor
{
public:
    
    IsVisitedShapeVisitor() = default;
    
    /// @brief Copy constructor.
    IsVisitedShapeVisitor(const IsVisitedShapeVisitor& other) = default;

    /// @brief Move constructor.
    IsVisitedShapeVisitor(IsVisitedShapeVisitor&& other) = default;

    ~IsVisitedShapeVisitor() override = default;
    
    /// @brief Copy assignment operator.
    IsVisitedShapeVisitor& operator= (const IsVisitedShapeVisitor& other) = default;
    
    /// @brief Move assignment operator.
    IsVisitedShapeVisitor& operator= (IsVisitedShapeVisitor&& other) = default;

    /// @brief Visits a DiskShape.
    void Visit(const DiskShape& /*shape*/) override
    {
        visited = true;
    }
    
    /// @brief Visits an EdgeShape.
    void Visit(const EdgeShape& /*shape*/) override
    {
        visited = true;
    }
    
    /// @brief Visits a PolygonShape.
    void Visit(const PolygonShape& /*shape*/) override
    {
        visited = true;
    }
    
    /// @brief Visits a ChainShape.
    void Visit(const ChainShape& /*shape*/) override
    {
        visited = true;
    }
    
    /// @brief Visits a MultiShape.
    void Visit(const MultiShape& /*shape*/) override
    {
        visited = true;
    }
    
    /// @brief Whether this visitor has been visited.
    bool IsVisited() const noexcept
    {
        return visited;
    }
    
private:
    bool visited = false; ///< Visited flag.
};

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_SHAPEVISITOR_HPP
