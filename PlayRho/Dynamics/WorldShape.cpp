/*
 * Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/WorldShape.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp> // for MixFriction

#include <set>

namespace playrho {
namespace d2 {

using playrho::size;

ShapeCounter GetShapeRange(const World& world) noexcept
{
    return world.GetShapeRange();
}

ShapeID CreateShape(World& world, const Shape& def)
{
    return world.CreateShape(def);
}

void Destroy(World& world, ShapeID id)
{
    world.Destroy(id);
}

const Shape& GetShape(const World& world, ShapeID id)
{
    return world.GetShape(id);
}

void SetShape(World& world, ShapeID id, const Shape& def)
{
    world.SetShape(id, def);
}

ShapeCounter GetAssociationCount(const World& world) noexcept
{
    auto sum = ShapeCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world,&sum](const auto &b) {
        sum += static_cast<ShapeCounter>(size(world.GetShapes(b)));
    });
    return sum;
}

ShapeCounter GetUsedShapesCount(const World& world) noexcept
{
    auto ids = std::set<ShapeID>{};
    for (auto&& bodyId: world.GetBodies()) {
        for (auto&& shapeId: world.GetShapes(bodyId)) {
            ids.insert(shapeId);
        }
    }
    return static_cast<ShapeCounter>(std::size(ids));
}

void SetFilterData(World& world, ShapeID id, const Filter& value)
{
    auto object = world.GetShape(id);
    SetFilter(object, value);
    world.SetShape(id, object);
}

void SetSensor(World& world, ShapeID id, bool value)
{
    auto object = world.GetShape(id);
    SetSensor(object, value);
    world.SetShape(id, object);
}

bool TestPoint(const World& world, BodyID bodyId, ShapeID shapeId, Length2 p)
{
    return TestPoint(GetShape(world, shapeId), InverseTransform(p, GetTransformation(world, bodyId)));
}

Real GetDefaultFriction(const Shape& a, const Shape& b)
{
    return MixFriction(GetFriction(a), GetFriction(b));
}

Real GetDefaultRestitution(const Shape& a, const Shape& b)
{
    return MixRestitution(GetRestitution(a), GetRestitution(b));
}

} // namespace d2
} // namespace playrho
