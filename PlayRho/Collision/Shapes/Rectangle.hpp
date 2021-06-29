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

#ifndef PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP
#define PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Units.hpp>
#include <PlayRho/Common/InvalidArgument.hpp>
#include <PlayRho/Common/NonNegative.hpp>
#include <PlayRho/Common/Finite.hpp>
#include <PlayRho/Common/Settings.hpp>

#include <PlayRho/Dynamics/Filter.hpp>

#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>

#include <array>

namespace playrho::shape_part {

/// @brief Static friction.
template <int F = 0>
struct StaticFriction {
    /// @brief Friction of the shape.
    static constexpr auto friction = NonNegative<Real>(F);
};

/// @brief Dynamic friction.
template <int F = 0>
struct DynamicFriction {
    /// @brief Friction of the shape.
    NonNegative<Real> friction = NonNegative<Real>(F);
};

/// @brief Static tenths friction.
/// @note This is a special template class for achieving fractional frictions with pre C++20
///   versions of C++ that don't yet support float and double template parameters.
template <int F = 2>
struct StaticTenthsFriction {
    /// @brief Friction of the shape.
    static constexpr auto friction = NonNegative<Real>{Real(F) / Real{10}};
};

/// @brief Static restitution policy class.
template <int R = 0>
struct StaticRestitution {
    /// @brief Restitution of the shape.
    static inline const auto restitution = Finite<Real>(R);
};

/// @brief Dynamic restitution policy class.
template <int R = 0>
struct DynamicRestitution {
    /// @brief Restitution of the shape.
    Finite<Real> restitution = Finite<Real>(R);
};

/// @brief Static area density policy class.
template <int D = 0>
struct StaticAreaDensity {
    /// @brief Areal density of the shape (for use with 2D shapes).
    static constexpr auto density = NonNegative<AreaDensity>{Real(D) * KilogramPerSquareMeter};
};

/// @brief Dynamic area density policy class.
template <int D = 0>
struct DynamicAreaDensity {
    /// @brief Areal density of the shape (for use with 2D shapes).
    NonNegative<AreaDensity> density = NonNegative<AreaDensity>{Real(D) * KilogramPerSquareMeter};
};

/// @brief Static multiples of linear slop vertex radius policy class.
template <int V = 2>
struct StaticLinearSlopVertexRadius {
    /// @brief Vertex radius of the shape.
    static constexpr auto vertexRadius = NonNegative<Length>{Real(V) * DefaultLinearSlop};
};

/// @brief Dynamic vertex radius policy class.
template <int V = 2>
struct DynamicVertexRadius {
    /// @brief Vertex radius of the shape.
    NonNegative<Length> vertexRadius = NonNegative<Length>{Real(V) * DefaultLinearSlop};
};

/// @brief Static filter policy class.
template <Filter::bits_type CategoryBits = 1, Filter::bits_type MaskBits = 0xFFFF,
          Filter::index_type GroupIndex = 0>
struct StaticFilter {
    /// @brief The filter of the shape.
    static inline const auto filter = Filter{CategoryBits, MaskBits, GroupIndex};
};

/// @brief Dynamic filter policy class.
template <Filter::bits_type CategoryBits = 1, Filter::bits_type MaskBits = 0xFFFF,
          Filter::index_type GroupIndex = 0>
struct DynamicFilter {
    /// @brief The filter of the shape.
    Filter filter = Filter{CategoryBits, MaskBits, GroupIndex};
};

/// @brief Static sensor policy class.
template <bool V = false>
struct StaticSensor {
    /// @brief Sensor property of the shape.
    static constexpr auto sensor = V;
};

/// @brief Dynamic sensor policy class.
template <bool V = false>
struct DynamicSensor {
    /// @brief Sensor property of the shape.
    bool sensor = V;
};

/// @brief "Discriminator" for named template arguments.
/// @note "[This allows] the various setter types to be identical. (You cannot have multiple direct
///   base classes of the same type. Indirect base classes, on the other hand, can have types that
///   are identical to those of other bases.)"
/// @see https://flylib.com/books/en/3.401.1.126/1/
template <class Base, int D>
struct Discriminator : Base {
};

/// @brief Policy selector for named template arguments.
/// @see https://flylib.com/books/en/3.401.1.126/1/
template <class Setter1, class Setter2, class Setter3, class Setter4, class Setter5, class Setter6>
struct PolicySelector : Discriminator<Setter1, 1>, //
                        Discriminator<Setter2, 2>, //
                        Discriminator<Setter3, 3>, //
                        Discriminator<Setter4, 4>, //
                        Discriminator<Setter5, 5>, //
                        Discriminator<Setter6, 6> //
{
};

/// @brief Default policies for the <code>Rectangle</code> template class.
struct DefaultPolicies {
    /// @brief Alias of the density policy.
    using Density = StaticAreaDensity<>;

    /// @brief Alias of the friction policy.
    using Friction = StaticTenthsFriction<>;

    /// @brief Alias of the restitution policy.
    using Restitution = StaticRestitution<>;

    /// @brief Alias of the vertex radius policy.
    using VertexRadius = StaticLinearSlopVertexRadius<>;

    /// @brief Alias of the filter policy.
    using Filter = StaticFilter<>;

    /// @brief Alias of the sensor policy.
    using Sensor = StaticSensor<>;
};

/// @brief Default policy arguments for the <code>Rectangle</code> template class.
struct DefaultPolicyArgs : virtual DefaultPolicies {
};

/// @brief Sets the alias for the density policy.
template <class Policy>
struct DensityIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::Density
    using Density = Policy;
};

/// @brief Sets the alias for the friction policy.
template <class Policy>
struct FrictionIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::Friction
    using Friction = Policy;
};

/// @brief Sets the alias for the restitution policy.
template <class Policy>
struct RestitutionIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::Restitution
    using Restitution = Policy;
};

/// @brief Sets the alias for the vertex radius policy.
template <class Policy>
struct VertexRadiusIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::VertexRadius
    using VertexRadius = Policy;
};

/// @brief Sets the alias for the filter policy.
template <class Policy>
struct FilterIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::Filter
    using Filter = Policy;
};

/// @brief Sets the alias for the sensor policy.
template <class Policy>
struct SensorIs : virtual DefaultPolicies {
    /// @copydoc DefaultPolicies::Sensor
    using Sensor = Policy;
};

} // namespace playrho::shape_part

namespace playrho::d2 {

/// @brief Whether or not an associated shape's geometry (its position and dimensions)
///   is mutable.
enum class Geometry {
    Constant,
    Mutable,
};

/// @brief Templated rectangular shape part.
/// @ingroup PartsGroup
template <Geometry R, int W = 0, int H = 0, // force break
          class P1 = shape_part::DefaultPolicyArgs, // force break
          class P2 = shape_part::DefaultPolicyArgs, // force break
          class P3 = shape_part::DefaultPolicyArgs, // force break
          class P4 = shape_part::DefaultPolicyArgs, // force break
          class P5 = shape_part::DefaultPolicyArgs, // force break
          class P6 = shape_part::DefaultPolicyArgs>
class Rectangle;

/// @brief A statically sized rectangle shape template specialization taking zero or more policy
///   classes.
/// @ingroup PartsGroup
template <int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
class Rectangle<Geometry::Constant, W, H, P1, P2, P3, P4, P5, P6> // break
    : public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Density, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Friction, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Restitution, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::VertexRadius, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Filter, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Sensor // break
{
    /// @brief Normals of the rectangle.
    static constexpr auto normals = std::array<UnitVec, 4u>{
        UnitVec::GetRight(), UnitVec::GetTop(), UnitVec::GetLeft(), UnitVec::GetBottom()};

    /// @brief Vertices of the rectangle.
    static constexpr auto vertices =
        std::array<Length2, 4u>{Length2{+(W * Meter) / 2, -(H* Meter) / 2}, //
                                Length2{+(W * Meter) / 2, +(H* Meter) / 2}, //
                                Length2{-(W * Meter) / 2, +(H* Meter) / 2}, //
                                Length2{-(W * Meter) / 2, -(H* Meter) / 2}};

public:
    /// @brief Gets the dimensions of this rectangle.
    /// @see SetDimensions.
    constexpr Length2 GetDimensions() const noexcept
    {
        return Length2{GetX(vertices[0]) - GetX(vertices[2]),
                       GetY(vertices[2]) - GetY(vertices[0])};
    }

    /// @brief Sets the dimensions of this rectangle.
    /// @throws InvalidArgument If called to change the dimensions.
    /// @see GetDimensions.
    void SetDimensions(Length2 val)
    {
        if (GetDimensions() != val) {
            throw InvalidArgument("changing dimensions not supported");
        }
    }

    /// @brief Gets the x and y offset of this rectangle.
    /// @see SetOffset.
    Length2 GetOffset() const noexcept
    {
        return Length2{(GetX(vertices[0]) + GetX(vertices[2])) / 2,
                       (GetY(vertices[0]) + GetY(vertices[2])) / 2};
    }

    /// @brief Sets the x and y offset of this rectangle.
    /// @throws InvalidArgument If called to change the offset.
    /// @see GetOffset.
    void SetOffset(Length2 val)
    {
        if (GetOffset() != val) {
            throw InvalidArgument("changing offset not supported");
        }
    }

    /// @brief Gets this rectangle's vertices.
    /// @see GetNormals.
    const std::array<Length2, 4u>& GetVertices() const noexcept
    {
        return vertices;
    }

    /// @brief Gets this rectangle's normals.
    /// @see GetVertices.
    const std::array<UnitVec, 4u>& GetNormals() const noexcept
    {
        return normals;
    }
};

/// @brief A dynamically sized rectangle shape template specialization taking zero or more policy
///   classes.
/// @ingroup PartsGroup
template <int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
class Rectangle<Geometry::Mutable, W, H, P1, P2, P3, P4, P5, P6> // break
    : public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Density, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Friction, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Restitution, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::VertexRadius, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Filter, // break
      public shape_part::PolicySelector<P1, P2, P3, P4, P5, P6>::Sensor // break
{
    /// @brief Normals of the rectangle.
    static constexpr auto normals = std::array<UnitVec, 4u>{
        UnitVec::GetRight(), UnitVec::GetTop(), UnitVec::GetLeft(), UnitVec::GetBottom()};

    /// @brief Vertices of the rectangle.
    std::array<Length2, 4u> vertices =
        std::array<Length2, 4u>{Length2{+(W * Meter) / 2, -(H* Meter) / 2}, //
                                Length2{+(W * Meter) / 2, +(H* Meter) / 2}, //
                                Length2{-(W * Meter) / 2, +(H* Meter) / 2}, //
                                Length2{-(W * Meter) / 2, -(H* Meter) / 2}};

public:
    Rectangle() = default;

    /// @brief Initializing constructor.
    Rectangle(Length width, Length height, Length2 offset = Length2{})
        : vertices{Length2{+width / 2, -height / 2} + offset, //
                   Length2{+width / 2, +height / 2} + offset, //
                   Length2{-width / 2, +height / 2} + offset, //
                   Length2{-width / 2, -height / 2} + offset}
    {
        // Intentionally empty.
    }

    /// @brief Gets the dimensions of this rectangle.
    /// @see SetDimensions.
    Length2 GetDimensions() const noexcept
    {
        return Length2{GetX(vertices[0]) - GetX(vertices[2]),
                       GetY(vertices[2]) - GetY(vertices[0])};
    }

    /// @brief Sets the dimensions of this rectangle.
    /// @see GetDimensions.
    void SetDimensions(Length2 val)
    {
        if (GetDimensions() != val) {
            const auto offset = GetOffset();
            vertices = {Length2{+GetX(val) / 2, -GetY(val) / 2} + offset, //
                        Length2{+GetX(val) / 2, +GetY(val) / 2} + offset, //
                        Length2{-GetX(val) / 2, +GetY(val) / 2} + offset, //
                        Length2{-GetX(val) / 2, -GetY(val) / 2} + offset};
        }
    }

    /// @brief Gets the x and y offset of this rectangle.
    /// @see SetOffset.
    Length2 GetOffset() const noexcept
    {
        return Length2{(GetX(vertices[0]) + GetX(vertices[2])) / 2,
                       (GetY(vertices[0]) + GetY(vertices[2])) / 2};
    }

    /// @brief Sets the x and y offset of this rectangle.
    /// @see GetOffset.
    void SetOffset(Length2 val)
    {
        if (GetOffset() != val) {
            const auto dims = GetDimensions();
            vertices = {Length2{+GetX(dims) / 2, -GetY(dims) / 2} + val, //
                        Length2{+GetX(dims) / 2, +GetY(dims) / 2} + val, //
                        Length2{-GetX(dims) / 2, +GetY(dims) / 2} + val, //
                        Length2{-GetX(dims) / 2, -GetY(dims) / 2} + val};
        }
    }
    /// @brief Gets this rectangle's vertices.
    /// @see GetNormals, SetDimensions, SetOffset.
    const std::array<Length2, 4u>& GetVertices() const noexcept
    {
        return vertices;
    }

    /// @brief Gets this rectangle's normals.
    /// @see GetVertices.
    const std::array<UnitVec, 4u>& GetNormals() const noexcept
    {
        return normals;
    }
};

/// @brief Gets the rectangle's width and height dimensions.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr Length2 GetDimensions(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.GetDimensions();
}

/// @brief Sets the rectangle's width and height dimensions.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
void SetDimensions(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Length2 value)
{
    arg.SetDimensions(value);
}

/// @brief Gets the rectangle's x and y offset.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr Length2 GetOffset(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.GetOffset();
}

/// @brief Sets the rectangle's x and y offset.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
void SetOffset(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Length2 value)
{
    arg.SetOffset(value);
}

/// @brief Gets the "child" count for the given shape configuration.
/// @return 1.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr ChildCounter GetChildCount(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>&) noexcept
{
    return 1;
}

/// @brief Gets the "child" shape for the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
DistanceProxy GetChild(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, ChildCounter index)
{
    if (index != 0) {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{arg.vertexRadius, static_cast<VertexCounter>(size(arg.GetVertices())),
                         data(arg.GetVertices()), data(arg.GetNormals())};
}

/// @brief Gets the density of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr NonNegative<AreaDensity>
GetDensity(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.density;
}

/// @brief Gets the restitution of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr Finite<Real>
GetRestitution(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.restitution;
}

/// @brief Gets the friction of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr NonNegative<Real>
GetFriction(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.friction;
}

/// @brief Gets the filter of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr Filter GetFilter(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.filter;
}

/// @brief Gets the is-sensor state of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr bool IsSensor(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.sensor;
}

/// @brief Gets the vertex radius of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
constexpr NonNegative<Length>
GetVertexRadius(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return arg.vertexRadius;
}

/// @brief Gets the vertex radius of the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
NonNegative<Length> GetVertexRadius(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg,
                                    ChildCounter) noexcept
{
    return GetVertexRadius(arg);
}

/// @brief Gets the mass data for the given shape configuration.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
MassData GetMassData(const Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg) noexcept
{
    return playrho::d2::GetMassData(arg.vertexRadius, arg.density,
                                    Span<const Length2>(arg.GetVertices()));
}

/// @brief Transforms the given polygon configuration's vertices by the given
///   transformation matrix.
/// @relatedalso Rectangle
/// @see https://en.wikipedia.org/wiki/Transformation_matrix
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
void Transform(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>&, const Mat22& m)
{
    if (m != GetIdentity<Mat22>()) {
        throw InvalidArgument("transformation by non-identity matrix not supported");
    }
}

/// @brief Density setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().density)>,
    void>
SetDensity(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, NonNegative<AreaDensity> value)
{
    if (value != GetDensity(arg)) {
        throw InvalidArgument("SetDensity by non-equivalent value not supported");
    }
}

/// @brief Density setter.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    !std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().density)>,
    void>
SetDensity(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, NonNegative<AreaDensity> value)
{
    arg.density = value;
}

/// @brief Filter setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().filter)>,
    void>
SetFilter(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Filter value)
{
    if (value != GetFilter(arg)) {
        throw InvalidArgument("SetFilter by non-equivalent filter not supported");
    }
}

/// @brief Filter setter.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    !std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().filter)>,
    void>
SetFilter(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Filter value)
{
    arg.filter = value;
}

/// @brief Sensor setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().sensor)>,
    void>
SetSensor(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, bool value)
{
    if (value != IsSensor(arg)) {
        throw InvalidArgument("SetSensor by non-equivalent value not supported");
    }
}

/// @brief Sensor setter.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    !std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().sensor)>,
    void>
SetSensor(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, bool value)
{
    arg.sensor = value;
}

/// @brief Friction setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().friction)>,
    void>
SetFriction(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Real value)
{
    if (value != GetFriction(arg)) {
        throw InvalidArgument("SetFriction by non-equivalent value not supported");
    }
}

/// @brief Sets friction.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    !std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().friction)>,
    void>
SetFriction(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Real value)
{
    arg.friction = value;
}

/// @brief Restitution setter that throws unless given the same value as current.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<std::is_const_v<decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>()
                                              .restitution)>,
                 void>
SetRestitution(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Real value)
{
    if (value != GetRestitution(arg)) {
        throw InvalidArgument("SetRestitution by non-equivalent value not supported");
    }
}

/// @brief Sets restitution.
/// @relatedalso Rectangle
template <Geometry R, int W, int H, class P1, class P2, class P3, class P4, class P5, class P6>
std::enable_if_t<
    !std::is_const_v<
        decltype(std::declval<Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>>().restitution)>,
    void>
SetRestitution(Rectangle<R, W, H, P1, P2, P3, P4, P5, P6>& arg, Real value)
{
    arg.restitution = value;
}

/// @brief Equality operator.
/// @relatedalso Rectangle
template <Geometry R1, int W1, int H1, //
          class P11, class P12, class P13, class P14, class P15, class P16, //
          Geometry R2, int W2, int H2, //
          class P21, class P22, class P23, class P24, class P25, class P26>
bool operator==(const Rectangle<R1, W1, H1, P11, P12, P13, P14, P15, P16>& lhs,
                const Rectangle<R2, W2, H2, P21, P22, P23, P24, P25, P26>& rhs) noexcept
{
    return GetDimensions(lhs) == GetDimensions(rhs) && // force break
           GetOffset(lhs) == GetOffset(rhs) && // force break
           GetDensity(lhs) == GetDensity(rhs) && // force break
           GetFriction(lhs) == GetFriction(rhs) && // force break
           GetRestitution(lhs) == GetRestitution(rhs) && // force break
           GetVertexRadius(lhs) == GetVertexRadius(rhs) && // force break
           GetFilter(lhs) == GetFilter(rhs) && // force break
           IsSensor(lhs) == IsSensor(rhs);
}

/// @brief Inequality operator.
/// @relatedalso Rectangle
template <Geometry R1, int W1, int H1, //
          class P11, class P12, class P13, class P14, class P15, class P16, //
          Geometry R2, int W2, int H2, //
          class P21, class P22, class P23, class P24, class P25, class P26>
bool operator!=(const Rectangle<R1, W1, H1, P11, P12, P13, P14, P15>& lhs,
                const Rectangle<R2, W2, H2, P21, P22, P23, P24, P25>& rhs) noexcept
{
    return !(lhs == rhs);
}

} // namespace playrho::d2

#endif // PLAYRHO_COLLISION_SHAPES_RECTANGLE_HPP
