/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "UnitTests.hpp"

#include <PlayRho/d2/Shape.hpp>
#include <PlayRho/d2/EdgeShapeConf.hpp>
#include <PlayRho/d2/DiskShapeConf.hpp>
#include <PlayRho/d2/PolygonShapeConf.hpp>
#include <PlayRho/d2/Compositor.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/Manifold.hpp>

#include <any>
#include <chrono>
#include <string>

using namespace playrho;
using namespace playrho::d2;

TEST(Shape, ByteSize)
{
    // Check size at test runtime instead of compile-time via static_assert to avoid stopping
    // builds and to report actual size rather than just reporting that expected size is wrong.
#if SHAPE_USES_UNIQUE_PTR
    EXPECT_EQ(sizeof(Shape), sizeof(std::unique_ptr<int>));
#else
    EXPECT_EQ(sizeof(Shape), sizeof(std::shared_ptr<int>));
#endif
}

TEST(Shape, Traits)
{
    // NOTE: Double parenthesis needed sometimes for proper macro expansion.

    EXPECT_TRUE(std::is_default_constructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_default_constructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_default_constructible<Shape>::value);

    // Construction with any 1 supporting argument should succeed...
    using X = DiskShapeConf;
    EXPECT_TRUE((std::is_constructible<Shape, X>::value));
    EXPECT_FALSE((std::is_nothrow_constructible<Shape, X>::value));
    EXPECT_FALSE((std::is_trivially_constructible<Shape, X>::value));

    // Construction with 2 arguments should fail...
    EXPECT_FALSE((std::is_constructible<Shape, X, X>::value));
    EXPECT_FALSE((std::is_nothrow_constructible<Shape, X, X>::value));
    EXPECT_FALSE((std::is_trivially_constructible<Shape, X, X>::value));

    EXPECT_TRUE(std::is_copy_constructible<Shape>::value);
#if SHAPE_USES_UNIQUE_PTR
    EXPECT_FALSE(std::is_nothrow_copy_constructible<Shape>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_constructible<Shape>::value);
#endif
    EXPECT_FALSE(std::is_trivially_copy_constructible<Shape>::value);

    EXPECT_TRUE(std::is_move_constructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_move_constructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_move_constructible<Shape>::value);

    EXPECT_TRUE(std::is_copy_assignable<Shape>::value);
#if SHAPE_USES_UNIQUE_PTR
    EXPECT_FALSE(std::is_nothrow_copy_assignable<Shape>::value);
#else
    EXPECT_TRUE(std::is_nothrow_copy_assignable<Shape>::value);
#endif
    EXPECT_FALSE(std::is_trivially_copy_assignable<Shape>::value);

    EXPECT_TRUE(std::is_move_assignable<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_move_assignable<Shape>::value);
    EXPECT_FALSE(std::is_trivially_move_assignable<Shape>::value);

    EXPECT_TRUE(std::is_destructible<Shape>::value);
    EXPECT_TRUE(std::is_nothrow_destructible<Shape>::value);
    EXPECT_FALSE(std::is_trivially_destructible<Shape>::value);

    // The value initializing constructor resolves for ineligible types but preferably any such
    // instantiation will not actually compile.
    EXPECT_TRUE((std::is_constructible<Shape, int>::value));
}

TEST(Shape, DefaultConstruction)
{
    EXPECT_EQ(Shape::DefaultDensity, (NonNegative<AreaDensity>{0_kgpm2}));
    const auto s = Shape{};
    EXPECT_FALSE(s.has_value());
    EXPECT_EQ(GetMassData(s), MassData());
    EXPECT_EQ(GetFriction(s), Real(0));
    EXPECT_EQ(GetRestitution(s), Real(0));
    EXPECT_EQ(GetDensity(s), Shape::DefaultDensity);
    EXPECT_THROW(GetVertexRadius(s, 0), InvalidArgument);
    EXPECT_EQ(GetChildCount(s), ChildCounter(0));
    EXPECT_THROW(GetChild(s, 0), InvalidArgument);
    EXPECT_TRUE(s == s);
    auto t = Shape{};
    EXPECT_TRUE(s == t);
    EXPECT_NO_THROW(Translate(t, Length2{}));
    EXPECT_EQ(GetType(s), GetTypeID<void>());
}

namespace {

struct MovableConf {
    static int defaultConstructorCalled;
    static int copyConstructorCalled;
    static int moveConstructorCalled;
    static int copyAssignmentCalled;
    static int moveAssignmentCalled;

    static void resetClass()
    {
        defaultConstructorCalled = 0;
        copyConstructorCalled = 0;
        moveConstructorCalled = 0;
        copyAssignmentCalled = 0;
        moveAssignmentCalled = 0;
    }

    std::string data;

    MovableConf()
    {
        ++defaultConstructorCalled;
    }

    MovableConf(const MovableConf& other): data{other.data}
    {
        ++copyConstructorCalled;
    }

    MovableConf(MovableConf&& other): data{std::move(other.data)}
    {
        ++moveConstructorCalled;
    }

    MovableConf& operator=(const MovableConf& other)
    {
        data = other.data;
        ++copyAssignmentCalled;
        return *this;
    }

    MovableConf& operator=(MovableConf&& other)
    {
        data = std::move(other.data);
        ++moveAssignmentCalled;
        return *this;
    }
};

int MovableConf::defaultConstructorCalled;
int MovableConf::copyConstructorCalled;
int MovableConf::moveConstructorCalled;
int MovableConf::copyAssignmentCalled;
int MovableConf::moveAssignmentCalled;

bool operator==(const MovableConf& lhs, const MovableConf& rhs) noexcept
{
    return lhs.data == rhs.data;
}

ChildCounter GetChildCount(const MovableConf&) noexcept
{
    return 0;
}

DistanceProxy GetChild(const MovableConf&, ChildCounter)
{
    throw InvalidArgument("not supported");
}

MassData GetMassData(const MovableConf&) noexcept
{
    return {};
}

NonNegative<Length> GetVertexRadius(const MovableConf&, ChildCounter)
{
    throw InvalidArgument("not supported");
}

NonNegative<AreaDensity> GetDensity(const MovableConf&) noexcept
{
    return {};
}

Real GetFriction(const MovableConf&) noexcept
{
    return {};
}

Real GetRestitution(const MovableConf&) noexcept
{
    return {};
}

void SetVertexRadius(MovableConf&, ChildCounter, NonNegative<Length>)
{
}

Filter GetFilter(const MovableConf&) noexcept
{
    return {};
}

bool IsSensor(const MovableConf&) noexcept
{
    return {};
}

static_assert(IsValidShapeType<MovableConf>::value, "MovableConf must be a valid shape type");

}

TEST(Shape, ConstructionFromMovable)
{
    MovableConf::resetClass();
    ASSERT_FALSE(MovableConf::copyConstructorCalled);
    ASSERT_FALSE(MovableConf::moveConstructorCalled);
    MovableConf conf;
    conf.data = "have some";
    Shape s{std::move(conf)};
    EXPECT_EQ(std::string(), conf.data);
    EXPECT_EQ(0, MovableConf::copyConstructorCalled);
    EXPECT_EQ(1, MovableConf::moveConstructorCalled);
    EXPECT_EQ(0, MovableConf::copyAssignmentCalled);
    EXPECT_EQ(0, MovableConf::moveAssignmentCalled);
}

TEST(Shape, AssignmentFromMovable)
{
    MovableConf::resetClass();
    ASSERT_FALSE(MovableConf::copyConstructorCalled);
    ASSERT_FALSE(MovableConf::moveConstructorCalled);
    MovableConf conf;
    conf.data = "have some";
    Shape s;
    s = std::move(conf);
    EXPECT_EQ(std::string(), conf.data);
    EXPECT_EQ(0, MovableConf::copyConstructorCalled);
    EXPECT_EQ(1, MovableConf::moveConstructorCalled);
    EXPECT_EQ(0, MovableConf::copyAssignmentCalled);
    EXPECT_EQ(0, MovableConf::moveAssignmentCalled);
}

TEST(Shape, ConstructionFromCopyable)
{
    MovableConf::resetClass();
    ASSERT_FALSE(MovableConf::copyConstructorCalled);
    ASSERT_FALSE(MovableConf::moveConstructorCalled);
    MovableConf conf;
    conf.data = "have some";
    Shape s{conf};
    EXPECT_EQ(std::string("have some"), conf.data);
    EXPECT_EQ(1, MovableConf::copyConstructorCalled);
    EXPECT_EQ(0, MovableConf::moveConstructorCalled);
    EXPECT_EQ(0, MovableConf::copyAssignmentCalled);
    EXPECT_EQ(0, MovableConf::moveAssignmentCalled);
}

TEST(Shape, AssignmentFromCopyable)
{
    MovableConf::resetClass();
    ASSERT_FALSE(MovableConf::copyConstructorCalled);
    ASSERT_FALSE(MovableConf::moveConstructorCalled);
    MovableConf conf;
    conf.data = "have some";
    Shape s;
    s = conf;
    EXPECT_EQ(std::string("have some"), conf.data);
    EXPECT_EQ(1, MovableConf::copyConstructorCalled);
    EXPECT_EQ(0, MovableConf::moveConstructorCalled);
    EXPECT_EQ(0, MovableConf::copyAssignmentCalled);
    EXPECT_EQ(0, MovableConf::moveAssignmentCalled);
}

namespace sans_some {
namespace {

struct ShapeTest {
    int number;
};

[[maybe_unused]] ChildCounter GetChildCount(const ShapeTest&)
{
    return 1u;
}

[[maybe_unused]] void Translate(ShapeTest&, const Length2&) {}

} // namespace
} // namespace sans_some

TEST(Shape, InitializingConstructor)
{
    EXPECT_TRUE((std::is_constructible<Shape, ::sans_some::ShapeTest>::value));
    EXPECT_FALSE(IsValidShapeType<::sans_some::ShapeTest>::value);
    EXPECT_TRUE((std::is_constructible<Shape, DiskShapeConf>::value));
    EXPECT_TRUE(IsValidShapeType<DiskShapeConf>::value);
    auto conf = DiskShapeConf{};
    auto s = Shape{conf};
    EXPECT_TRUE(s.has_value());
    EXPECT_EQ(GetChildCount(s), ChildCounter(1));
    EXPECT_EQ(GetFilter(s).categoryBits, Filter{}.categoryBits);
    EXPECT_EQ(GetFilter(s).maskBits, Filter{}.maskBits);
    EXPECT_EQ(GetFilter(s).groupIndex, Filter{}.groupIndex);
    EXPECT_EQ(IsSensor(s), false);
    conf.UseIsSensor(true);
    s = Shape{conf};
    EXPECT_EQ(IsSensor(s), true);
}

TEST(Shape, Assignment)
{
    auto s = Shape{};
    ASSERT_EQ(GetType(s), GetTypeID<void>());
    ASSERT_EQ(GetChildCount(s), ChildCounter(0));
    ASSERT_EQ(GetFriction(s), Real(0));
    ASSERT_EQ(GetRestitution(s), Real(0));
    ASSERT_EQ(GetDensity(s), 0_kgpm2);

    const auto friction = Real(0.1);
    const auto restitution = Real(0.2);
    const auto density = 0.4_kgpm2;
    s = DiskShapeConf{1_m}.UseFriction(friction).UseRestitution(restitution).UseDensity(density);
    EXPECT_NE(GetType(s), GetTypeID<void>());
    EXPECT_EQ(GetType(s), GetTypeID<DiskShapeConf>());
    EXPECT_EQ(GetChildCount(s), ChildCounter(1));
    EXPECT_EQ(GetFriction(s), friction);
    EXPECT_EQ(GetRestitution(s), restitution);
    EXPECT_EQ(GetDensity(s), density);

    s = EdgeShapeConf();
    EXPECT_NE(GetType(s), GetTypeID<void>());
    EXPECT_EQ(GetType(s), GetTypeID<EdgeShapeConf>());

    // Test copy assignment...
    const auto otherShape = Shape{};
    ASSERT_EQ(GetType(otherShape), GetTypeID<void>());
    s = otherShape;
    EXPECT_EQ(GetType(s), GetTypeID<void>());
    EXPECT_TRUE(s == otherShape);
}

TEST(Shape, TypeCast)
{
    const auto shape = Shape{};
    EXPECT_THROW(TypeCast<int>(shape), std::bad_cast);
}

TEST(Shape, SetVertexRadius)
{
    auto foo = Shape{DiskShapeConf{1_m}};
    const auto radius = NonNegative<Length>(0.42_m);
    EXPECT_NO_THROW(SetVertexRadius(foo, 0, radius));
    EXPECT_EQ(GetVertexRadius(foo, 0), radius);
}

TEST(Shape, ForConstantDataTypeCastIsLikeAnyCast)
{
    const auto foo = Shape{DiskShapeConf{1_m}};
    const auto bar = std::any{DiskShapeConf{1_m}};
    EXPECT_TRUE(TypeCast<const DiskShapeConf*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<const DiskShapeConf*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<DiskShapeConf*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<DiskShapeConf*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<const DiskShapeConf>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<const DiskShapeConf>(&bar) != nullptr);
    EXPECT_TRUE(TypeCast<DiskShapeConf>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<DiskShapeConf>(&bar) != nullptr);
}

TEST(Shape, ForMutableDataTypeCastIsLikeAnyCast)
{
    auto foo = Shape{DiskShapeConf{1_m}};
    auto bar = std::any{DiskShapeConf{1_m}};
    EXPECT_TRUE(TypeCast<const DiskShapeConf*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<const DiskShapeConf*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<DiskShapeConf*>(&foo) == nullptr);
    EXPECT_TRUE(std::any_cast<DiskShapeConf*>(&bar) == nullptr);
    EXPECT_TRUE(TypeCast<const DiskShapeConf>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<const DiskShapeConf>(&bar) != nullptr);
    EXPECT_TRUE(TypeCast<DiskShapeConf>(&foo) != nullptr);
    EXPECT_TRUE(std::any_cast<DiskShapeConf>(&bar) != nullptr);
}

TEST(Shape, types)
{
    using namespace playrho::d2::part;

    EXPECT_EQ(GetTypeID<DiskShapeConf>(), GetTypeID<DiskShapeConf>());

    const auto sc = DiskShapeConf{1_m};
    EXPECT_EQ(GetTypeID(sc), GetTypeID<DiskShapeConf>());
    EXPECT_EQ(GetTypeID<DiskShapeConf>(), GetTypeID(sc));
    EXPECT_EQ(GetTypeID(sc), GetTypeID(sc));
    EXPECT_NE(GetTypeID<DiskShapeConf>(), GetTypeID<EdgeShapeConf>());
    EXPECT_NE(GetTypeID(DiskShapeConf{}), GetTypeID(EdgeShapeConf{}));
    EXPECT_EQ(GetTypeID(DiskShapeConf{}), GetTypeID(DiskShapeConf{}));
    EXPECT_EQ(GetTypeID(EdgeShapeConf{}), GetTypeID(EdgeShapeConf{}));
    EXPECT_EQ(GetTypeID(Compositor<GeometryIs<StaticRectangle<1, 1>>>{}),
              GetTypeID(Compositor<GeometryIs<StaticRectangle<1, 1>>>{}));

    const auto s1 = Shape{sc};
    ASSERT_EQ(GetTypeID<Shape>(), GetTypeID(s1));
    EXPECT_EQ(GetType(s1), GetTypeID<DiskShapeConf>());
    const auto& st1 = GetType(s1);
    ASSERT_NE(st1, GetTypeID<Shape>());
    EXPECT_EQ(st1, GetTypeID(sc));
    EXPECT_EQ(Shape(Compositor<GeometryIs<StaticRectangle<1, 1>>>{}),
              Shape(Compositor<GeometryIs<StaticRectangle<1, 1>>>{}));

    const auto s2 = Shape{s1}; // This should copy construct
    const auto& st2 = GetType(s2);
    EXPECT_EQ(st2, GetTypeID(sc)); // Confirm s2 was a copy construction
}

TEST(Shape, TestOverlapSlowerThanCollideShapesForCircles)
{
    const auto shape = DiskShapeConf{2_m};
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto child = GetChild(shape, 0);

    const auto maxloops = 1000000u;

    std::chrono::duration<double> elapsed_test_overlap;
    std::chrono::duration<double> elapsed_collide_shapes;

    for (auto attempt = 0u; attempt < 2u; ++attempt) {
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i) {
                if (TestOverlap(child, xfm, child, xfm) >= 0_m2) {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_test_overlap = end - start;
            ASSERT_EQ(count, maxloops);
        }
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i) {
                const auto manifold = CollideShapes(child, xfm, child, xfm);
                if (manifold.GetPointCount() > 0) {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_collide_shapes = end - start;
            ASSERT_EQ(count, maxloops);
        }

        EXPECT_GT(elapsed_test_overlap.count(), elapsed_collide_shapes.count());
    }
}

TEST(Shape, TestOverlapFasterThanCollideShapesForPolygons)
{
    const auto shape = PolygonShapeConf{2_m, 2_m};
    const auto xfm = Transformation{Length2{}, UnitVec::GetRight()};
    const auto child = GetChild(shape, 0);

    const auto maxloops = 1000000u;

    std::chrono::duration<double> elapsed_test_overlap;
    std::chrono::duration<double> elapsed_collide_shapes;

    for (auto attempt = 0u; attempt < 2u; ++attempt) {
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i) {
                if (TestOverlap(child, xfm, child, xfm) >= 0_m2) {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_test_overlap = end - start;
            ASSERT_EQ(count, maxloops);
        }
        {
            auto count = 0u;
            const auto start = std::chrono::high_resolution_clock::now();
            for (auto i = decltype(maxloops){0}; i < maxloops; ++i) {
                const auto manifold = CollideShapes(child, xfm, child, xfm);
                if (manifold.GetPointCount() > 0) {
                    ++count;
                }
            }
            const auto end = std::chrono::high_resolution_clock::now();
            elapsed_collide_shapes = end - start;
            ASSERT_EQ(count, maxloops);
        }

        EXPECT_LT(elapsed_test_overlap.count(), elapsed_collide_shapes.count());
    }
}

TEST(Shape, Equality)
{
    EXPECT_TRUE(Shape(EdgeShapeConf()) == Shape(EdgeShapeConf()));
    const auto shapeA = Shape(DiskShapeConf{}.UseRadius(100_m));
    const auto shapeB = Shape(DiskShapeConf{}.UseRadius(100_m));
    EXPECT_TRUE(shapeA == shapeB);
    EXPECT_FALSE(Shape(DiskShapeConf()) == Shape(EdgeShapeConf()));
    EXPECT_FALSE(Shape(EdgeShapeConf()) == Shape(EdgeShapeConf().UseIsSensor(true)));
    const auto filter = Filter{0x2u, 0x8, 0x1};
    EXPECT_FALSE(Shape(EdgeShapeConf()) == Shape(EdgeShapeConf().UseFilter(filter)));
}

TEST(Shape, Inequality)
{
    EXPECT_FALSE(Shape(EdgeShapeConf()) != Shape(EdgeShapeConf()));
    const auto shapeA = Shape(DiskShapeConf{}.UseRadius(100_m));
    const auto shapeB = Shape(DiskShapeConf{}.UseRadius(100_m));
    EXPECT_FALSE(shapeA != shapeB);
    EXPECT_TRUE(Shape(DiskShapeConf()) != Shape(EdgeShapeConf()));
    const auto filter = Filter{0x2u, 0x8, 0x1};
    EXPECT_TRUE(Shape(EdgeShapeConf()) != Shape(EdgeShapeConf().UseFilter(filter)));
}

TEST(Shape, EmptyShapeTranslateIsNoop)
{
    auto s = Shape{};
    EXPECT_NO_THROW(Translate(s, Length2{1_m, 2_m}));
}

TEST(Shape, EmptyShapeScaleIsNoop)
{
    auto s = Shape{};
    EXPECT_NO_THROW(Scale(s, Vec2(Real(2), Real(2))));
}

TEST(Shape, EmptyShapeRotateIsNoop)
{
    auto s = Shape{};
    EXPECT_NO_THROW(Rotate(s, UnitVec::GetTop()));
}

TEST(Shape, EmptyShapeSetVertexRadiusIsNoop)
{
    auto s = Shape{};
    EXPECT_NO_THROW(SetVertexRadius(s, 0, 2_m));
}

TEST(Shape, DynamicRectangleSmallerThanPolygon)
{
    using namespace playrho::d2::part;
    switch (sizeof(Real))
    {
#if defined(_WIN64) || !defined(_WIN32)
    case 4u:
        // Compositor only smaller for 4-byte sized Real values. This is because PolygonShapeConf
        // uses std::vector to store vertices and normals. This hides the full amount of memory it
        // uses from sizeof(PolygonShapeConf). This also means that not all the data that
        // PolygonShapeConf contains is available in one contiguous block of memory which increases
        // the chance of memory cache misses that will slow simulations down. Meanwhile, the memory
        // for the Compositor used here will be entirely contiguous.
        EXPECT_LT(sizeof(Compositor<GeometryIs<DynamicRectangle<>>, //
                         DensityIs<DynamicAreaDensity<>>, //
                         RestitutionIs<DynamicRestitution<>>, //
                         FrictionIs<DynamicFriction<>>, //
                         SensorIs<DynamicSensor<>>, //
                         FilterIs<DynamicFilter<>>>),
                  sizeof(PolygonShapeConf));
        break;
#endif
    default:
        break;
    }
}

#if 0
#include <cstddef> // for std::max_align_t
#include <variant>

class Foo {
    struct Concept {
        virtual ~Concept() = default;
        virtual std::unique_ptr<Concept> Clone_() const = 0;
        virtual void NewTo_(void* buffer) const = 0;
        virtual Real GetFriction_() const noexcept = 0;
    };

    template <typename T>
    struct Model final : Concept {
        using data_type = T;

        /// @brief Initializing constructor.
        Model(T arg) : data{std::move(arg)} {}

        std::unique_ptr<Concept> Clone_() const override
        {
            return std::make_unique<Model>(data);
        }

        void NewTo_(void* buffer) const override
        {
            ::new (buffer) Model(data);
        }

        Real GetFriction_() const noexcept override
        {
            return GetFriction(data);
        }

        data_type data; ///< Data.
    };

    const Concept* GetConcept() const
    {
        switch (m_type) {
        case union_type::pointer:
            return m_pointer.get();
        case union_type::buffer:
            return reinterpret_cast<const Concept*>(::std::addressof(m_buffer));
        case union_type::none:
            break;
        }
        return nullptr;
    }

public:

#if 1
    union {
        std::unique_ptr<const Concept> m_pointer;
        alignas(std::max_align_t) std::byte m_buffer[alignof(std::max_align_t) * 3u];
    };
    enum class union_type {none, pointer, buffer};
    union_type m_type = union_type::none;
#else
    using pointer = std::unique_ptr<const Concept>;
    //using buffer = std::aligned_storage_t<16u>;
    using buffer = std::byte[62u];
    std::variant<pointer, buffer> m_data;
#endif

    Foo() noexcept {}

    ~Foo() noexcept
    {
#if 0
        if (std::holds_alternative<buffer>(m_data)) {
            reinterpret_cast<const Concept*>(&std::get<buffer>(m_data))->~Concept();
        }
#endif
        switch (m_type) {
        case union_type::pointer:
            m_pointer.~unique_ptr<const Concept>();
            break;
        case union_type::buffer:
            reinterpret_cast<const Concept*>(m_buffer)->~Concept();
            break;
        case union_type::none:
            break;
        }
    }

    template <class T>
    Foo(T&& arg)
    {
#if 0
        static_assert(alignof(Model<T>) <= alignof(decltype(m_buffer)), "bad alignment");
        if (sizeof(Model<T>) > sizeof(m_buffer)) {
            new (&m_pointer) std::unique_ptr<const Concept>{std::make_unique<Model<T>>(std::forward<T>(arg))};
            m_type = union_type::pointer;
        }
        else {
            new (&m_buffer) Model<T>(std::move(arg));
            m_type = union_type::buffer;
        }
#else
#if 0
        if (sizeof(Model<T>) > sizeof(buffer)) {
            m_data = std::unique_ptr<const Concept>{std::make_unique<Model<T>>(std::forward<T>(arg))};
        }
        else {
            buffer b;
            m_data = b;
            new (&std::get<buffer>(m_data)) Model<T>(std::move(arg));
        }
#else
        if (sizeof(Model<T>) > sizeof(m_buffer)) {
            ::new ((void*)(::std::addressof(m_pointer))) std::unique_ptr<const Concept>{std::make_unique<Model<T>>(std::forward<T>(arg))};
            m_type = union_type::pointer;
        }
        else {
            ::new ((void*)(::std::addressof(m_buffer))) Model<T>(std::move(arg));
            m_type = union_type::buffer;
        }
#endif
#endif
    }

    Foo(const Foo& other)
    {
        switch (other.m_type) {
        case union_type::buffer:
            reinterpret_cast<const Concept*>(::std::addressof(other.m_buffer))->NewTo_(::std::addressof(m_buffer));
            m_type = union_type::buffer;
            break;
        case union_type::pointer:
            ::new ((void*)(::std::addressof(m_pointer))) std::unique_ptr<const Concept>{other.m_pointer->Clone_()};
            m_type = union_type::pointer;
            break;
        case union_type::none:
            break;
        }
    }

    bool has_value() const noexcept
    {
        //return m_type != union_type::none;
        //return !std::holds_alternative<pointer>(m_data) || std::get<pointer>(m_data);
        return m_type == union_type::buffer || (m_type == union_type::pointer && m_pointer);
    }

    friend Real GetFriction(const Foo& foo) noexcept
    {
#if 0
        //return foo.has_value()? reinterpret_cast<const Concept*>(&foo.m_buffer)->GetFriction_() : Real(0);
        return std::holds_alternative<buffer>(foo.m_data)?
        reinterpret_cast<const Concept*>(&std::get<buffer>(foo.m_data))->GetFriction_():
        std::get<pointer>(foo.m_data)? std::get<pointer>(foo.m_data)->GetFriction_(): Real(0);
#else
        const auto c = foo.GetConcept();
        return c? c->GetFriction_(): Real(0);
        //return (foo.m_type == union_type::buffer)?
        //reinterpret_cast<const Concept*>(foo.m_buffer)->GetFriction_():
        //(foo.m_type == union_type::pointer && foo.m_pointer)? foo.m_pointer->GetFriction_(): Real(0);
#endif
    }
};

// static_assert(sizeof(Foo) == 64u, "");

struct Foobar {
    Real f;
};

static Real GetFriction(const Foobar& value)
{
    return value.f;
}

TEST(Shape, TestFoo)
{
    EXPECT_EQ(sizeof(Foo), 64u);
    EXPECT_FALSE(Foo().has_value());
    EXPECT_TRUE(Foo(Foobar{3.0f}).has_value());
    EXPECT_EQ(GetFriction(Foo(Foobar{3.0f})), 3.0f);
}

class Concept {
public:
    virtual ~Concept() {};
};

struct storage {
    union {
        std::unique_ptr<const Concept> pointer;
        alignas(std::max_align_t) std::byte buffer[alignof(std::max_align_t) * 3u];
    };
    enum class union_type {pointer, buffer};
    union_type type = union_type::pointer;
    storage(): pointer{} {}
    ~storage() {
        if (type == union_type::pointer) {
            pointer.~unique_ptr<const Concept>();
        }
        else {
            reinterpret_cast<const Concept*>(buffer)->~Concept();
        }
    }
};

TEST(Shape, TestStorage)
{
    storage data;
    EXPECT_EQ(sizeof(data), 64u);
    EXPECT_EQ(sizeof(data.buffer), 48u);
}

#endif
