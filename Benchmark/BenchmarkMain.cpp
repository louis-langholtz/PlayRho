#include <benchmark/benchmark.h>

#include <PlayRho/Common/Math.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>

#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>

#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>

static void FloatAdd(benchmark::State& state)
{
    const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(a + b);
    }
}

static void FloatMult(benchmark::State& state)
{
    const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(a * b);
    }
}

static void FloatDiv(benchmark::State& state)
{
    const auto a = 5.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(a / b);
    }
}

static void FloatSqrt(benchmark::State& state)
{
    const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(std::sqrt(a));
    }
}

static void FloatSin(benchmark::State& state)
{
    const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(std::sin(a));
    }
}

static void FloatCos(benchmark::State& state)
{
    const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void FloatAtan2(benchmark::State& state)
{
    const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(std::atan2(a, b));
    }
}

static void FloatAlmostEqual1(benchmark::State& state)
{
    const auto x = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
    const auto y = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
    const auto ulp = static_cast<int>(rand() % 8);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize((playrho::Abs(x - y) < (std::numeric_limits<float>::epsilon() * playrho::Abs(x + y) * ulp)) || playrho::AlmostZero(x - y));
    }
}

static void FloatAlmostEqual2(benchmark::State& state)
{
    const auto x = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
    const auto y = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
    const auto ulp = static_cast<int>(rand() % 8);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Abs((int nX = *((int*)&x) < 0 ? 0x80000000 - nX : nX) - (int nY = *((int*)&y) < 0 ? 0x80000000 - nY : nY)) <= ulp);
    }
}


// ----

static void LengthSquaredViaDotProduct(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto v1 = playrho::Vec2(a, b);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Dot(v1, v1));
    }
}

static void GetLengthSquared(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetLengthSquared(playrho::Vec2(a, b)));
    }
}

static void GetLength(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetLength(playrho::Vec2(a, b)));
    }
}

static void hypot(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(std::hypot(a, b));
    }
}

static void UnitVectorFromVector(benchmark::State& state)
{
    const auto a = static_cast<float>(rand() - (RAND_MAX / 2)) * 22.93f / static_cast<float>(RAND_MAX);
    const auto b = static_cast<float>(rand() - (RAND_MAX / 2)) * 91.1092f / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetUnitVector(playrho::Vec2{a, b}));
    }
}

static void UnitVectorFromVectorAndBack(benchmark::State& state)
{
    const auto a = static_cast<float>(rand() - (RAND_MAX / 2)) * 22.93f / static_cast<float>(RAND_MAX);
    const auto b = static_cast<float>(rand() - (RAND_MAX / 2)) * 91.1092f / static_cast<float>(RAND_MAX);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetVec2(playrho::GetUnitVector(playrho::Vec2{a, b})));
    }
}

static void UnitVecFromAngle(benchmark::State& state)
{
    const auto a = static_cast<float>(rand()) * (23.1f * playrho::Radian * playrho::Pi / static_cast<float>(RAND_MAX));
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::UnitVec2::Get(a));
    }
}

// ----

static void FloatAddTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a + b);
    }
}

static void FloatMultTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a * b);
    }
}

static void FloatDivTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 5.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a / b);
    }
}

static void FloatSqrtTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::sqrt(a));
        benchmark::DoNotOptimize(std::sqrt(b));
    }
}

static void FloatSinTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::sin(a));
        benchmark::DoNotOptimize(std::sin(b));
    }
}

static void FloatCosTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::cos(a));
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void FloatAtan2TwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::atan2(a, b));
        benchmark::DoNotOptimize(std::atan2(b, a));
    }
}

static void FloatAlmostEqualThreeRand1(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto x = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
        const auto y = static_cast<float>(rand() - (RAND_MAX / 2)) / static_cast<float>(RAND_MAX / 2);
        const auto ulp = static_cast<int>(rand() % 8);
        benchmark::DoNotOptimize((playrho::Abs(x - y) <
                                  (std::numeric_limits<float>::epsilon() *
                                   playrho::Abs(x + y) * ulp)) || playrho::AlmostZero(x - y));
    }
}

static void DoubleAddTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a + b);
    }
}

static void DoubleMultTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a * b);
    }
}

static void DoubleDivTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 5.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a / b);
    }
}

static void DoubleSqrtTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::sqrt(a));
        benchmark::DoNotOptimize(std::sqrt(b));
    }
}

static void DoubleSinTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::sin(a));
        benchmark::DoNotOptimize(std::sin(b));
    }
}

static void DoubleCosTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::cos(a));
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void DoubleAtan2TwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::atan2(a, b));
        benchmark::DoNotOptimize(std::atan2(b, a));
    }
}

// ---

static void LengthSquaredViaDotProductTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto v1 = playrho::Vec2(a, b);
        benchmark::DoNotOptimize(playrho::Dot(v1, v1));
    }
}

static void GetLengthSquaredTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetLengthSquared(playrho::Vec2(a, b)));
    }
}

static void GetLengthTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetLength(playrho::Vec2(a, b)));
    }
}

static void hypotTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        benchmark::DoNotOptimize(std::hypot(a, b));
    }
}

static void UnitVectorFromVectorTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = static_cast<float>(rand() - (RAND_MAX / 2)) * 22.93f / static_cast<float>(RAND_MAX);
        const auto b = static_cast<float>(rand() - (RAND_MAX / 2)) * 91.1092f / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetUnitVector(playrho::Vec2{a, b}));
    }
}

static void UnitVectorFromVectorAndBackTwoRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = static_cast<float>(rand() - (RAND_MAX / 2)) * 22.93f / static_cast<float>(RAND_MAX);
        const auto b = static_cast<float>(rand() - (RAND_MAX / 2)) * 91.1092f / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetVec2(playrho::GetUnitVector(playrho::Vec2{a, b})));
    }
}

// ---

static void TwoRandValues(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
    }
}

static void ThreeRandValues(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Real(15.91) * static_cast<playrho::Real>(rand()) /
                                 static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) /
                                 static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(static_cast<float>(rand() - (RAND_MAX / 2)) /
                                 static_cast<float>(RAND_MAX / 2));
    }
}

static void FourRandValues(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(playrho::Real(81.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
        benchmark::DoNotOptimize(playrho::Real(+0.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX));
    }
}

// ----

static void DotProduct(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto v1 = playrho::Vec2(a, b);
    const auto c = playrho::Real(81.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto d = playrho::Real(+0.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto v2 = playrho::Vec2(c, d);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Dot(v1, v2));
    }
}

static void CrossProduct(benchmark::State& state)
{
    const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto v1 = playrho::Vec2(a, b);
    const auto c = playrho::Real(81.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto d = playrho::Real(+0.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
    const auto v2 = playrho::Vec2(c, d);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::Cross(v1, v2));
    }
}

// ----

static void DotProductFourRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto v1 = playrho::Vec2(a, b);
        const auto c = playrho::Real(81.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto d = playrho::Real(+0.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto v2 = playrho::Vec2(c, d);
        benchmark::DoNotOptimize(playrho::Dot(v1, v2));
    }
}

static void CrossProductFourRand(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = playrho::Real(15.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto b = playrho::Real(-4.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto v1 = playrho::Vec2(a, b);
        const auto c = playrho::Real(81.91) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto d = playrho::Real(+0.1092) * static_cast<playrho::Real>(rand()) / static_cast<playrho::Real>(RAND_MAX);
        const auto v2 = playrho::Vec2(c, d);
        benchmark::DoNotOptimize(Cross(v1, v2));
    }
}

static void MaxSepBetweenRelRectanglesNoStop(benchmark::State& state)
{
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::GetRight()}; // top
    
    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape0 = playrho::PolygonShape(dim, dim);
    const auto shape1 = playrho::PolygonShape(dim, dim);
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +
    
    const auto child0 = shape0.GetChild(0);
    const auto child1 = shape1.GetChild(0);
    
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm0, child1, xfm1));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child1, xfm1, child0, xfm0));
    }
}

static void MaxSepBetweenRelRectangles(benchmark::State& state)
{
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::GetRight()}; // top

    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape0 = playrho::PolygonShape(dim, dim);
    const auto shape1 = playrho::PolygonShape(dim, dim);
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +

    const auto child0 = shape0.GetChild(0);
    const auto child1 = shape1.GetChild(0);
    const auto totalRadius = child0.GetVertexRadius() + child1.GetVertexRadius();

    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm0, child1, xfm1, totalRadius));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child1, xfm1, child0, xfm0, totalRadius));
    }
}

static void MaxSepBetweenRelRectangles2NoStop(benchmark::State& state)
{
    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape = playrho::PolygonShape(dim, dim);
    
    const auto xfm0 = playrho::Transformation{
        playrho::Vec2{0, -1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // bottom
    const auto xfm1 = playrho::Transformation{
        playrho::Vec2{0, +1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // top
    
    const auto child0 = shape.GetChild(0);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm0, child0, xfm1));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm1, child0, xfm0));
    }
}

static void MaxSepBetweenRelRectangles2(benchmark::State& state)
{
    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape = playrho::PolygonShape(dim, dim);
    
    const auto xfm0 = playrho::Transformation{
        playrho::Vec2{0, -1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // bottom
    const auto xfm1 = playrho::Transformation{
        playrho::Vec2{0, +1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // top
    
    const auto child0 = shape.GetChild(0);
    const auto totalRadius = child0.GetVertexRadius() * playrho::Real(2);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm0, child0, xfm1, totalRadius));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, xfm1, child0, xfm0, totalRadius));
    }
}

static void MaxSepBetweenRel4x4(benchmark::State& state)
{
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::GetRight()}; // top
    
    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape0 = playrho::PolygonShape(dim, dim);
    const auto shape1 = playrho::PolygonShape(dim, dim);
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +
    
    const auto child0 = shape0.GetChild(0);
    const auto child1 = shape1.GetChild(0);

    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation4x4(child0, xfm0, child1, xfm1));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation4x4(child1, xfm1, child0, xfm0));
    }
}

static void MaxSepBetweenRel2_4x4(benchmark::State& state)
{
    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape = playrho::PolygonShape(dim, dim);
    
    const auto xfm0 = playrho::Transformation{
        playrho::Vec2{0, -1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // bottom
    const auto xfm1 = playrho::Transformation{
        playrho::Vec2{0, +1} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // top
    
    const auto child0 = shape.GetChild(0);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation4x4(child0, xfm0, child0, xfm1));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation4x4(child0, xfm1, child0, xfm0));
    }
}

static void MaxSepBetweenAbsRectangles(benchmark::State& state)
{
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::GetRight()}; // top

    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape0 = playrho::PolygonShape{dim, dim}.Transform(xfm0);
    const auto shape1 = playrho::PolygonShape{dim, dim}.Transform(xfm1);
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +
    
    const auto child0 = shape0.GetChild(0);
    const auto child1 = shape1.GetChild(0);
    const auto totalRadius = child0.GetVertexRadius() + child1.GetVertexRadius();
    
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, child1, totalRadius));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child1, child0, totalRadius));
    }
}

static void ManifoldForTwoSquares1(benchmark::State& state)
{
    const auto dim = playrho::Real(2) * playrho::Meter;
    
    // creates a square
    const auto shape = playrho::PolygonShape(dim, dim);
    
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec2::GetRight()}; // top
    
    // Rotate square A and put it below square B.
    // In ASCII art terms:
    //
    //   +---4---+
    //   |   |   |
    //   | B 3   |
    //   |   |   |
    //   |   2   |
    //   |   |   |
    //   |   1   |
    //   |  /+\  |
    //   2-1-*-1-2
    //    /  1  \
    //   / A |   \
    //  +    2    +
    //   \   |   /
    //    \  3  /
    //     \ | /
    //      \4/
    //       +

    while (state.KeepRunning())
    {
        // CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1);
        benchmark::DoNotOptimize(playrho::CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1));
    }
}

static void ManifoldForTwoSquares2(benchmark::State& state)
{
    // Shape A: square
    const auto shape0 = playrho::PolygonShape(playrho::Real{2} * playrho::Meter, playrho::Real{2} * playrho::Meter);
    
    // Shape B: wide rectangle
    const auto shape1 = playrho::PolygonShape(playrho::Real{3} * playrho::Meter, playrho::Real{1.5f} * playrho::Meter);
    
    const auto xfm0 = playrho::Transformation{
        playrho::Vec2{-2, 0} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // left
    const auto xfm1 = playrho::Transformation{
        playrho::Vec2{+2, 0} * (playrho::Real(1) * playrho::Meter),
        playrho::UnitVec2::GetRight()
    }; // right
    
    // Put square left, wide rectangle right.
    // In ASCII art terms:
    //
    //   +-------2
    //   |     +-+---------+
    //   |   A | 1   B     |
    //   |     | |         |
    //   4-3-2-1-*-1-2-3-4-5
    //   |     | |         |
    //   |     | 1         |
    //   |     +-+---------+
    //   +-------2
    //
    while (state.KeepRunning())
    {
        //CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1);
        benchmark::DoNotOptimize(CollideShapes(shape0.GetChild(0), xfm0, shape1.GetChild(0), xfm1));
    }
}

static void ConstructAndAssignVC(benchmark::State& state)
{
    const auto friction = playrho::Real(0.5);
    const auto restitution = playrho::Real(1);
    const auto tangentSpeed = playrho::LinearVelocity{playrho::Real(1.5) * playrho::MeterPerSecond};
    const auto invMass = playrho::Real(1) / playrho::Kilogram;
    const auto invRotI = playrho::Real(1) / ((playrho::SquareMeter * playrho::Kilogram) / playrho::SquareRadian);
    const auto normal = playrho::UnitVec2::GetRight();
    const auto location = playrho::Length2D{playrho::Real(0) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto impulse = playrho::ContactImpulses{playrho::Momentum{0}, playrho::Momentum{0}};
    const auto separation = playrho::Length{playrho::Real(-0.001) * playrho::Meter};
    const auto ps0 = playrho::WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = playrho::WorldManifold{normal, ps0};
    
    const auto locA = playrho::Length2D{playrho::Real(+1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posA = playrho::Position{locA, playrho::Angle(0)};
    const auto velA = playrho::Velocity{
        playrho::LinearVelocity2D{playrho::Real(-0.5) * playrho::MeterPerSecond, playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}
    };
    
    const auto locB = playrho::Length2D{playrho::Real(-1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posB = playrho::Position{locB, playrho::Angle(0)};
    const auto velB = playrho::Velocity{
        playrho::LinearVelocity2D{playrho::Real(+0.5) * playrho::MeterPerSecond, playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}
    };
    
    auto bcA = playrho::BodyConstraint{invMass, invRotI, locA, posA, velA};
    auto bcB = playrho::BodyConstraint{invMass, invRotI, locB, posB, velB};
    
    auto vc = playrho::VelocityConstraint{};
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(vc = playrho::VelocityConstraint{friction, restitution, tangentSpeed, worldManifold, bcA, bcB});
    }
}

static void malloc_free_random_size(benchmark::State& state)
{
    auto sizes = std::array<size_t, 100>();
    for (auto& size: sizes)
    {
        size = (static_cast<std::size_t>(std::rand()) % std::size_t{1ul << 18ul}) + std::size_t{1ul};
    }

    auto i = std::size_t{0};
    auto p = static_cast<void*>(nullptr);
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(p = std::malloc(sizes[i]));
        std::free(p);
        i = (i < (sizes.max_size() - 1ul))? i + 1: 0;
    }
}

static void random_malloc_free_100(benchmark::State& state)
{
    auto sizes = std::array<size_t, 100>();
    for (auto& size: sizes)
    {
        size = (static_cast<std::size_t>(std::rand()) % std::size_t{1ul << 18ul}) + std::size_t{1ul};
    }
    auto pointers = std::array<void*, 100>();
    while (state.KeepRunning())
    {
        auto ptr = std::begin(pointers);
        for (auto size: sizes)
        {
            benchmark::DoNotOptimize(*ptr = std::malloc(size));
            ++ptr;
        }
        for (auto& p: pointers)
        {
            std::free(p);
        }
    }
}

static void SolveVC(benchmark::State& state)
{
    const auto friction = playrho::Real(0.5);
    const auto restitution = playrho::Real(1);
    const auto tangentSpeed = playrho::LinearVelocity{playrho::Real(1.5) * playrho::MeterPerSecond};
    const auto invMass = playrho::Real(1) / playrho::Kilogram;
    const auto invRotI = playrho::Real(1) / ((playrho::SquareMeter * playrho::Kilogram) / playrho::SquareRadian);
    const auto normal = playrho::UnitVec2::GetRight();
    const auto location = playrho::Length2D{playrho::Real(0) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto impulse = playrho::ContactImpulses{playrho::Momentum{0}, playrho::Momentum{0}};
    const auto separation = playrho::Length{playrho::Real(-0.001) * playrho::Meter};
    const auto ps0 = playrho::WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = playrho::WorldManifold{normal, ps0};
    
    const auto locA = playrho::Length2D{playrho::Real(+1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posA = playrho::Position{locA, playrho::Angle(0)};
    const auto velA = playrho::Velocity{
        playrho::LinearVelocity2D{playrho::Real(-0.5) * playrho::MeterPerSecond, playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}
    };

    const auto locB = playrho::Length2D{playrho::Real(-1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posB = playrho::Position{locB, playrho::Angle(0)};
    const auto velB = playrho::Velocity{
        playrho::LinearVelocity2D{playrho::Real(+0.5) * playrho::MeterPerSecond, playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}
    };

    auto bcA = playrho::BodyConstraint{invMass, invRotI, locA, posA, velA};
    auto bcB = playrho::BodyConstraint{invMass, invRotI, locB, posB, velB};

    auto vc = playrho::VelocityConstraint{friction, restitution, tangentSpeed, worldManifold, bcA, bcB};
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GaussSeidel::SolveVelocityConstraint(vc));
        benchmark::ClobberMemory();
    }
}

static void DropTiles(int count)
{
    const auto linearSlop = playrho::Real(0.001f) * playrho::Meter;
    const auto vertexRadius = playrho::Length{linearSlop * playrho::Real(2)};
    const auto conf = playrho::PolygonShape::Conf{}.UseVertexRadius(vertexRadius);
    const auto m_world = std::make_unique<playrho::World>(playrho::WorldDef{}.UseMinVertexRadius(vertexRadius));
    
    {
        const auto a = playrho::Real{0.5f};
        const auto ground = m_world->CreateBody(playrho::BodyDef{}.UseLocation(playrho::Length2D{0, -a * playrho::Meter}));
        
        const auto N = 200;
        const auto M = 10;
        playrho::Length2D position;
        GetY(position) = playrho::Real(0.0f) * playrho::Meter;
        for (auto j = 0; j < M; ++j)
        {
            GetX(position) = -N * a * playrho::Meter;
            for (auto i = 0; i < N; ++i)
            {
                auto shape = playrho::PolygonShape{conf};
                SetAsBox(shape, a * playrho::Meter, a * playrho::Meter, position, playrho::Angle{0});
                ground->CreateFixture(std::make_shared<playrho::PolygonShape>(shape));
                GetX(position) += 2.0f * a * playrho::Meter;
            }
            GetY(position) -= 2.0f * a * playrho::Meter;
        }
    }
    
    {
        const auto a = playrho::Real{0.5f};
        const auto shape = std::make_shared<playrho::PolygonShape>(a * playrho::Meter, a * playrho::Meter, conf);
        shape->SetDensity(playrho::Real{5} * playrho::KilogramPerSquareMeter);
        
        playrho::Length2D x(playrho::Real(-7.0f) * playrho::Meter, playrho::Real(0.75f) * playrho::Meter);
        playrho::Length2D y;
        const auto deltaX = playrho::Length2D(playrho::Real(0.5625f) * playrho::Meter, playrho::Real(1.25f) * playrho::Meter);
        const auto deltaY = playrho::Length2D(playrho::Real(1.125f) * playrho::Meter, playrho::Real(0.0f) * playrho::Meter);
        
        for (auto i = 0; i < count; ++i)
        {
            y = x;
            
            for (auto j = i; j < count; ++j)
            {
                const auto body = m_world->CreateBody(playrho::BodyDef{}.UseType(playrho::BodyType::Dynamic).UseLocation(y));
                body->CreateFixture(shape);
                y += deltaY;
            }
            
            x += deltaX;
        }
    }
    
    auto step = playrho::StepConf{};
    step.SetTime(playrho::Time{playrho::Second / playrho::Real{60}});
    step.linearSlop = linearSlop;
    step.regMinSeparation = -linearSlop * playrho::Real(3);
    step.toiMinSeparation = -linearSlop * playrho::Real(1.5f);
    step.targetDepth = linearSlop * playrho::Real(3);
    step.tolerance = linearSlop / playrho::Real(4);
    step.maxLinearCorrection = linearSlop * playrho::Real(40);
    step.aabbExtension = linearSlop * playrho::Real(20);
    step.maxTranslation = playrho::Length{playrho::Meter * playrho::Real(4)};
    step.velocityThreshold = (playrho::Real{8} / playrho::Real{10}) * playrho::MeterPerSecond;
    step.maxSubSteps = std::uint8_t{48};

    while (GetAwakeCount(*m_world) > 0)
    {
        m_world->Step(step);
    }
}

static void TilesComesToRest12(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        DropTiles(12);
    }
}

static void TilesComesToRest20(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        DropTiles(20);
    }
}

static void TilesComesToRest36(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        DropTiles(36);
    }
}

#if 0
#define ADD_BM(n, f) \
    BENCHMARK_PRIVATE_DECLARE(f) = \
        benchmark::RegisterBenchmark(n, f);
#endif

BENCHMARK(FloatAdd);
BENCHMARK(FloatMult);
BENCHMARK(FloatDiv);

BENCHMARK(FloatSqrt);
BENCHMARK(FloatSin);
BENCHMARK(FloatCos);
BENCHMARK(FloatAtan2);

BENCHMARK(FloatAlmostEqual1);
BENCHMARK(FloatAlmostEqual2);

BENCHMARK(DotProduct);
BENCHMARK(CrossProduct);
BENCHMARK(LengthSquaredViaDotProduct);
BENCHMARK(GetLengthSquared);
BENCHMARK(GetLength);
BENCHMARK(hypot);
BENCHMARK(UnitVectorFromVector);
BENCHMARK(UnitVectorFromVectorAndBack);
BENCHMARK(UnitVecFromAngle);

BENCHMARK(TwoRandValues);

BENCHMARK(FloatAddTwoRand);
BENCHMARK(FloatMultTwoRand);
BENCHMARK(FloatDivTwoRand);
BENCHMARK(FloatSqrtTwoRand);
BENCHMARK(FloatSinTwoRand);
BENCHMARK(FloatCosTwoRand);
BENCHMARK(FloatAtan2TwoRand);

BENCHMARK(ThreeRandValues);
BENCHMARK(FloatAlmostEqualThreeRand1);

BENCHMARK(DoubleAddTwoRand);
BENCHMARK(DoubleMultTwoRand);
BENCHMARK(DoubleDivTwoRand);
BENCHMARK(DoubleSqrtTwoRand);
BENCHMARK(DoubleSinTwoRand);
BENCHMARK(DoubleCosTwoRand);
BENCHMARK(DoubleAtan2TwoRand);

BENCHMARK(LengthSquaredViaDotProductTwoRand);
BENCHMARK(GetLengthSquaredTwoRand);
BENCHMARK(GetLengthTwoRand);
BENCHMARK(hypotTwoRand);
BENCHMARK(UnitVectorFromVectorTwoRand);
BENCHMARK(UnitVectorFromVectorAndBackTwoRand);

BENCHMARK(FourRandValues);

BENCHMARK(DotProductFourRand);
BENCHMARK(CrossProductFourRand);

BENCHMARK(ConstructAndAssignVC);
BENCHMARK(SolveVC);

BENCHMARK(MaxSepBetweenAbsRectangles);
BENCHMARK(MaxSepBetweenRel4x4);
BENCHMARK(MaxSepBetweenRel2_4x4);
BENCHMARK(MaxSepBetweenRelRectanglesNoStop);
BENCHMARK(MaxSepBetweenRelRectangles2NoStop);
BENCHMARK(MaxSepBetweenRelRectangles);
BENCHMARK(MaxSepBetweenRelRectangles2);

BENCHMARK(ManifoldForTwoSquares1);
BENCHMARK(ManifoldForTwoSquares2);

BENCHMARK(malloc_free_random_size);
BENCHMARK(random_malloc_free_100);

BENCHMARK(TilesComesToRest12);
BENCHMARK(TilesComesToRest20);
BENCHMARK(TilesComesToRest36);

BENCHMARK_MAIN()
