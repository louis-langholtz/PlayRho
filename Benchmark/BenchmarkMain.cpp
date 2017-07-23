#include <benchmark/benchmark.h>

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>

#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>

using namespace playrho;

static void DoubleAddition(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = 91.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(a + b);
    }
}

static void DoubleMultiplication(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = 91.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(a * b);
    }
}

static void DoubleDivision(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 5.93 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = -4.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(a / b);
    }
}

static void DoubleSqrt(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = 91.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(std::sqrt(a));
        benchmark::DoNotOptimize(std::sqrt(b));
    }
}

static void DoubleSin(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = -4.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(std::sin(a));
        benchmark::DoNotOptimize(std::sin(b));
    }
}

static void DoubleCos(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        const auto b = -4.1092 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        benchmark::DoNotOptimize(std::cos(a));
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void ManifoldCalcForTwoSquares(benchmark::State& state)
{
    const auto dim = Real(2) * Meter;
    
    // creates a square
    const auto shape = PolygonShape(dim, dim);
    
    const auto rot0 = Angle{Real{45.0f} * Degree};
    const auto xfm0 = Transformation{Vec2{0, -2} * (Real(1) * Meter), UnitVec2::Get(rot0)}; // bottom
    const auto xfm1 = Transformation{Vec2{0, +2} * (Real(1) * Meter), UnitVec2::GetRight()}; // top
    
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
        benchmark::DoNotOptimize(CollideShapes(shape.GetChild(0), xfm0, shape.GetChild(0), xfm1));
    }
}

static void Mani(benchmark::State& state)
{
    // Shape A: square
    const auto shape0 = PolygonShape(Real{2} * Meter, Real{2} * Meter);
    
    // Shape B: wide rectangle
    const auto shape1 = PolygonShape(Real{3} * Meter, Real{1.5f} * Meter);
    
    const auto xfm0 = Transformation{
        Vec2{-2, 0} * (Real(1) * Meter),
        UnitVec2::GetRight()
    }; // left
    const auto xfm1 = Transformation{
        Vec2{+2, 0} * (Real(1) * Meter),
        UnitVec2::GetRight()
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

BENCHMARK(DoubleAddition);
BENCHMARK(DoubleMultiplication);
BENCHMARK(DoubleDivision);
BENCHMARK(DoubleSqrt);
BENCHMARK(DoubleSin);
BENCHMARK(DoubleCos);
BENCHMARK(ManifoldCalcForTwoSquares);
BENCHMARK(Mani);

BENCHMARK_MAIN()
