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

using namespace playrho;

static void FloatAddition(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a + b);
    }
}

static void FloatMultiplication(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a * b);
    }
}

static void FloatDivision(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 5.93f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(a / b);
    }
}

static void FloatSqrt(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = 91.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::sqrt(a));
        benchmark::DoNotOptimize(std::sqrt(b));
    }
}

static void FloatSin(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::sin(a));
        benchmark::DoNotOptimize(std::sin(b));
    }
}

static void FloatCos(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        const auto b = -4.1092f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        benchmark::DoNotOptimize(std::cos(a));
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void DoubleAddition(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a + b);
    }
}

static void DoubleMultiplication(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 22.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a * b);
    }
}

static void DoubleDivision(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 5.93 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(a / b);
    }
}

static void DoubleSqrt(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = 91.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::sqrt(a));
        benchmark::DoNotOptimize(std::sqrt(b));
    }
}

static void DoubleSin(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::sin(a));
        benchmark::DoNotOptimize(std::sin(b));
    }
}

static void DoubleCos(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = 15.91 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        const auto b = -4.1092 * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        benchmark::DoNotOptimize(std::cos(a));
        benchmark::DoNotOptimize(std::cos(b));
    }
}

static void LengthSquaredViaDotProduct(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto v1 = Vec2(a, b);
        benchmark::DoNotOptimize(Dot(v1, v1));
    }
}

static void BM_GetLengthSquared(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetLengthSquared(Vec2(a, b)));
    }
}

static void BM_GetLength(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetLength(Vec2(a, b)));
    }
}

static void UnitVectorFromVec2(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        benchmark::DoNotOptimize(playrho::GetUnitVector(Vec2(a, b)));
    }
}

static void TwoRandValues(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
        benchmark::DoNotOptimize(Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
    }
}

static void FourRandValues(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
        benchmark::DoNotOptimize(Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
        benchmark::DoNotOptimize(Real(81.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
        benchmark::DoNotOptimize(Real(+0.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX));
    }
}

static void DotProduct(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto v1 = Vec2(a, b);
        const auto c = Real(81.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto d = Real(+0.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto v2 = Vec2(c, d);
        benchmark::DoNotOptimize(Dot(v1, v2));
    }
}

static void CrossProduct(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        const auto a = Real(15.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto b = Real(-4.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto v1 = Vec2(a, b);
        const auto c = Real(81.91) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto d = Real(+0.1092) * static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX);
        const auto v2 = Vec2(c, d);
        benchmark::DoNotOptimize(Cross(v1, v2));
    }
}

static void GetMaxSeparation(benchmark::State& state)
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

    const auto child = shape.GetChild(0);
    const auto totalRadius = child.GetVertexRadius() + child.GetVertexRadius();

    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child, xfm0, child, xfm1, totalRadius));
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child, xfm1, child, xfm0, totalRadius));
    }
}

static void ManifoldForTwoSquares1(benchmark::State& state)
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

static void ManifoldForTwoSquares2(benchmark::State& state)
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

static void ConstructAndAssignVC(benchmark::State& state)
{
    const auto friction = Real(0.5);
    const auto restitution = Real(1);
    const auto tangentSpeed = LinearVelocity{Real(1.5) * MeterPerSecond};
    const auto invMass = Real(1) / Kilogram;
    const auto invRotI = Real(1) / ((SquareMeter * Kilogram) / SquareRadian);
    const auto normal = UnitVec2::GetRight();
    const auto location = Length2D{Real(0) * Meter, Real(0) * Meter};
    const auto impulse = ContactImpulses{Momentum{0}, Momentum{0}};
    const auto separation = Length{Real(-0.001) * Meter};
    const auto ps0 = WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = WorldManifold{normal, ps0};
    
    const auto locA = Length2D{Real(+1) * Meter, Real(0) * Meter};
    const auto posA = Position{locA, Angle(0)};
    const auto velA = Velocity{
        LinearVelocity2D{Real(-0.5) * MeterPerSecond, Real(0) * MeterPerSecond},
        AngularVelocity{Real(0) * RadianPerSecond}
    };
    
    const auto locB = Length2D{Real(-1) * Meter, Real(0) * Meter};
    const auto posB = Position{locB, Angle(0)};
    const auto velB = Velocity{
        LinearVelocity2D{Real(+0.5) * MeterPerSecond, Real(0) * MeterPerSecond},
        AngularVelocity{Real(0) * RadianPerSecond}
    };
    
    auto bcA = BodyConstraint{invMass, invRotI, locA, posA, velA};
    auto bcB = BodyConstraint{invMass, invRotI, locB, posB, velB};
    
    auto vc = VelocityConstraint{};
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(vc = VelocityConstraint{friction, restitution, tangentSpeed, worldManifold, bcA, bcB});
    }
}

static void SolveVC(benchmark::State& state)
{
    const auto friction = Real(0.5);
    const auto restitution = Real(1);
    const auto tangentSpeed = LinearVelocity{Real(1.5) * MeterPerSecond};
    const auto invMass = Real(1) / Kilogram;
    const auto invRotI = Real(1) / ((SquareMeter * Kilogram) / SquareRadian);
    const auto normal = UnitVec2::GetRight();
    const auto location = Length2D{Real(0) * Meter, Real(0) * Meter};
    const auto impulse = ContactImpulses{Momentum{0}, Momentum{0}};
    const auto separation = Length{Real(-0.001) * Meter};
    const auto ps0 = WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = WorldManifold{normal, ps0};
    
    const auto locA = Length2D{Real(+1) * Meter, Real(0) * Meter};
    const auto posA = Position{locA, Angle(0)};
    const auto velA = Velocity{
        LinearVelocity2D{Real(-0.5) * MeterPerSecond, Real(0) * MeterPerSecond},
        AngularVelocity{Real(0) * RadianPerSecond}
    };

    const auto locB = Length2D{Real(-1) * Meter, Real(0) * Meter};
    const auto posB = Position{locB, Angle(0)};
    const auto velB = Velocity{
        LinearVelocity2D{Real(+0.5) * MeterPerSecond, Real(0) * MeterPerSecond},
        AngularVelocity{Real(0) * RadianPerSecond}
    };

    auto bcA = BodyConstraint{invMass, invRotI, locA, posA, velA};
    auto bcB = BodyConstraint{invMass, invRotI, locB, posB, velB};

    auto vc = VelocityConstraint{friction, restitution, tangentSpeed, worldManifold, bcA, bcB};
    while (state.KeepRunning())
    {
        benchmark::DoNotOptimize(GaussSeidel::SolveVelocityConstraint(vc));
        benchmark::ClobberMemory();
    }
}

static void DropTiles(int count)
{
    const auto m_world = std::make_unique<World>();
    
    {
        const auto a = Real{0.5f};
        const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Length2D{0, -a * Meter}));
        
        const auto N = 200;
        const auto M = 10;
        Length2D position;
        position.y = Real(0.0f) * Meter;
        for (auto j = 0; j < M; ++j)
        {
            position.x = -N * a * Meter;
            for (auto i = 0; i < N; ++i)
            {
                PolygonShape shape;
                SetAsBox(shape, a * Meter, a * Meter, position, Angle{0});
                ground->CreateFixture(std::make_shared<PolygonShape>(shape));
                position.x += 2.0f * a * Meter;
            }
            position.y -= 2.0f * a * Meter;
        }
    }
    
    {
        const auto a = Real{0.5f};
        const auto shape = std::make_shared<PolygonShape>(a * Meter, a * Meter);
        shape->SetDensity(Real{5} * KilogramPerSquareMeter);
        
        Length2D x(Real(-7.0f) * Meter, Real(0.75f) * Meter);
        Length2D y;
        const auto deltaX = Length2D(Real(0.5625f) * Meter, Real(1.25f) * Meter);
        const auto deltaY = Length2D(Real(1.125f) * Meter, Real(0.0f) * Meter);
        
        for (auto i = 0; i < count; ++i)
        {
            y = x;
            
            for (auto j = i; j < count; ++j)
            {
                const auto body = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(y));
                body->CreateFixture(shape);
                y += deltaY;
            }
            
            x += deltaX;
        }
    }
    
    auto step = StepConf{};
    step.SetTime(Time{Second / Real{60}});
    while (GetAwakeCount(*m_world) > 0)
    {
        m_world->Step(step);
    }
}

static void TilesComesToRest(benchmark::State& state)
{
    while (state.KeepRunning())
    {
        DropTiles(12);
    }
}

BENCHMARK(TwoRandValues);

BENCHMARK(FloatAddition);
BENCHMARK(FloatMultiplication);
BENCHMARK(FloatDivision);
BENCHMARK(FloatSqrt);
BENCHMARK(FloatSin);
BENCHMARK(FloatCos);

BENCHMARK(DoubleAddition);
BENCHMARK(DoubleMultiplication);
BENCHMARK(DoubleDivision);
BENCHMARK(DoubleSqrt);
BENCHMARK(DoubleSin);
BENCHMARK(DoubleCos);

BENCHMARK(LengthSquaredViaDotProduct);
BENCHMARK(BM_GetLengthSquared);
BENCHMARK(BM_GetLength);
BENCHMARK(UnitVectorFromVec2);

BENCHMARK(FourRandValues);

BENCHMARK(DotProduct);
BENCHMARK(CrossProduct);

BENCHMARK(ConstructAndAssignVC);
BENCHMARK(SolveVC);

BENCHMARK(GetMaxSeparation);
BENCHMARK(ManifoldForTwoSquares1);
BENCHMARK(ManifoldForTwoSquares2);

BENCHMARK(TilesComesToRest);

BENCHMARK_MAIN()
