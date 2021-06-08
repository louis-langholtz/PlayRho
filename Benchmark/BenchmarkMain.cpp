/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

/*
 * Notes:
 *   - Any code in here must, for now, be C++14 standard compliant code.
 *   - I'm of the opinion that short of looking at the resultant assembly, it's hard
 *     to say/know/tell what the compiler actually optimzies or doesn't.
 *   - `benchmark::DoNotOptimize` seemingly only prevents enclosed expressions from
 *     being totally optimized away and has no effect on avoiding sub-expression
 *     optimization especially in regards to output from constexpr functions.
 *   - I've opted to use "random" data to help prevent optimizations that might make
 *     timing meaningless. This incurs the time overhead of generating the random value
 *     which then must be factored in to analysis of the output results.
 */

#include <benchmark/benchmark.h>
#include <tuple>
#include <utility>
#include <cstdlib>
#include <vector>
#include <future>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <ctime>
#include <map>
#include <optional>
#include <type_traits>
#include <functional>
#include <chrono>
#include <thread>

// #define BENCHMARK_GCDISPATCH
#ifdef BENCHMARK_GCDISPATCH
#include <dispatch/dispatch.h>
#endif // BENCHMARK_GCDISPATCH

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Intervals.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp> // for GetAwakeCount
#include <PlayRho/Dynamics/WorldShape.hpp> // for CreateShape
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/Contacts/ContactSolver.hpp>
#include <PlayRho/Dynamics/Contacts/VelocityConstraint.hpp>
#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>

#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/DynamicTree.hpp>
#include <PlayRho/Collision/Manifold.hpp>
#include <PlayRho/Collision/WorldManifold.hpp>
#include <PlayRho/Collision/ShapeSeparation.hpp>
#if 0
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#endif
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>
#include <PlayRho/Collision/Shapes/Rectangle.hpp>

#define BENCHMARK_BOX2D
#ifdef BENCHMARK_BOX2D
#include <box2d/box2d.h>
#endif // BENCHMARK_BOX2D

template <typename T>
static T Rand(T lo, T hi)
{
    const auto u = std::rand() / static_cast<float>(RAND_MAX); // # between 0 and 1
    return static_cast<T>((hi - lo) * u) + lo;
}

static playrho::d2::UnitVec GetRandUnitVec2(playrho::Angle lo, playrho::Angle hi)
{
    const auto a = Rand(static_cast<float>(playrho::Real{lo / playrho::Radian}),
                        static_cast<float>(playrho::Real{hi / playrho::Radian}));
    return playrho::d2::UnitVec::Get(playrho::Real{a} * playrho::Radian);
}

static playrho::d2::Transformation GetRandTransformation(playrho::d2::Position pos0,
                                                         playrho::d2::Position pos1)
{
    const auto x0 = static_cast<double>(playrho::Real{GetX(pos0.linear) / playrho::Meter});
    const auto y0 = static_cast<double>(playrho::Real{GetY(pos0.linear) / playrho::Meter});
    const auto a0 = static_cast<double>(playrho::Real{pos0.angular / playrho::Radian});

    const auto x1 = static_cast<double>(playrho::Real{GetX(pos1.linear) / playrho::Meter});
    const auto y1 = static_cast<double>(playrho::Real{GetY(pos1.linear) / playrho::Meter});
    const auto a1 = static_cast<double>(playrho::Real{pos1.angular / playrho::Radian});

    const auto x = static_cast<playrho::Real>(Rand(x0, x1)) * playrho::Meter;
    const auto y = static_cast<playrho::Real>(Rand(y0, y1)) * playrho::Meter;
    const auto a = static_cast<playrho::Real>(Rand(a0, a1)) * playrho::Radian;

    return playrho::d2::Transformation{playrho::Length2{x, y}, playrho::d2::UnitVec::Get(a)};
}

template <typename T>
static std::vector<T> Rands(unsigned count, T lo, T hi)
{
    auto rands = std::vector<T>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(Rand(lo, hi));
    }
    return rands;
}

template <typename T>
static std::pair<T, T> RandPair(T lo, T hi)
{
    const auto first = Rand(lo, hi);
    const auto second = Rand(lo, hi);
    return std::make_pair(first, second);
}

static std::pair<playrho::d2::UnitVec, playrho::d2::UnitVec> GetRandUnitVec2Pair(playrho::Angle lo,
                                                                                 playrho::Angle hi)
{
    return std::make_pair(GetRandUnitVec2(lo, hi), GetRandUnitVec2(lo, hi));
}

static std::pair<playrho::d2::Transformation, playrho::d2::Transformation>
GetRandTransformationPair(playrho::d2::Position pos0, playrho::d2::Position pos1)
{
    return std::make_pair(GetRandTransformation(pos0, pos1), GetRandTransformation(pos0, pos1));
}

template <typename T>
static std::tuple<T, T, T> RandTriplet(T lo, T hi)
{
    const auto first = Rand(lo, hi);
    const auto second = Rand(lo, hi);
    const auto third = Rand(lo, hi);
    return std::make_tuple(first, second, third);
}

template <typename T>
static std::tuple<T, T, T, T> RandQuad(T lo, T hi)
{
    const auto first = Rand(lo, hi);
    const auto second = Rand(lo, hi);
    const auto third = Rand(lo, hi);
    const auto fourth = Rand(lo, hi);
    return std::make_tuple(first, second, third, fourth);
}

template <typename T>
static std::tuple<T, T, T, T, T, T, T, T> RandOctet(T lo, T hi)
{
    const auto first = Rand(lo, hi);
    const auto second = Rand(lo, hi);
    const auto third = Rand(lo, hi);
    const auto fourth = Rand(lo, hi);
    const auto fifth = Rand(lo, hi);
    const auto sixth = Rand(lo, hi);
    const auto seventh = Rand(lo, hi);
    const auto eighth = Rand(lo, hi);
    return std::make_tuple(first, second, third, fourth, fifth, sixth, seventh, eighth);
}

template <typename T>
static std::vector<std::pair<T, T>> RandPairs(unsigned count, T lo, T hi)
{
    auto rands = std::vector<std::pair<T, T>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(RandPair(lo, hi));
    }
    return rands;
}

static std::vector<std::pair<playrho::d2::UnitVec, playrho::d2::UnitVec>>
GetRandUnitVec2Pairs(unsigned count, playrho::Angle lo, playrho::Angle hi)
{
    auto rands = std::vector<std::pair<playrho::d2::UnitVec, playrho::d2::UnitVec>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(GetRandUnitVec2Pair(lo, hi));
    }
    return rands;
}

static std::vector<std::pair<playrho::d2::Transformation, playrho::d2::Transformation>>
GetRandTransformationPairs(unsigned count, playrho::d2::Position pos0, playrho::d2::Position pos1)
{
    auto rands = std::vector<std::pair<playrho::d2::Transformation, playrho::d2::Transformation>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(GetRandTransformationPair(pos0, pos1));
    }
    return rands;
}

template <typename T>
static std::vector<std::tuple<T, T, T>> RandTriplets(unsigned count, T lo, T hi)
{
    auto rands = std::vector<std::tuple<T, T, T>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(RandTriplet(lo, hi));
    }
    return rands;
}

template <typename T>
static std::vector<std::tuple<T, T, T, T>> RandQuads(unsigned count, T lo, T hi)
{
    auto rands = std::vector<std::tuple<T, T, T, T>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(RandQuad(lo, hi));
    }
    return rands;
}

template <typename T>
static std::vector<std::tuple<T, T, T, T, T, T, T, T>> RandOctets(unsigned count, T lo, T hi)
{
    auto rands = std::vector<std::tuple<float, float, float, float, float, float, float, float>>{};
    rands.reserve(count);
    for (auto i = decltype(count){0}; i < count; ++i) {
        rands.push_back(RandOctet(lo, hi));
    }
    return rands;
}

// ----

static void FloatAdd(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first + val.second);
        }
    }
}

static void FloatMul(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first * val.second);
        }
    }
}

static void FloatDiv(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first / val.second);
        }
    }
}

static void FloatSqrt(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), 0.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::sqrt(val));
        }
    }
}

static void FloatSin(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0f, +4.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::sin(val));
        }
    }
}

static void FloatCos(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0f, +4.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::cos(val));
        }
    }
}

static void FloatSinCos(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0f, +4.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            // If runtime of sin + cos = sin or cos then seemingly hardware
            //   calculates both at same time and compiler knows that.
            benchmark::DoNotOptimize(std::make_pair(std::sin(val), std::cos(val)));
        }
    }
}

static void FloatAtan2(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::atan2(val.first, val.second));
        }
    }
}

static void FloatHypot(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::hypot(val.first, val.second));
        }
    }
}

static void FloatMulAdd(benchmark::State& state)
{
    const auto vals = RandTriplets(static_cast<unsigned>(state.range()), -1000.0f, 1000.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize((std::get<0>(val) * std::get<1>(val)) + std::get<2>(val));
        }
    }
}

static void FloatFma(benchmark::State& state)
{
    const auto vals = RandTriplets(static_cast<unsigned>(state.range()), -1000.0f, 1000.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                std::fma(std::get<0>(val), std::get<1>(val), std::get<2>(val)));
        }
    }
}

// ----

static void DoubleAdd(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first + val.second);
        }
    }
}

static void DoubleMul(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first * val.second);
        }
    }
}

static void DoubleDiv(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first / val.second);
        }
    }
}

static void DoubleSqrt(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), 0.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::sqrt(val));
        }
    }
}

static void DoubleSin(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0, +4.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::sin(val));
        }
    }
}

static void DoubleCos(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0, +4.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::cos(val));
        }
    }
}

static void DoubleSinCos(benchmark::State& state)
{
    const auto vals = Rands(static_cast<unsigned>(state.range()), -4.0, +4.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            // If runtime of sin + cos = sin or cos then seemingly hardware
            //   calculates both at same time and compiler knows that.
            benchmark::DoNotOptimize(std::make_pair(std::sin(val), std::cos(val)));
        }
    }
}

static void DoubleAtan2(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::atan2(val.first, val.second));
        }
    }
}

static void DoubleHypot(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::hypot(val.first, val.second));
        }
    }
}

static void DoubleMulAdd(benchmark::State& state)
{
    const auto vals = RandTriplets(static_cast<unsigned>(state.range()), -1000.0, 1000.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize((std::get<0>(val) * std::get<1>(val)) + std::get<2>(val));
        }
    }
}

static void DoubleFma(benchmark::State& state)
{
    const auto vals = RandTriplets(static_cast<unsigned>(state.range()), -1000.0, 1000.0);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                std::fma(std::get<0>(val), std::get<1>(val), std::get<2>(val)));
        }
    }
}

// ---

static void noopFunc() {}

static void AsyncFutureDeferred(benchmark::State& state)
{
    std::future<void> f;
    for (auto _ : state) {
        benchmark::DoNotOptimize(f = std::async(std::launch::deferred, noopFunc));
        f.get();
    }
}

static void AsyncFutureAsync(benchmark::State& state)
{
    std::future<void> f;
    for (auto _ : state) {
        benchmark::DoNotOptimize(f = std::async(std::launch::async, noopFunc));
        f.get();
    }
}

#ifdef BENCHMARK_GCDISPATCH
template <typename F, typename... Args>
auto gcd_async(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type>
{
    using result_type = typename std::result_of<F(Args...)>::type;
    using packaged_type = std::packaged_task<result_type()>;

    auto p = new packaged_type(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    auto result = p->get_future();
    dispatch_async_f(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), p,
                     [](void* f_) {
                         packaged_type* f = static_cast<packaged_type*>(f_);
                         (*f)();
                         delete f;
                     });

    return result;
}

static void AsyncFutureDispatch(benchmark::State& state)
{
    std::future<void> f;
    for (auto _ : state) {
        benchmark::DoNotOptimize(f = gcd_async(noopFunc));
        f.get();
    }
}
#endif // BENCHMARK_GCDISPATCH

static void ThreadCreateAndDestroy(benchmark::State& state)
{
    for (auto _ : state) {
        std::thread t{noopFunc};
        t.join();
    }
}

/// @brief Concurrent queue.
/// @details A pretty conventional concurrent queue implementation using a regular queue
///   structure made thread safe with a mutex and a condition variable.
/// @note Behavior is undefined if destroyed in one thread while being accessed in another.
/// @see
/// https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
template <typename T>
class ConcurrentQueue
{
public:
    using value_type = T;

    void Enqueue(const T& e)
    {
        {
            std::lock_guard<decltype(m_mutex)> lock{m_mutex};
            m_queue.push(e); // inserts e at back
        }
        m_cond.notify_one();
    }

    void Enqueue(T&& e)
    {
        {
            std::lock_guard<decltype(m_mutex)> lock{m_mutex};
            m_queue.push(std::move(e)); // inserts e at back
        }
        m_cond.notify_one();
    }

    T Dequeue()
    {
        std::unique_lock<decltype(m_mutex)> lock{m_mutex};
        while (m_queue.empty()) {
            m_cond.wait(lock);
        }
        // Use std::move to be explicit about the intention of moving the data.
        T e = std::move(m_queue.front());
        m_queue.pop(); // removes element from front
        return e;
    }

    void Dequeue(T& e)
    {
        std::unique_lock<decltype(m_mutex)> lock{m_mutex};
        while (m_queue.empty()) {
            m_cond.wait(lock);
        }
        // Use std::move to be explicit about the intention of moving the data.
        e = std::move(m_queue.front());
        m_queue.pop(); // removes element from front
    }

private:
    std::queue<T> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cond;
};

template <typename T>
class Concurrent
{
public:
    using value_type = T;

    void Enqueue(const T& e)
    {
        {
            std::lock_guard<decltype(m_mutex)> lock{m_mutex};
            m_element = e;
        }
        m_cond.notify_one();
    }

    void Enqueue(T&& e)
    {
        {
            std::lock_guard<decltype(m_mutex)> lock{m_mutex};
            m_element = std::move(e);
        }
        m_cond.notify_one();
    }

    T Dequeue()
    {
        std::unique_lock<decltype(m_mutex)> lock{m_mutex};
        while (!m_element.has_value()) {
            m_cond.wait(lock);
        }
        // Use std::move to be explicit about the intention of moving the data.
        T e = std::move(m_element.value());
        m_element.reset();
        return e;
    }

    void Dequeue(T& e)
    {
        std::unique_lock<decltype(m_mutex)> lock{m_mutex};
        while (!m_element.has_value()) {
            m_cond.wait(lock);
        }
        // Use std::move to be explicit about the intention of moving the data.
        e = std::move(m_element.value());
        m_element.reset();
    }

private:
    std::optional<T> m_element;
    std::mutex m_mutex;
    std::condition_variable m_cond;
};

/// @brief Atomic element.
/// @note Supports single reader, single writer.
/// @see https://en.cppreference.com/w/cpp/atomic/atomic_flag
template <typename T>
class AtomicSingleElementQueue
{
public:
    using value_type = T;

    AtomicSingleElementQueue()
    {
        // Reader starts locked out...
        while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
            ; // spin
    }

    void Enqueue(const T& e)
    {
        m_element = e;
        m_lock.clear(std::memory_order_release); // release lock
    }

    void Enqueue(T&& e)
    {
        m_element = std::move(e);
        m_lock.clear(std::memory_order_release); // release lock
    }

    T Dequeue()
    {
        while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
            ; // spin
        // Use std::move to be explicit about the intention of moving the data.
        T e = std::move(m_element);
        m_element = T{};
        return e;
    }

    void Dequeue(T& e)
    {
        while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
            ; // spin
        // Use std::move to be explicit about the intention of moving the data.
        e = std::move(m_element);
        m_element = T{};
    }

private:
    T m_element = T{};
    std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
};

/// @brief Atomic queue.
/// @note Supports multiple readers, single writer.
/// @see https://en.cppreference.com/w/cpp/atomic/atomic_flag
template <typename T>
class AtomicQueue
{
public:
    using value_type = T;

    void Enqueue(const T& e)
    {
        while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
        {
            // spin
        }
        m_queue.push(e);
        m_lock.clear(std::memory_order_release); // release lock
    }

    void Enqueue(T&& e)
    {
        while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
        {
            // spin
        }
        m_queue.push(std::move(e));
        m_lock.clear(std::memory_order_release); // release lock
    }

    T Dequeue()
    {
        for (;;) {
            while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
            {
                // spin
            }
            if (!m_queue.empty()) {
                break;
            }
            m_lock.clear(std::memory_order_release); // release lock
        }

        // Use std::move to be explicit about the intention of moving the data.
        T e = std::move(m_queue.front());
        m_queue.pop(); // removes element from front
        m_lock.clear(std::memory_order_release); // release lock
        return e;
    }

    void Dequeue(T& e)
    {
        for (;;) {
            while (m_lock.test_and_set(std::memory_order_acquire)) // acquire lock
            {
                // spin
            }
            if (!m_queue.empty()) {
                break;
            }
            m_lock.clear(std::memory_order_release); // release lock
        }

        // Use std::move to be explicit about the intention of moving the data.
        e = std::move(m_queue.front());
        m_queue.pop(); // removes element from front
        m_lock.clear(std::memory_order_release); // release lock
    }

private:
    std::queue<T> m_queue;
    std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
};

static void MultiThreadQD(benchmark::State& state)
{
    ConcurrentQueue<int> queue01;
    ConcurrentQueue<int> queue10;

    // 13538 ns with stddev of 1479 ns.
    // 11541 ns on another run with 6081 ns of CPU time.

    std::thread t{[&]() {
        for (;;) {
            const auto v = queue01.Dequeue();
            if (v == 0)
                break;
            queue10.Enqueue(v);
        }
    }};

    const auto in = 12;
    auto out = 0;
    for (auto _ : state) {
        queue01.Enqueue(in);
        queue10.Dequeue(out);
    }
    queue01.Enqueue(0);
    t.join();
}

static void MultiThreadQDE(benchmark::State& state)
{
    Concurrent<int> queue01;
    Concurrent<int> queue10;

    // 13538 ns with stddev of 1479 ns.
    // 11541 ns on another run with 6081 ns of CPU time.

    std::thread t{[&]() {
        for (;;) {
            const auto v = queue01.Dequeue();
            if (v == 0)
                break;
            queue10.Enqueue(v);
        }
    }};

    const auto in = 12;
    auto out = 0;
    for (auto _ : state) {
        queue01.Enqueue(in);
        queue10.Dequeue(out);
    }
    queue01.Enqueue(0);
    t.join();
}

static void MultiThreadQDA(benchmark::State& state)
{
    AtomicSingleElementQueue<int> queue01;
    AtomicSingleElementQueue<int> queue10;

    std::thread t{[&]() {
        for (;;) {
            const auto v = queue01.Dequeue();
            if (v == 0)
                break;
            queue10.Enqueue(v);
        }
    }};

    const auto in = 12;
    auto out = 0;
    for (auto _ : state) {
        queue01.Enqueue(in);
        queue10.Dequeue(out);
    }
    queue01.Enqueue(0);
    t.join();
}

static void MultiThreadQDAQ(benchmark::State& state)
{
    AtomicQueue<int> queue01;
    AtomicQueue<int> queue10;

    const auto in = 12;
    auto out = 0;

    std::thread t{[&]() {
        for (;;) {
            const auto v = queue01.Dequeue();
            if (v == 0)
                break;
            queue10.Enqueue(v);
        }
    }};
    for (auto _ : state) {
        queue01.Enqueue(in);
        queue10.Dequeue(out);
    }
    queue01.Enqueue(0);
    t.join();
}

// ---

static void AlmostEqual1(benchmark::State& state)
{
    const auto ulp = static_cast<int>(rand() % 8);
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto x = val.first;
            const auto y = val.second;
            benchmark::DoNotOptimize((playrho::abs(x - y) < (std::numeric_limits<float>::epsilon() *
                                                             playrho::abs(x + y) * ulp)) ||
                                     playrho::AlmostZero(x - y));
        }
    }
}

static void AlmostEqual2(benchmark::State& state)
{
    const auto ulp = static_cast<unsigned>(rand() % 8);
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto x = val.first;
            const auto y = val.second;
            // Accesses the floats as unsigned 32 bit ints and strips off the signbits.
            const auto nX = (*reinterpret_cast<const std::uint32_t*>(&x)) & 0x7FFFFFF;
            const auto nY = (*reinterpret_cast<const std::uint32_t*>(&y)) & 0x7FFFFFF;
            benchmark::DoNotOptimize(((nX >= nY) ? nX - nY : nY - nX) <= ulp);
        }
    }
}

static void AlmostEqual3(benchmark::State& state)
{
    const auto ulp = static_cast<int>(rand() % 8);
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto x = val.first;
            const auto y = val.second;
            // Accesses the floats as unsigned 32 bit ints and strips off the signbits.
            const auto nX = static_cast<std::int32_t>(
                (*reinterpret_cast<const std::uint32_t*>(&x)) & std::uint32_t{0x7FFFFFF});
            const auto nY = static_cast<std::int32_t>(
                (*reinterpret_cast<const std::uint32_t*>(&y)) & std::uint32_t{0x7FFFFFF});
            benchmark::DoNotOptimize(std::abs(nX - nY) <= ulp);
        }
    }
}

static void ModuloViaTrunc(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(playrho::ModuloViaTrunc(val.first, val.second));
        }
    }
}

static void ModuloViaFmod(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(playrho::ModuloViaFmod(val.first, val.second));
        }
    }
}

static void LengthSquaredViaDotProduct(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto vec = playrho::Vec2(val.first, val.second);
            benchmark::DoNotOptimize(playrho::Dot(vec, vec));
        }
    }
}

static void GetMagnitudeSquared(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                playrho::GetMagnitudeSquared(playrho::Vec2(val.first, val.second)));
        }
    }
}

static void GetMagnitude(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(playrho::GetMagnitude(playrho::Vec2(val.first, val.second)));
        }
    }
}

static void UnitVectorFromVector(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                playrho::d2::GetUnitVector(playrho::Vec2{val.first, val.second}));
        }
    }
}

static playrho::Vec2 GetUnitVec1(playrho::Vec2 vec, playrho::Vec2 fallback)
{
    const auto magSquared = playrho::Square(vec[0]) + playrho::Square(vec[1]);
    if (playrho::isnormal(magSquared)) {
        const auto mag = playrho::sqrt(magSquared);
        return playrho::Vec2{vec[0] / mag, vec[1] / mag};
    }
    return fallback;
}

static playrho::Vec2 GetUnitVec2(playrho::Vec2 vec, playrho::Vec2 fallback)
{
    const auto magSquared = playrho::Square(vec[0]) + playrho::Square(vec[1]);
    if (playrho::isnormal(magSquared)) {
        const auto mag = playrho::sqrt(magSquared);
        return playrho::Vec2{vec[0] / mag, vec[1] / mag};
    }
    const auto mag = playrho::hypot(vec[0], vec[1]);
    if (playrho::isnormal(mag)) {
        return playrho::Vec2{vec[0] / mag, vec[1] / mag};
    }
    return fallback;
}

static void GetUnitVec1(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -10000.0f, 10000.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                GetUnitVec1(playrho::Vec2{val.first, val.second}, playrho::Vec2{0, 0}));
        }
    }
}

static void GetUnitVec2(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -10000.0f, 10000.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(
                GetUnitVec2(playrho::Vec2{val.first, val.second}, playrho::Vec2{0, 0}));
        }
    }
}

static void UnitVectorFromVectorAndBack(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(playrho::d2::GetVec2(
                playrho::d2::GetUnitVector(playrho::Vec2{val.first, val.second})));
        }
    }
}

static void UnitVecFromAngle(benchmark::State& state)
{
    // With angle modulo in the regular phase solver code it's unlikely to see angles
    // outside of the range -2 * Pi to +2 * Pi.
    const auto vals = Rands(static_cast<unsigned>(state.range()), -8.0f, +8.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            // If runtime of sin + cos = sin or cos then seemingly hardware
            //   calculates both at same time and compiler knows that.
            benchmark::DoNotOptimize(playrho::d2::UnitVec::Get(val * playrho::Radian));
        }
    }
}

static void DiffSignsViaSignbit(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -1.0f, 1.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(std::signbit(val.first) != std::signbit(val.second));
        }
    }
}

static void DiffSignsViaMul(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -1.0f, 1.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            benchmark::DoNotOptimize(val.first * val.second < 0.0f);
        }
    }
}

static void DotProduct(benchmark::State& state)
{
    const auto vals = RandQuads(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto v1 = playrho::Vec2{std::get<0>(val), std::get<1>(val)};
            const auto v2 = playrho::Vec2{std::get<2>(val), std::get<3>(val)};
            benchmark::DoNotOptimize(playrho::Dot(v1, v2));
        }
    }
}

static void CrossProduct(benchmark::State& state)
{
    const auto vals = RandQuads(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto v1 = playrho::Vec2{std::get<0>(val), std::get<1>(val)};
            const auto v2 = playrho::Vec2{std::get<2>(val), std::get<3>(val)};
            benchmark::DoNotOptimize(playrho::Cross(v1, v2));
        }
    }
}

static void IntervalIsIntersecting(benchmark::State& state)
{
    const auto vals = RandQuads(static_cast<unsigned>(state.range()), playrho::Real{-100.0f},
                                playrho::Real{100.0f});
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto c = std::get<2>(val);
            const auto d = std::get<3>(val);
            const auto i0 = playrho::Interval<playrho::Real>{a, b};
            const auto i1 = playrho::Interval<playrho::Real>{c, d};
            benchmark::DoNotOptimize(playrho::IsIntersecting(i0, i1));
        }
    }
}

static void LessFloat(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    // auto c = 0.0f;
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a < b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LessDouble(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    // auto c = 0.0f;
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a < b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LessEqualFloat(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    // auto c = 0.0f;
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LessEqualDouble(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserFloat(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    auto c = 0.0f;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a < b) ? a : b;
            static_assert(std::is_same<decltype(v), const float>::value, "not float");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserDouble(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    auto c = 0.0;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a < b) ? a : b;
            static_assert(std::is_same<decltype(v), const double>::value, "not double");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserEqualFloat(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    auto c = 0.0f;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b) ? a : b;
            static_assert(std::is_same<decltype(v), const float>::value, "not float");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserEqualDouble(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    auto c = 0.0;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b) ? a : b;
            static_assert(std::is_same<decltype(v), const double>::value, "not double");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserEqualLength(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    auto c = 0.0f * playrho::Meter;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b) ? a : b;
            static_assert(std::is_same<decltype(v), const playrho::Length>::value, "not Length");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void MinFloat(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    auto c = 0.0f;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = std::min(a, b);
            static_assert(std::is_same<decltype(v), const float>::value, "not float");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void MinDouble(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0, 100.0);
    auto c = 0.0;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = std::min(a, b);
            static_assert(std::is_same<decltype(v), const double>::value, "not double");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LessLength(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    // auto c = 0.0f * playrho::Meter;
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a < b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LessEqualLength(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    // auto c = 0.0f * playrho::Meter;
    auto c = false;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = (a <= b);
            static_assert(std::is_same<decltype(v), const bool>::value, "not bool");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LesserLength(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    auto c = 0.0f * playrho::Meter;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            static_assert(std::is_same<decltype(b), const playrho::Length>::value, "not Length");
            const auto v = (a < b) ? a : b;
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void MinLength(benchmark::State& state)
{
    const auto vals = RandPairs(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    auto c = 0.0f * playrho::Meter;
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto v = std::min(a, b);
            static_assert(std::is_same<decltype(v), const playrho::Length>::value, "not Length");
            benchmark::DoNotOptimize(c = v);
        }
    }
}

static void LengthIntervalIsIntersecting(benchmark::State& state)
{
    const auto vals = RandQuads(static_cast<unsigned>(state.range()), -100.0f * playrho::Meter,
                                100.0f * playrho::Meter);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto a = std::get<0>(val);
            const auto b = std::get<1>(val);
            const auto c = std::get<2>(val);
            const auto d = std::get<3>(val);
            const auto i0 = playrho::LengthInterval{a, b};
            const auto i1 = playrho::LengthInterval{c, d};
            benchmark::DoNotOptimize(playrho::IsIntersecting(i0, i1));
        }
    }
}

static void AabbTestOverlap(benchmark::State& state)
{
    const auto vals = RandOctets(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto p0 = playrho::Length2{std::get<0>(val) * playrho::Meter,
                                             std::get<1>(val) * playrho::Meter};
            const auto p1 = playrho::Length2{std::get<2>(val) * playrho::Meter,
                                             std::get<3>(val) * playrho::Meter};
            const auto p2 = playrho::Length2{std::get<4>(val) * playrho::Meter,
                                             std::get<5>(val) * playrho::Meter};
            const auto p3 = playrho::Length2{std::get<6>(val) * playrho::Meter,
                                             std::get<7>(val) * playrho::Meter};
            const auto aabb0 = playrho::d2::AABB{p0, p1};
            const auto aabb1 = playrho::d2::AABB{p2, p3};
            benchmark::DoNotOptimize(playrho::d2::TestOverlap(aabb0, aabb1));
        }
    }
}

static void AabbContains(benchmark::State& state)
{
    const auto vals = RandOctets(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto p0 = playrho::Length2{std::get<0>(val) * playrho::Meter,
                                             std::get<1>(val) * playrho::Meter};
            const auto p1 = playrho::Length2{std::get<2>(val) * playrho::Meter,
                                             std::get<3>(val) * playrho::Meter};
            const auto p2 = playrho::Length2{std::get<4>(val) * playrho::Meter,
                                             std::get<5>(val) * playrho::Meter};
            const auto p3 = playrho::Length2{std::get<6>(val) * playrho::Meter,
                                             std::get<7>(val) * playrho::Meter};
            const auto aabb0 = playrho::d2::AABB{p0, p1};
            const auto aabb1 = playrho::d2::AABB{p2, p3};
            benchmark::DoNotOptimize(playrho::d2::Contains(aabb0, aabb1));
        }
    }
}

static void AABB(benchmark::State& state)
{
    const auto vals = RandOctets(static_cast<unsigned>(state.range()), -100.0f, 100.0f);
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto p0 = playrho::Length2{std::get<0>(val) * playrho::Meter,
                                             std::get<1>(val) * playrho::Meter};
            const auto p1 = playrho::Length2{std::get<2>(val) * playrho::Meter,
                                             std::get<3>(val) * playrho::Meter};
            const auto p2 = playrho::Length2{std::get<4>(val) * playrho::Meter,
                                             std::get<5>(val) * playrho::Meter};
            const auto p3 = playrho::Length2{std::get<6>(val) * playrho::Meter,
                                             std::get<7>(val) * playrho::Meter};
            const auto aabb0 = playrho::d2::AABB{p0, p1};
            const auto aabb1 = playrho::d2::AABB{p2, p3};
            benchmark::DoNotOptimize(playrho::d2::TestOverlap(aabb0, aabb1));
            benchmark::DoNotOptimize(playrho::d2::Contains(aabb0, aabb1));
        }
    }
}

// ----

using TransformationPair = std::pair<playrho::d2::Transformation, playrho::d2::Transformation>;
using TransformationPairs = std::vector<TransformationPair>;

static TransformationPairs GetTransformationPairs(unsigned count)
{
    static std::map<unsigned, TransformationPairs> xfms;

    constexpr auto pos0 =
        playrho::d2::Position{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter),
                              playrho::Angle{playrho::Real{0.0f} * playrho::Degree}}; // bottom

    constexpr auto pos1 =
        playrho::d2::Position{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter),
                              playrho::Angle{playrho::Real{360.0f} * playrho::Degree}}; // top

    auto it = xfms.find(count);
    if (it == xfms.end()) {
        const auto result =
            xfms.insert(std::make_pair(count, GetRandTransformationPairs(count, pos0, pos1)));
        it = result.first;
    }

    return it->second;
}

static void MaxSepBetweenRelSquaresNoStop(benchmark::State& state)
{
    const auto shape0 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();
    const auto shape1 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();

    const auto child0 = GetChild(shape0, 0);
    const auto child1 = GetChild(shape1, 0);

    const auto vals = GetTransformationPairs(static_cast<unsigned>(state.range()));
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto xf0 = val.first;
            const auto xf1 = val.second;
            benchmark::DoNotOptimize(playrho::d2::GetMaxSeparation(child0, xf0, child1, xf1));
        }
    }
}

static void MaxSepBetweenRel4x4(benchmark::State& state)
{
    const auto shape0 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();
    const auto shape1 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();

    const auto child0 = GetChild(shape0, 0);
    const auto child1 = GetChild(shape1, 0);

    const auto vals = GetTransformationPairs(static_cast<unsigned>(state.range()));
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto xf0 = val.first;
            const auto xf1 = val.second;
            benchmark::DoNotOptimize(playrho::d2::GetMaxSeparation4x4(child0, xf0, child1, xf1));
        }
    }
}

static void MaxSepBetweenRelSquares(benchmark::State& state)
{
    const auto shape0 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();
    const auto shape1 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();

    const auto child0 = GetChild(shape0, 0);
    const auto child1 = GetChild(shape1, 0);
    const auto totalRadius = child0.GetVertexRadius() + child1.GetVertexRadius();

    const auto vals = GetTransformationPairs(static_cast<unsigned>(state.range()));
    for (auto _ : state) {
        for (const auto& val : vals) {
            const auto xf0 = val.first;
            const auto xf1 = val.second;
            benchmark::DoNotOptimize(
                playrho::d2::GetMaxSeparation(child0, xf0, child1, xf1, totalRadius));
        }
    }
}

#if 0
static void MaxSepBetweenAbsSquares(benchmark::State& state)
{
    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 = playrho::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec::Get(rot0)}; // bottom
    const auto xfm1 = playrho::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter), playrho::UnitVec::GetRight()}; // top

    const auto dim = playrho::Real(2) * playrho::Meter;
    const auto shape0 = playrho::PolygonShapeConf{dim, dim}.Transform(xfm0);
    const auto shape1 = playrho::PolygonShapeConf{dim, dim}.Transform(xfm1);
    
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
    
    const auto child0 = GetChild(shape0, 0);
    const auto child1 = GetChild(shape1, 0);
    const auto totalRadius = child0.GetVertexRadius() + child1.GetVertexRadius();
    
    for (auto _: state)
    {
        benchmark::DoNotOptimize(playrho::GetMaxSeparation(child0, child1, totalRadius));
    }
}
#endif

static void ManifoldForTwoSquares1(benchmark::State& state)
{
    // creates a square
    const auto shape = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();

    const auto rot0 = playrho::Angle{playrho::Real{45.0f} * playrho::Degree};
    const auto xfm0 =
        playrho::d2::Transformation{playrho::Vec2{0, -2} * (playrho::Real(1) * playrho::Meter),
                                    playrho::d2::UnitVec::Get(rot0)}; // bottom
    const auto xfm1 =
        playrho::d2::Transformation{playrho::Vec2{0, +2} * (playrho::Real(1) * playrho::Meter),
                                    playrho::d2::UnitVec::GetRight()}; // top

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

    for (auto _ : state) {
        benchmark::DoNotOptimize(
            playrho::d2::CollideShapes(GetChild(shape, 0), xfm0, GetChild(shape, 0), xfm1));
    }
}

static void ManifoldForTwoSquares2(benchmark::State& state)
{
    // Shape A: square
    const auto shape0 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 4, 4>();

    // Shape B: wide rectangle
    const auto shape1 = playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 6, 3>();

    const auto xfm0 =
        playrho::d2::Transformation{playrho::Vec2{-2, 0} * (playrho::Real(1) * playrho::Meter),
                                    playrho::d2::UnitVec::GetRight()}; // left
    const auto xfm1 =
        playrho::d2::Transformation{playrho::Vec2{+2, 0} * (playrho::Real(1) * playrho::Meter),
                                    playrho::d2::UnitVec::GetRight()}; // right

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
    for (auto _ : state) {
        benchmark::DoNotOptimize(
            CollideShapes(GetChild(shape0, 0), xfm0, GetChild(shape1, 0), xfm1));
    }
}

static void ConstructAndAssignVC(benchmark::State& state)
{
    const auto friction = playrho::Real(0.5);
    const auto restitution = playrho::Real(1);
    const auto tangentSpeed = playrho::LinearVelocity{playrho::Real(1.5) * playrho::MeterPerSecond};
    const auto invMass = playrho::Real(1) / playrho::Kilogram;
    const auto invRotI =
        playrho::Real(1) / ((playrho::SquareMeter * playrho::Kilogram) / playrho::SquareRadian);
    const auto normal = playrho::d2::UnitVec::GetRight();
    const auto location =
        playrho::Length2{playrho::Real(0) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto impulse = playrho::Momentum2{playrho::Momentum{0}, playrho::Momentum{0}};
    const auto separation = playrho::Length{playrho::Real(-0.001) * playrho::Meter};
    const auto ps0 = playrho::d2::WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = playrho::d2::WorldManifold{normal, ps0};

    const auto locA =
        playrho::Length2{playrho::Real(+1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posA = playrho::d2::Position{locA, playrho::Angle(0)};
    const auto velA = playrho::d2::Velocity{
        playrho::LinearVelocity2{playrho::Real(-0.5) * playrho::MeterPerSecond,
                                 playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}};

    const auto locB =
        playrho::Length2{playrho::Real(-1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posB = playrho::d2::Position{locB, playrho::Angle(0)};
    const auto velB = playrho::d2::Velocity{
        playrho::LinearVelocity2{playrho::Real(+0.5) * playrho::MeterPerSecond,
                                 playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}};

    const auto bodyConstraints = std::vector<playrho::d2::BodyConstraint>{
        playrho::d2::BodyConstraint{invMass, invRotI, locA, posA, velA},
        playrho::d2::BodyConstraint{invMass, invRotI, locB, posB, velB}};
    auto vc = playrho::d2::VelocityConstraint{};
    for (auto _ : state) {
        benchmark::DoNotOptimize(vc = playrho::d2::VelocityConstraint{
                                     friction, restitution, tangentSpeed, worldManifold,
                                     playrho::BodyID(0u), playrho::BodyID(1u), bodyConstraints});
    }
}

#if 0
static void malloc_free_random_size(benchmark::State& state)
{
    auto sizes = std::array<size_t, 100>();
    for (auto& size: sizes)
    {
        size = (static_cast<std::size_t>(std::rand()) % std::size_t{1ul << 18ul}) + std::size_t{1ul};
    }

    auto i = std::size_t{0};
    auto p = static_cast<void*>(nullptr);
    for (auto _: state)
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
    for (auto _: state)
    {
        auto ptr = begin(pointers);
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
#endif

static void SolveVC(benchmark::State& state)
{
    const auto friction = playrho::Real(0.5);
    const auto restitution = playrho::Real(1);
    const auto tangentSpeed = playrho::LinearVelocity{playrho::Real(1.5) * playrho::MeterPerSecond};
    const auto invMass = playrho::Real(1) / playrho::Kilogram;
    const auto invRotI =
        playrho::Real(1) / ((playrho::SquareMeter * playrho::Kilogram) / playrho::SquareRadian);
    const auto normal = playrho::d2::UnitVec::GetRight();
    const auto location =
        playrho::Length2{playrho::Real(0) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto impulse = playrho::Momentum2{playrho::Momentum{0}, playrho::Momentum{0}};
    const auto separation = playrho::Length{playrho::Real(-0.001) * playrho::Meter};
    const auto ps0 = playrho::d2::WorldManifold::PointData{location, impulse, separation};
    const auto worldManifold = playrho::d2::WorldManifold{normal, ps0};

    const auto locA =
        playrho::Length2{playrho::Real(+1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posA = playrho::d2::Position{locA, playrho::Angle(0)};
    const auto velA = playrho::d2::Velocity{
        playrho::LinearVelocity2{playrho::Real(-0.5) * playrho::MeterPerSecond,
                                 playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}};

    const auto locB =
        playrho::Length2{playrho::Real(-1) * playrho::Meter, playrho::Real(0) * playrho::Meter};
    const auto posB = playrho::d2::Position{locB, playrho::Angle(0)};
    const auto velB = playrho::d2::Velocity{
        playrho::LinearVelocity2{playrho::Real(+0.5) * playrho::MeterPerSecond,
                                 playrho::Real(0) * playrho::MeterPerSecond},
        playrho::AngularVelocity{playrho::Real(0) * playrho::RadianPerSecond}};

    auto bodyConstraints = std::vector<playrho::d2::BodyConstraint>{
        playrho::d2::BodyConstraint{invMass, invRotI, locA, posA, velA},
        playrho::d2::BodyConstraint{invMass, invRotI, locB, posB, velB}};
    auto vc =
        playrho::d2::VelocityConstraint{friction,       restitution,         tangentSpeed,
                                        worldManifold,  playrho::BodyID(0u), playrho::BodyID(1u),
                                        bodyConstraints};
    for (auto _ : state) {
        benchmark::DoNotOptimize(
            playrho::GaussSeidel::SolveVelocityConstraint(vc, bodyConstraints));
        benchmark::ClobberMemory();
    }
}

static void WorldStepPlayRho(benchmark::State& state)
{
    const auto stepConf = playrho::StepConf{};
    auto world =
        playrho::d2::World{playrho::d2::WorldConf{}.UseTreeCapacity(0).UseContactCapacity(0u)};
    for (auto _ : state) {
        world.Step(stepConf);
    }
}

#ifdef BENCHMARK_BOX2D
static void WorldStepBox2D(benchmark::State& state)
{
    const auto gravity = b2Vec2(0.0f, 0.0f);
    b2World world(gravity);
    for (auto _ : state) {
        world.Step(1.0f / 60, 8, 3);
    }
}
#endif

static void CreateBodyWithOneShapePlayRho(benchmark::State& state)
{
    const auto numBodies = state.range();
    const auto shape =
        playrho::d2::Shape(playrho::d2::Rectangle<playrho::d2::Geometry::Constant, 1, 1>{});
    for (auto _ : state) {
        state.PauseTiming();
        playrho::d2::World world{
            playrho::d2::WorldConf{/* zero G */}.UseTreeCapacity(0u).UseContactCapacity(0u)};
        const auto shapeId = world.CreateShape(shape);
        state.ResumeTiming();
        for (auto i = 0; i < numBodies; ++i) {
            CreateBody(world, playrho::d2::Body{}.Attach(shapeId), false);
        }
    }
}

#ifdef BENCHMARK_BOX2D
static void CreateBodyWithOneShapeBox2D(benchmark::State& state)
{
    const auto numBodies = state.range();
    const auto a = 0.5f;
    b2PolygonShape shape;
    shape.SetAsBox(a, a);
    b2BodyDef bd;
    for (auto _ : state) {
        state.PauseTiming();
        b2World world(b2Vec2(0.0f, 0.0f));
        state.ResumeTiming();
        for (auto i = 0; i < numBodies; ++i) {
            world.CreateBody(&bd)->CreateFixture(&shape, 0.0f);
        }
    }
}
#endif

static void WorldStepWithStatsStaticPlayRho(benchmark::State& state)
{
    const auto stepConf = playrho::StepConf{};
    auto stepStats = playrho::StepStats{};
    const auto numBodies = state.range();
    auto world = playrho::d2::World{playrho::d2::WorldConf{/* zero G */}};
    for (auto i = decltype(numBodies){0}; i < numBodies; ++i) {
        CreateBody(world, playrho::d2::BodyConf{}.UseType(playrho::BodyType::Static));
    }
    for (auto _ : state) {
        benchmark::DoNotOptimize(stepStats = world.Step(stepConf));
    }
}

#ifdef BENCHMARK_BOX2D
static void WorldStepWithStatsStaticBox2D(benchmark::State& state)
{
    const auto numBodies = state.range();
    const auto gravity = b2Vec2(0.0f, 0.0f);
    b2World world(gravity);
    b2BodyDef bd;
    bd.type = b2_staticBody;
    bd.position = b2Vec2(0.0f, 0.0f);
    for (auto i = decltype(numBodies){0}; i < numBodies; ++i) {
        world.CreateBody(&bd);
    }
    for (auto _ : state) {
        world.Step(1.0f / 60, 8, 3);
    }
}
#endif

static void DropDisksPlayRho(benchmark::State& state)
{
    const auto numDisks = state.range();
    auto world = playrho::d2::World{};
    const auto diskRadius = 0.5f * playrho::Meter;
    const auto diskConf = playrho::d2::DiskShapeConf{}.UseRadius(diskRadius);
    const auto shapeId = world.CreateShape(playrho::d2::Shape{diskConf});
    for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
        const auto x = i * diskRadius * 4;
        const auto location = playrho::Length2{x, 0 * playrho::Meter};
        auto body = playrho::d2::Body{playrho::d2::BodyConf{}
                                          .UseType(playrho::BodyType::Dynamic)
                                          .UseLocation(location)
                                          .UseLinearAcceleration(playrho::d2::EarthlyGravity)};
        body.Attach(shapeId);
        CreateBody(world, body);
    }
    const auto stepConf = playrho::StepConf{};
    for (auto _ : state) {
        world.Step(stepConf);
    }
}

static void DropDisksSixtyStepsPlayRho(benchmark::State& state)
{
    const auto numDisks = state.range();
    const auto stepConf = playrho::StepConf{};
    const auto diskRadius = 0.5f * playrho::Meter;
    const auto diskConf = playrho::d2::DiskShapeConf{}.UseRadius(diskRadius);
    for (auto _ : state) {
        state.PauseTiming();
        auto world = playrho::d2::World{};
        const auto shapeId = world.CreateShape(playrho::d2::Shape{diskConf});
        for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
            const auto x = i * diskRadius * 4;
            const auto location = playrho::Length2{x, 0 * playrho::Meter};
            auto body = playrho::d2::Body{playrho::d2::BodyConf{}
                                              .UseType(playrho::BodyType::Dynamic)
                                              .UseLocation(location)
                                              .UseLinearAcceleration(playrho::d2::EarthlyGravity)};
            body.Attach(shapeId);
            CreateBody(world, body);
        }
        state.ResumeTiming();
        for (auto i = 0; i < 60; ++i) {
            world.Step(stepConf);
        }
    }
}

#ifdef BENCHMARK_BOX2D
static void DropDisksBox2D(benchmark::State& state)
{
    const auto numDisks = state.range();
    const auto gravity = b2Vec2(0.0f, static_cast<float>(playrho::EarthlyLinearAcceleration /
                                                         playrho::MeterPerSquareSecond));
    b2World world(gravity);
    const auto diskRadius = 0.5f;
    b2CircleShape circle;
    circle.m_p.SetZero();
    circle.m_radius = diskRadius;
    for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
        const auto x = i * diskRadius * 4;
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position = b2Vec2(x, 0.0f);
        const auto body = world.CreateBody(&bd);
        body->CreateFixture(&circle, 0.01f);
    }
    for (auto _ : state) {
        world.Step(1.0f / 60, 8, 3);
    }
}

static void DropDisksSixtyStepsBox2D(benchmark::State& state)
{
    const auto numDisks = state.range();
    const auto gravity = b2Vec2(0.0f, static_cast<float>(playrho::EarthlyLinearAcceleration /
                                                         playrho::MeterPerSquareSecond));
    const auto diskRadius = 0.5f;
    b2CircleShape circle;
    circle.m_p.SetZero();
    circle.m_radius = diskRadius;
    for (auto _ : state) {
        state.PauseTiming();
        b2World world(gravity);
        for (auto i = decltype(numDisks){0}; i < numDisks; ++i) {
            const auto x = i * diskRadius * 4;
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = b2Vec2(x, 0.0f);
            const auto body = world.CreateBody(&bd);
            body->CreateFixture(&circle, 0.01f);
        }
        state.ResumeTiming();
        for (auto i = 0; i < 60; ++i) {
            world.Step(1.0f / 60, 8, 3);
        }
    }
}
#endif

static void AddPairStressTestPlayRho(benchmark::State& state, int count)
{
    using namespace playrho;
    using namespace playrho::d2;
    const auto diskConf =
        DiskShapeConf{}.UseRadius(playrho::Meter / 10).UseDensity(0.01f * KilogramPerSquareMeter);
    const auto diskShape = Shape{diskConf};

    const auto rectShape =
        Shape{Rectangle<Geometry::Constant, 3, 3,
                        shape_part::DensityIs<shape_part::StaticAreaDensity<1>>>()};

    const auto rectBodyConf =
        playrho::d2::BodyConf{}
            .UseType(playrho::BodyType::Dynamic)
            .UseBullet(true)
            .UseLocation(playrho::Length2{-40.0f * playrho::Meter, 5.0f * playrho::Meter})
            .UseLinearVelocity(
                playrho::LinearVelocity2{playrho::Vec2(150.0f, 0.0f) * playrho::MeterPerSecond});

    constexpr auto linearSlop = 0.005f * playrho::Meter;
    constexpr auto angularSlop = (2.0f / 180.0f * playrho::Pi) * playrho::Radian;

    const auto worldConf = playrho::d2::WorldConf{/* zero G */}.UseTreeCapacity(8192);
    auto stepConf = playrho::StepConf{};
    stepConf.deltaTime = playrho::Second / 60;
    stepConf.linearSlop = linearSlop;
    stepConf.angularSlop = angularSlop;
    stepConf.regMinSeparation = -linearSlop * 3;
    stepConf.toiMinSeparation = -linearSlop * 1.5f;
    stepConf.targetDepth = linearSlop * 3;
    stepConf.tolerance = linearSlop / 4;
    stepConf.maxLinearCorrection = 0.2f * playrho::Meter;
    stepConf.maxAngularCorrection = (8.0f / 180.0f * playrho::Pi) * playrho::Radian;
    stepConf.aabbExtension = 0.1f * playrho::Meter;
    stepConf.maxTranslation = 2.0f * playrho::Meter;
    stepConf.velocityThreshold = 1.0f * playrho::MeterPerSecond;
    stepConf.maxSubSteps = std::uint8_t{8};

    const auto minX = -6.0f;
    const auto maxX = 0.0f;
    const auto minY = 4.0f;
    const auto maxY = 6.0f;
    const auto bd = playrho::d2::BodyConf{}.UseType(playrho::BodyType::Dynamic);
    for (auto _ : state) {
        state.PauseTiming();
        auto world = playrho::d2::World{worldConf};
        const auto diskShapeId = world.CreateShape(diskShape);
        const auto rectShapeId = world.CreateShape(rectShape);
        {
            for (auto i = 0; i < count; ++i) {
                const auto location =
                    playrho::Vec2(Rand(minX, maxX), Rand(minY, maxY)) * playrho::Meter;
                // Uses parenthesis here to work around Visual C++'s const propagating of the copy.
                auto body = playrho::d2::Body{playrho::d2::BodyConf(bd).UseLocation(location)};
                body.Attach(diskShapeId);
                CreateBody(world, body);
            }
        }
        auto rectBody = playrho::d2::Body{rectBodyConf};
        rectBody.Attach(rectShapeId);
        CreateBody(world, rectBody);
        for (auto i = 0; i < state.range(); ++i) {
            world.Step(stepConf);
        }
        state.ResumeTiming();

        world.Step(stepConf);
    }
}

static void AddPairStressTestPlayRho400(benchmark::State& state)
{
    AddPairStressTestPlayRho(state, 400);
}

#ifdef BENCHMARK_BOX2D
static void AddPairStressTestBox2D(benchmark::State& state, int count)
{
    const auto gravity = b2Vec2(0.0f, 0.0f);
    const auto minX = -6.0f;
    const auto maxX = 0.0f;
    const auto minY = 4.0f;
    const auto maxY = 6.0f;
    b2CircleShape circle;
    circle.m_p.SetZero();
    circle.m_radius = 0.1f;

    b2PolygonShape shape;
    shape.SetAsBox(1.5f, 1.5f);
    for (auto _ : state) {
        state.PauseTiming();
        b2World world(gravity);
        {
            for (auto i = 0; i < count; ++i) {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position = b2Vec2(Rand(minX, maxX), Rand(minY, maxY));
                const auto body = world.CreateBody(&bd);
                body->CreateFixture(&circle, 0.01f);
            }
        }
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(-40.0f, 5.0f);
        bd.bullet = true;
        const auto body = world.CreateBody(&bd);
        body->CreateFixture(&shape, 1.0f);
        body->SetLinearVelocity(b2Vec2(150.0f, 0.0f));
        for (auto i = 0; i < state.range(); ++i) {
            world.Step(1.0f / 60, 8, 3);
        }
        state.ResumeTiming();

        world.Step(1.0f / 60, 8, 3);
    }
}

static void AddPairStressTestBox2D400(benchmark::State& state)
{
    AddPairStressTestBox2D(state, 400);
}
#endif // BENCHMARK_BOX2D

constexpr auto DeltaXx = 0.5625f;
constexpr auto DeltaXy = 1.25f;
constexpr auto DeltaYx = 1.125f;
constexpr auto DeltaYy = 0.0f;
constexpr auto TilesWidth = 200;
constexpr auto TilesHeight = 10;
constexpr auto TilesGravityX = 0.0f;
constexpr auto TilesGravityY = -10.0f;

static void DropTilesPlayRho(int count, bool groundIsComboShape = true)
{
    constexpr auto linearSlop = 0.005f * playrho::Meter;
    constexpr auto angularSlop = (2.0f / 180.0f * playrho::Pi) * playrho::Radian;
    constexpr auto vertexRadius = linearSlop * 2;
    constexpr auto gravity =
        playrho::LinearAcceleration2{TilesGravityX * playrho::MeterPerSquareSecond,
                                     TilesGravityY * playrho::MeterPerSquareSecond};
    auto conf = playrho::d2::Rectangle<
        playrho::d2::Geometry::Mutable, 0, 0,
        playrho::shape_part::VertexRadiusIs<playrho::shape_part::DynamicVertexRadius<>>>{};
    conf.vertexRadius = vertexRadius;
    auto world = playrho::d2::World{
        playrho::d2::WorldConf{}.UseMinVertexRadius(vertexRadius).UseTreeCapacity(8192)};

    {
        constexpr auto a = 0.5f;
        auto ground = playrho::d2::Body{
            playrho::d2::BodyConf{}.UseLocation(playrho::Length2{0, -a * playrho::Meter})};
        constexpr auto N = TilesWidth;
        constexpr auto M = TilesHeight;
        playrho::Length2 position{};
        if (groundIsComboShape) {
            SetDimensions(conf, playrho::Length2{1 * playrho::Meter, 1 * playrho::Meter});
            // y max = 0.5_m, y min = -9.5_m, y/2 = -4.5_m
            GetY(position) = 0.0f * playrho::Meter;
            for (auto j = 0; j < M; ++j) {
                GetX(position) = -N * a * playrho::Meter;
                for (auto i = 0; i < N; ++i) {
                    SetOffset(conf, position);
                    ground.Attach(CreateShape(world, conf));
                    GetX(position) += 2.0f * a * playrho::Meter;
                }
                GetY(position) -= 2.0f * a * playrho::Meter;
            }
        }
        else {
            GetY(position) = -4.5f * playrho::Meter;
            SetDimensions(conf, playrho::Length2{N * playrho::Meter, M * playrho::Meter});
            SetOffset(conf, position);
            ground.Attach(CreateShape(world, conf));
        }
        CreateBody(world, ground);
    }

    {
        const auto shapeId = world.CreateShape(playrho::d2::Shape(
            playrho::d2::Rectangle<
                playrho::d2::Geometry::Constant, 1, 1,
                playrho::shape_part::DensityIs<playrho::shape_part::StaticAreaDensity<5>>>{}));

        playrho::Length2 x(-7.0f * playrho::Meter, 0.75f * playrho::Meter);
        playrho::Length2 y;
        constexpr auto deltaX =
            playrho::Length2(DeltaXx * playrho::Meter, DeltaXy * playrho::Meter);
        constexpr auto deltaY =
            playrho::Length2(DeltaYx * playrho::Meter, DeltaYy * playrho::Meter);

        for (auto i = 0; i < count; ++i) {
            y = x;
            for (auto j = i; j < count; ++j) {
                auto body = playrho::d2::Body{playrho::d2::BodyConf{}
                                                  .UseType(playrho::BodyType::Dynamic)
                                                  .UseLocation(y)
                                                  .UseLinearAcceleration(gravity)};
                body.Attach(shapeId);
                CreateBody(world, body);
                y += deltaY;
            }
            x += deltaX;
        }
    }

    auto step = playrho::StepConf{};
    step.deltaTime = playrho::Second / 60;
    step.linearSlop = linearSlop;
    step.angularSlop = angularSlop;
    step.regMinSeparation = -linearSlop * 3;
    step.toiMinSeparation = -linearSlop * 1.5f;
    step.targetDepth = linearSlop * 3;
    step.tolerance = linearSlop / 4;
    step.maxLinearCorrection = 0.2f * playrho::Meter;
    step.maxAngularCorrection = (8.0f / 180.0f * playrho::Pi) * playrho::Radian;
    step.aabbExtension = 0.1f * playrho::Meter;
    step.displaceMultiplier = 4.0f;
    step.maxTranslation = 2.0f * playrho::Meter;
    step.maxRotation = playrho::Pi * playrho::Real(0.5f) * playrho::Radian;
    step.velocityThreshold = 1.0f * playrho::MeterPerSecond;
    step.maxSubSteps = std::uint8_t{8};
    step.regPositionIters = 3;
    step.toiVelocityIters = 8;

    while (GetAwakeCount(world) > 0) {
        world.Step(step);
    }
}

#ifdef BENCHMARK_BOX2D
static unsigned GetAwakeCount(const b2World& world)
{
    auto count = 0u;
    for (auto body = world.GetBodyList(); body; body = body->GetNext()) {
        if (body->IsAwake())
            ++count;
    }
    return count;
}

static void DropTilesBox2D(int count, bool groundIsComboShape = true)
{
    b2Vec2 gravity;
    gravity.Set(TilesGravityX, TilesGravityY);
    b2World world(gravity);
    {
        const auto a = 0.5f;
        auto bodyConf = b2BodyDef{};
        bodyConf.position = b2Vec2{0, -a};
        const auto ground = world.CreateBody(&bodyConf);

        const auto N = TilesWidth;
        const auto M = TilesHeight;
        auto position = b2Vec2(0.0f, 0.0f);
        if (groundIsComboShape) {
            for (auto j = 0; j < M; ++j) {
                position.x = -N * a;
                for (auto i = 0; i < N; ++i) {
                    b2PolygonShape shape;
                    shape.SetAsBox(a, a, position, 0.0f);
                    ground->CreateFixture(&shape, 0.0f);
                    position.x += 2.0f * a;
                }
                position.y -= 2.0f * a;
            }
        }
        else {
            position.y = -4.5f;
            b2PolygonShape shape;
            shape.SetAsBox(a * playrho::Meter * N, a * playrho::Meter * M, position,
                           playrho::Angle{0});
            ground->CreateFixture(&shape, 0.0f);
        }
    }

    {
        const auto a = 0.5f;
        b2PolygonShape shape;
        shape.SetAsBox(a, a);

        b2Vec2 x(-7.0f, 0.75f);
        b2Vec2 y;
        b2Vec2 deltaX(DeltaXx, DeltaXy);
        b2Vec2 deltaY(DeltaYx, DeltaYy);

        for (auto i = 0; i < count; ++i) {
            y = x;
            for (auto j = i; j < count; ++j) {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position = y;
                const auto body = world.CreateBody(&bd);
                body->CreateFixture(&shape, 5.0f);
                y += deltaY;
            }
            x += deltaX;
        }
    }

    while (GetAwakeCount(world) > 0) {
        world.Step(1.0f / 60, 8, 3);
    }
}
#endif // BENCHMARK_BOX2D

static void TilesRestComboGroundPlayRho(benchmark::State& state)
{
    const auto range = state.range();
    for (auto _ : state) {
        DropTilesPlayRho(range);
    }
}

static void TilesRestOneGroundPlayRho(benchmark::State& state)
{
    const auto range = state.range();
    for (auto _ : state) {
        DropTilesPlayRho(range, false);
    }
}

#ifdef BENCHMARK_BOX2D
static void TilesRestComboGroundBox2D(benchmark::State& state)
{
    const auto range = state.range();
    for (auto _ : state) {
        DropTilesBox2D(range);
    }
}

static void TilesRestOneGroundBox2D(benchmark::State& state)
{
    const auto range = state.range();
    for (auto _ : state) {
        DropTilesBox2D(range, false);
    }
}
#endif // BENCHMARK_BOX2D

class TumblerPlayRho
{
public:
    TumblerPlayRho();
    void Step();
    void AddSquare();
    bool IsWithin(const playrho::d2::AABB& aabb) const;

private:
    static playrho::BodyID CreateEnclosure(playrho::d2::World& world);
    static playrho::JointID CreateRevoluteJoint(playrho::d2::World& world, playrho::BodyID stable,
                                                playrho::BodyID turn);
    static playrho::ShapeID CreateSquareShape(playrho::d2::World& world, playrho::Length squareLen);

    playrho::d2::World m_world{playrho::d2::WorldConf{}.UseContactCapacity(9600u)};
    playrho::StepConf m_stepConf;
    playrho::Length m_squareLen = 0.25f * playrho::Meter; // full width & height unlike Box2D!!
    playrho::ShapeID m_squareId = CreateSquareShape(m_world, m_squareLen);
};

TumblerPlayRho::TumblerPlayRho()
{
    const auto g = CreateBody(m_world, playrho::d2::BodyConf{}.UseType(playrho::BodyType::Static));
    const auto b = CreateEnclosure(m_world);
    CreateRevoluteJoint(m_world, g, b);

    constexpr auto linearSlop = 0.005f * playrho::Meter;
    constexpr auto angularSlop = (2.0f / 180.0f * playrho::Pi) * playrho::Radian;

    auto step = playrho::StepConf{};
    step.deltaTime = playrho::Second / 60;
    step.linearSlop = linearSlop;
    step.angularSlop = angularSlop;
    step.regMinSeparation = -linearSlop * 3;
    step.toiMinSeparation = -linearSlop * 1.5f;
    step.targetDepth = linearSlop * 3;
    step.tolerance = linearSlop / 4;
    step.maxLinearCorrection = 0.2f * playrho::Meter;
    step.maxAngularCorrection = (8.0f / 180.0f * playrho::Pi) * playrho::Radian;
    step.aabbExtension = 0.1f * playrho::Meter;
    step.displaceMultiplier = 4.0f;
    step.maxTranslation = 2.0f * playrho::Meter;
    step.maxRotation = playrho::Pi * playrho::Real(0.5f) * playrho::Radian;
    step.velocityThreshold = 1.0f * playrho::MeterPerSecond;
    step.maxSubSteps = std::uint8_t{8};
    step.regPositionIters = 3;
    step.toiVelocityIters = 8;
    m_stepConf = step;
}

playrho::BodyID TumblerPlayRho::CreateEnclosure(playrho::d2::World& world)
{
    auto b = playrho::d2::Body{playrho::d2::BodyConf{}
                                   .UseType(playrho::BodyType::Dynamic)
                                   .UseLocation(playrho::Vec2(0, 10) * playrho::Meter)
                                   .UseAllowSleep(false)};
    auto conf = playrho::d2::Rectangle<
        playrho::d2::Geometry::Mutable, 0, 0,
        playrho::shape_part::DensityIs<playrho::shape_part::StaticAreaDensity<5>>>{};
    SetDimensions(conf, playrho::Length2{1 * playrho::Meter, 20 * playrho::Meter});
    SetOffset(conf, playrho::Vec2(10.0f, 0.0f) * playrho::Meter);
    b.Attach(world.CreateShape(playrho::d2::Shape(conf)));
    SetDimensions(conf, playrho::Length2{1 * playrho::Meter, 20 * playrho::Meter});
    SetOffset(conf, playrho::Vec2(-10.0f, 0.0f) * playrho::Meter);
    b.Attach(world.CreateShape(playrho::d2::Shape(conf)));
    SetDimensions(conf, playrho::Length2{20 * playrho::Meter, 1 * playrho::Meter});
    SetOffset(conf, playrho::Vec2(0.0f, 10.0f) * playrho::Meter);
    b.Attach(world.CreateShape(playrho::d2::Shape(conf)));
    SetDimensions(conf, playrho::Length2{20 * playrho::Meter, 1 * playrho::Meter});
    SetOffset(conf, playrho::Vec2(0.0f, -10.0f) * playrho::Meter);
    b.Attach(world.CreateShape(playrho::d2::Shape(conf)));
    return CreateBody(world, b);
}

playrho::ShapeID TumblerPlayRho::CreateSquareShape(playrho::d2::World& world,
                                                   playrho::Length squareLen)
{
    auto conf = playrho::d2::Rectangle<
        playrho::d2::Geometry::Mutable, 0, 0,
        playrho::shape_part::DensityIs<playrho::shape_part::StaticAreaDensity<1>>>{};
    conf.SetDimensions(playrho::Length2{squareLen, squareLen});
    return CreateShape(world, playrho::d2::Shape{conf});
}

playrho::JointID TumblerPlayRho::CreateRevoluteJoint(playrho::d2::World& world,
                                                     playrho::BodyID stable, playrho::BodyID turn)
{
    playrho::d2::RevoluteJointConf jd;
    jd.bodyA = stable;
    jd.bodyB = turn;
    jd.localAnchorA = playrho::Vec2(0.0f, 10.0f) * playrho::Meter;
    jd.localAnchorB = playrho::Length2{};
    jd.referenceAngle = playrho::Angle{0};
    jd.motorSpeed = 0.05f * playrho::Pi * playrho::Radian / playrho::Second;
    jd.maxMotorTorque = 100000 * playrho::NewtonMeter; // 1e8f;
    jd.enableMotor = true;
    return world.CreateJoint(playrho::d2::Joint(jd));
}

void TumblerPlayRho::Step()
{
    m_world.Step(m_stepConf);
}

void TumblerPlayRho::AddSquare()
{
    auto b = playrho::d2::Body{playrho::d2::BodyConf{}
                                   .UseType(playrho::BodyType::Dynamic)
                                   .UseLocation(playrho::Vec2(0, 10) * playrho::Meter)
                                   .UseLinearAcceleration(playrho::d2::EarthlyGravity)};
    b.Attach(m_squareId);
    CreateBody(m_world, b);
}

bool TumblerPlayRho::IsWithin(const playrho::d2::AABB& aabb) const
{
    return playrho::d2::Contains(aabb, GetAABB(m_world.GetTree()));
}

static void TumblerAddSquaresForStepsPlayRho(benchmark::State& state, int additionalSteps)
{
    using namespace std::chrono_literals;
    const auto rangeX =
        playrho::Interval<playrho::Length>{-15 * playrho::Meter, +15 * playrho::Meter};
    const auto rangeY =
        playrho::Interval<playrho::Length>{-5 * playrho::Meter, +25 * playrho::Meter};
    const auto aabb = playrho::d2::AABB{rangeX, rangeY};
    const auto squareAddingSteps = state.range();
    for (auto _ : state) {
        state.PauseTiming();
        TumblerPlayRho tumbler;
        std::this_thread::sleep_for(2000ms);
        for (auto i = 0; i < squareAddingSteps; ++i) {
            tumbler.Step();
            tumbler.AddSquare();
        }
        state.ResumeTiming();
        for (auto i = 0; i < additionalSteps; ++i) {
            tumbler.Step();
        }
        if (!tumbler.IsWithin(aabb)) {
            std::cout << "escaped!" << std::endl;
            continue;
        }
    }
}

static void TumblerAddSquaresPlus60StepsPlayRho(benchmark::State& state)
{
    TumblerAddSquaresForStepsPlayRho(state, 60);
}

#ifdef BENCHMARK_BOX2D

class TumblerBox2D
{
public:
    TumblerBox2D();
    void Step();
    void AddSquare();
    bool IsWithin(const b2AABB& aabb) const;

private:
    static b2Body* CreateEnclosure(b2World& world);
    static b2RevoluteJoint* CreateRevoluteJoint(b2World& world, b2Body* stable, b2Body* turn);

    b2World m_world{
        b2Vec2(0.0f, playrho::EarthlyLinearAcceleration / playrho::MeterPerSquareSecond)};
};

TumblerBox2D::TumblerBox2D()
{
    b2BodyDef bd;
    const auto ground = m_world.CreateBody(&bd);
    const auto enclosure = CreateEnclosure(m_world);
    CreateRevoluteJoint(m_world, ground, enclosure);
}

b2Body* TumblerBox2D::CreateEnclosure(b2World& world)
{
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.allowSleep = false;
    bd.position.Set(0.0f, 10.0f);
    const auto body = world.CreateBody(&bd);
    b2PolygonShape shape;
    shape.SetAsBox(0.5f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0);
    body->CreateFixture(&shape, 5.0f);
    shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
    body->CreateFixture(&shape, 5.0f);
    shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
    body->CreateFixture(&shape, 5.0f);
    shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
    body->CreateFixture(&shape, 5.0f);
    return body;
}

b2RevoluteJoint* TumblerBox2D::CreateRevoluteJoint(b2World& world, b2Body* stable, b2Body* turn)
{
    b2RevoluteJointDef jd;
    jd.bodyA = stable;
    jd.bodyB = turn;
    jd.localAnchorA.Set(0.0f, 10.0f);
    jd.localAnchorB.Set(0.0f, 0.0f);
    jd.referenceAngle = 0.0f;
    jd.motorSpeed = 0.05f * b2_pi;
    jd.maxMotorTorque = 1e8f;
    jd.enableMotor = true;
    return static_cast<b2RevoluteJoint*>(world.CreateJoint(&jd));
}

void TumblerBox2D::AddSquare()
{
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(0.0f, 10.0f);
    const auto body = m_world.CreateBody(&bd);
    b2PolygonShape shape;
    shape.SetAsBox(0.125f, 0.125f); // Box2D treats these as half width & half height lengths!!
    body->CreateFixture(&shape, 1.0f);
}

void TumblerBox2D::Step()
{
    m_world.Step(1.0f / 60.0f, 8, 3);
}

bool TumblerBox2D::IsWithin(const b2AABB&) const
{
    return true;
}

static void TumblerAddSquaresForStepsBox2D(benchmark::State& state, int additionalSteps)
{
    const auto squareAddingSteps = state.range();
    const auto aabb = b2AABB{b2Vec2{-15.0f, -5.0f}, b2Vec2{15.0f, 25.0f}};
    TumblerBox2D tumbler;
    for (auto i = 0; i < squareAddingSteps; ++i) {
        tumbler.Step();
        tumbler.AddSquare();
    }
    for (auto _ : state) {
        for (auto i = 0; i < additionalSteps; ++i) {
            tumbler.Step();
        }
#if 0
        if (!tumbler.IsWithin(aabb)) {
            std::cout << "escaped!" << std::endl;
            continue;
        }
#endif
    }
}

static void TumblerAddSquaresPlus60StepsBox2D(benchmark::State& state)
{
    TumblerAddSquaresForStepsBox2D(state, 60);
}

#endif // BENCHMARK_BOX2D

#if 0
#define ADD_BM(n, f) BENCHMARK_PRIVATE_DECLARE(f) = benchmark::RegisterBenchmark(n, f);
#endif

BENCHMARK(FloatAdd)->Arg(1000);
BENCHMARK(FloatMul)->Arg(1000);
BENCHMARK(FloatMulAdd)->Arg(1000);
BENCHMARK(FloatDiv)->Arg(1000);
BENCHMARK(FloatSqrt)->Arg(1000);
BENCHMARK(FloatSin)->Arg(1000);
BENCHMARK(FloatCos)->Arg(1000);
BENCHMARK(FloatSinCos)->Arg(1000);
BENCHMARK(FloatAtan2)->Arg(1000);
BENCHMARK(FloatHypot)->Arg(1000);
BENCHMARK(FloatFma)->Arg(1000);

BENCHMARK(DoubleAdd)->Arg(1000);
BENCHMARK(DoubleMul)->Arg(1000);
BENCHMARK(DoubleMulAdd)->Arg(1000);
BENCHMARK(DoubleDiv)->Arg(1000);
BENCHMARK(DoubleSqrt)->Arg(1000);
BENCHMARK(DoubleSin)->Arg(1000);
BENCHMARK(DoubleCos)->Arg(1000);
BENCHMARK(DoubleSinCos)->Arg(1000);
BENCHMARK(DoubleAtan2)->Arg(1000);
BENCHMARK(DoubleHypot)->Arg(1000);
BENCHMARK(DoubleFma)->Arg(1000);

BENCHMARK(AlmostEqual1)->Arg(1000);
BENCHMARK(AlmostEqual2)->Arg(1000);
BENCHMARK(AlmostEqual3)->Arg(1000);
BENCHMARK(DiffSignsViaSignbit)->Arg(1000);
BENCHMARK(DiffSignsViaMul)->Arg(1000);
BENCHMARK(ModuloViaTrunc)->Arg(1000);
BENCHMARK(ModuloViaFmod)->Arg(1000);

BENCHMARK(DotProduct)->Arg(1000);
BENCHMARK(CrossProduct)->Arg(1000);
BENCHMARK(LengthSquaredViaDotProduct)->Arg(1000);
BENCHMARK(GetMagnitudeSquared)->Arg(1000);
BENCHMARK(GetMagnitude)->Arg(1000);
BENCHMARK(GetUnitVec1)->Arg(1000);
BENCHMARK(GetUnitVec2)->Arg(1000);
BENCHMARK(UnitVectorFromVector)->Arg(1000);
BENCHMARK(UnitVectorFromVectorAndBack)->Arg(1000);
BENCHMARK(UnitVecFromAngle)->Arg(1000);

BENCHMARK(LessLength)->Arg(1000);
BENCHMARK(LessFloat)->Arg(1000);
BENCHMARK(LessDouble)->Arg(1000);

BENCHMARK(LessEqualLength)->Arg(1000);
BENCHMARK(LessEqualFloat)->Arg(1000);
BENCHMARK(LessEqualDouble)->Arg(1000);

BENCHMARK(LesserLength)->Arg(1000);
BENCHMARK(LesserFloat)->Arg(1000);
BENCHMARK(LesserDouble)->Arg(1000);

BENCHMARK(LesserEqualLength)->Arg(1000);
BENCHMARK(LesserEqualFloat)->Arg(1000);
BENCHMARK(LesserEqualDouble)->Arg(1000);

BENCHMARK(MinLength)->Arg(1000);
BENCHMARK(MinFloat)->Arg(1000);
BENCHMARK(MinDouble)->Arg(1000);

BENCHMARK(IntervalIsIntersecting)->Arg(1000);
BENCHMARK(LengthIntervalIsIntersecting)->Arg(1000);
BENCHMARK(AabbTestOverlap)->Arg(1000);
BENCHMARK(AabbContains)->Arg(1000);
BENCHMARK(AABB)->Arg(1000);
// BENCHMARK(malloc_free_random_size);

// BENCHMARK(MaxSepBetweenAbsSquares);
BENCHMARK(MaxSepBetweenRel4x4)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(MaxSepBetweenRelSquaresNoStop)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(MaxSepBetweenRelSquares)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);

BENCHMARK(ConstructAndAssignVC);
BENCHMARK(SolveVC);

BENCHMARK(ManifoldForTwoSquares1);
BENCHMARK(ManifoldForTwoSquares2);

BENCHMARK(AsyncFutureDeferred);
BENCHMARK(AsyncFutureAsync);
#ifdef BENCHMARK_GCDISPATCH
BENCHMARK(AsyncFutureDispatch);
#endif // BENCHMARK_GCDISPATCH
BENCHMARK(ThreadCreateAndDestroy);
BENCHMARK(MultiThreadQD);
BENCHMARK(MultiThreadQDE);
BENCHMARK(MultiThreadQDA);
BENCHMARK(MultiThreadQDAQ);

BENCHMARK(WorldStepPlayRho);
#ifdef BENCHMARK_BOX2D
BENCHMARK(WorldStepBox2D);
#endif

BENCHMARK(CreateBodyWithOneShapePlayRho)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#ifdef BENCHMARK_BOX2D
BENCHMARK(CreateBodyWithOneShapeBox2D)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#endif

// Next two benchmarks can have a stddev time of some 20% between repeats.
BENCHMARK(WorldStepWithStatsStaticPlayRho)
    ->Arg(0)
    ->Arg(1)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000);
// BENCHMARK(WorldStepWithStatsDynamicBodies)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000)->Repetitions(4);
#ifdef BENCHMARK_BOX2D
BENCHMARK(WorldStepWithStatsStaticBox2D)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#endif

BENCHMARK(DropDisksPlayRho)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#ifdef BENCHMARK_BOX2D
BENCHMARK(DropDisksBox2D)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#endif

BENCHMARK(DropDisksSixtyStepsPlayRho)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#ifdef BENCHMARK_BOX2D
BENCHMARK(DropDisksSixtyStepsBox2D)->Arg(0)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
#endif

// BENCHMARK(random_malloc_free_100);

BENCHMARK(TumblerAddSquaresPlus60StepsPlayRho)->Arg(100)->Arg(200)->Arg(400)->Arg(800)->Arg(1600);
#ifdef BENCHMARK_BOX2D
BENCHMARK(TumblerAddSquaresPlus60StepsBox2D)->Arg(100)->Arg(200)->Arg(400)->Arg(800)->Arg(1600);
#endif

BENCHMARK(AddPairStressTestPlayRho400)
    ->Arg(0)
    ->Arg(10)
    ->Arg(15)
    ->Arg(16)
    ->Arg(17)
    ->Arg(18)
    ->Arg(19)
    ->Arg(20)
    ->Arg(30);
#ifdef BENCHMARK_BOX2D
BENCHMARK(AddPairStressTestBox2D400)
    ->Arg(0)
    ->Arg(10)
    ->Arg(15)
    ->Arg(16)
    ->Arg(17)
    ->Arg(18)
    ->Arg(19)
    ->Arg(20)
    ->Arg(30);
#endif // BENCHMARK_BOX2D

BENCHMARK(TilesRestComboGroundPlayRho)->Arg(12)->Arg(20)->Arg(36);
BENCHMARK(TilesRestOneGroundPlayRho)->Arg(12)->Arg(20)->Arg(36);

#ifdef BENCHMARK_BOX2D
BENCHMARK(TilesRestComboGroundBox2D)->Arg(12)->Arg(20)->Arg(36);
BENCHMARK(TilesRestOneGroundBox2D)->Arg(12)->Arg(20)->Arg(36);
#endif // BENCHMARK_BOX2D

// BENCHMARK_MAIN()
int main(int argc, char** argv)
{
    ::benchmark::Initialize(&argc, argv);
    if (::benchmark::ReportUnrecognizedArguments(argc, argv))
        return 1;

    std::srand(
        static_cast<unsigned>(std::time(0))); // use current time as seed for random generator
    ::benchmark::RunSpecifiedBenchmarks();
}
