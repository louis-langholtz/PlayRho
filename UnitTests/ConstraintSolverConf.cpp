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

#include <PlayRho/Dynamics/StepConf.hpp>

#include <PlayRho/Dynamics/Contacts/ConstraintSolverConf.hpp>

using namespace playrho;

TEST(ConstraintSolverConf, DefaultConstruction)
{
    const ConstraintSolverConf conf;
    EXPECT_EQ(conf.resolutionRate, ConstraintSolverConf::DefaultRegResolutionRate);
    EXPECT_EQ(conf.linearSlop, ConstraintSolverConf::DefaultLinearSlop);
    EXPECT_EQ(conf.angularSlop, ConstraintSolverConf::DefaultAngularSlop);
    EXPECT_EQ(conf.maxLinearCorrection, ConstraintSolverConf::DefaultMaxLinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, ConstraintSolverConf::DefaultMaxAngularCorrection);
}

TEST(ConstraintSolverConf, GetDefaultPositionSolverConf)
{
    const auto conf = GetDefaultPositionSolverConf();
    EXPECT_EQ(conf.resolutionRate, ConstraintSolverConf::DefaultRegResolutionRate);
    EXPECT_EQ(conf.linearSlop, ConstraintSolverConf::DefaultLinearSlop);
    EXPECT_EQ(conf.angularSlop, ConstraintSolverConf::DefaultAngularSlop);
    EXPECT_EQ(conf.maxLinearCorrection, ConstraintSolverConf::DefaultMaxLinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, ConstraintSolverConf::DefaultMaxAngularCorrection);
}

TEST(ConstraintSolverConf, GetDefaultToiPositionSolverConf)
{
    const auto conf = GetDefaultToiPositionSolverConf();
    EXPECT_EQ(conf.resolutionRate, ConstraintSolverConf::DefaultToiResolutionRate);
    EXPECT_EQ(conf.linearSlop, ConstraintSolverConf::DefaultLinearSlop);
    EXPECT_EQ(conf.angularSlop, ConstraintSolverConf::DefaultAngularSlop);
    EXPECT_EQ(conf.maxLinearCorrection, ConstraintSolverConf::DefaultMaxLinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, ConstraintSolverConf::DefaultMaxAngularCorrection);
}

TEST(ConstraintSolverConf, GetRegConstraintSolverConf)
{
    constexpr auto ResolutionRate = Real(42);
    constexpr auto LinearSlop = 11_m;
    constexpr auto AngularSlop = 4.5_deg;
    constexpr auto LinearCorrection = 4_m;
    constexpr auto AngularCorrection = 0.4_deg;

    auto step = StepConf{};
    step.regResolutionRate = ResolutionRate;
    step.linearSlop = LinearSlop;
    step.angularSlop = AngularSlop;
    step.maxLinearCorrection = LinearCorrection;
    step.maxAngularCorrection = AngularCorrection;
    const auto conf = GetRegConstraintSolverConf(step);
    EXPECT_EQ(conf.resolutionRate, ResolutionRate);
    EXPECT_EQ(conf.linearSlop, LinearSlop);
    EXPECT_EQ(conf.angularSlop, AngularSlop);
    EXPECT_EQ(conf.maxLinearCorrection, LinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, AngularCorrection);
}

TEST(ConstraintSolverConf, GetToiConstraintSolverConf)
{
    constexpr auto ResolutionRate = Real(42);
    constexpr auto LinearSlop = 11_m;
    constexpr auto AngularSlop = 4.5_deg;
    constexpr auto LinearCorrection = 4_m;
    constexpr auto AngularCorrection = 0.4_deg;

    auto step = StepConf{};
    step.toiResolutionRate = ResolutionRate;
    step.linearSlop = LinearSlop;
    step.angularSlop = AngularSlop;
    step.maxLinearCorrection = LinearCorrection;
    step.maxAngularCorrection = AngularCorrection;
    const auto conf = GetToiConstraintSolverConf(step);
    EXPECT_EQ(conf.resolutionRate, ResolutionRate);
    EXPECT_EQ(conf.linearSlop, LinearSlop);
    EXPECT_EQ(conf.angularSlop, AngularSlop);
    EXPECT_EQ(conf.maxLinearCorrection, LinearCorrection);
    EXPECT_EQ(conf.maxAngularCorrection, AngularCorrection);
}
