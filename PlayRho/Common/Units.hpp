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

/**
 * @file
 * @brief Units file.
 */

#ifndef Units_hpp
#define Units_hpp

#include <PlayRho/Common/RealNum.hpp>
#include <PlayRho/Common/Templates.hpp>

// #define USE_BOOST_UNITS
#ifdef USE_BOOST_UNITS
#include <boost/units/io.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/acceleration.hpp>
#include <boost/units/systems/si/frequency.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/mass.hpp>
#include <boost/units/systems/si/momentum.hpp>
#include <boost/units/systems/si/inverse_mass.hpp>
#include <boost/units/systems/si/area.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/angular_momentum.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <boost/units/systems/si/second_moment_of_area.hpp>
#include <boost/units/systems/si/surface_density.hpp>
#include <boost/units/systems/si/moment_of_inertia.hpp>
#include <boost/units/systems/si/inverse_moment_of_inertia.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#endif

namespace playrho
{    
#ifdef USE_BOOST_UNITS
    
    using Time = boost::units::quantity<boost::units::si::time, Real>;
    constexpr auto Second = Time{boost::units::si::second * Real{1}};
    
    using Frequency = boost::units::quantity<boost::units::si::frequency, Real>;
    constexpr auto Hertz = Frequency{boost::units::si::hertz * Real{1}};
    
    using Length = boost::units::quantity<boost::units::si::length, Real>;
    constexpr auto Meter = Length{boost::units::si::meter * Real{1}};
    
    using LinearVelocity = boost::units::quantity<boost::units::si::velocity, Real>;
    constexpr auto MeterPerSecond = LinearVelocity{boost::units::si::meter_per_second * Real{1}};
    
    using LinearAcceleration = boost::units::quantity<boost::units::si::acceleration, Real>;
    constexpr auto MeterPerSquareSecond = LinearAcceleration{boost::units::si::meter_per_second_squared * Real{1}};
    
    using Mass = boost::units::quantity<boost::units::si::mass, Real>;
    constexpr auto Kilogram = Mass{boost::units::si::kilogram * Real{1}};
    
    using InvMass = boost::units::quantity<boost::units::si::inverse_mass, Real>;
    
    using Area = boost::units::quantity<boost::units::si::area, Real>;
    constexpr auto SquareMeter = Area{boost::units::si::square_meter * Real{1}};
    
    using Density = boost::units::quantity<boost::units::si::surface_density, Real>;
    constexpr auto KilogramPerSquareMeter = Density{boost::units::si::kilogram_per_square_meter * Real{1}};
    
    using Angle = boost::units::quantity<boost::units::si::plane_angle, Real>;
    constexpr auto Radian = Angle{boost::units::si::radian * Real{1}};
    constexpr auto Degree = Angle{boost::units::degree::degree * Real{1}};
    constexpr auto SquareRadian = Radian * Radian;
    
    using AngularVelocity = boost::units::quantity<boost::units::si::angular_velocity, Real>;
    constexpr auto RadianPerSecond = AngularVelocity{boost::units::si::radian_per_second * Real{1}};
    constexpr auto DegreePerSecond = AngularVelocity{RadianPerSecond * Degree / Radian};
    
    using AngularAcceleration = boost::units::quantity<boost::units::si::angular_acceleration, Real>;
    constexpr auto RadianPerSquareSecond = Radian / (Second * Second);
    
    using Force = boost::units::quantity<boost::units::si::force, Real>;
    constexpr auto Newton = Force{boost::units::si::newton * Real{1}};
    
    using Torque = boost::units::quantity<boost::units::si::torque, Real>;
    constexpr auto NewtonMeter = Torque{boost::units::si::newton_meter * Real{1}};
    
    using SecondMomentOfArea = boost::units::quantity<boost::units::si::second_moment_of_area, Real>;
    
    using RotInertia = boost::units::quantity<boost::units::si::moment_of_inertia, Real>;
    
    /// @brief Inverse rotational inertia.
    /// @note Units of L^-2 M^-1 QP^2.
    using InvRotInertia = boost::units::quantity<boost::units::si::inverse_moment_of_inertia, Real>;
    
    using Momentum = boost::units::quantity<boost::units::si::momentum, Real>;
    constexpr auto NewtonSecond = Newton * Second;
    
    /// @brief Angular momentum.
    /// @note Units of L^2 M T^-1 QP^-1.
    using AngularMomentum = boost::units::quantity<boost::units::si::angular_momentum, Real>;
    
#else // USE_BOOST_UNITS
    
    using Time = Real;
    constexpr auto Second = Real{1};
    
    using Frequency = Real;
    constexpr auto Hertz = Real{1};
    
    using Length = Real;
    constexpr auto Meter = Real{1};
    
    using LinearVelocity = Real;
    constexpr auto MeterPerSecond = Real{1};
    
    using LinearAcceleration = Real;
    constexpr auto MeterPerSquareSecond = Real{1};
    
    using Mass = Real;
    constexpr auto Kilogram = Real{1};
    
    using InvMass = Real;
    
    using Area = Real;
    constexpr auto SquareMeter = Real{1};
    
    using Density = Real;
    constexpr auto KilogramPerSquareMeter = Real{1};
    
    using Angle = Real;
    constexpr auto Radian = Real{1};
    constexpr auto Degree = Pi / Real{180};
    constexpr auto SquareRadian = Radian * Radian;
    
    using AngularVelocity = Real;
    constexpr auto RadianPerSecond = Real{1};
    constexpr auto DegreePerSecond = Degree;
    
    using AngularAcceleration = Real;
    constexpr auto RadianPerSquareSecond = Real{1};
    
    using Force = Real;
    constexpr auto Newton = Real{1};
    
    using Torque = Real;
    constexpr auto NewtonMeter = Real{1};
    
    using SecondMomentOfArea = Real;
    
    using RotInertia = Real;
    using InvRotInertia = Real;
    
    using Momentum = Real;
    constexpr auto NewtonSecond = Real{1};
    
    using AngularMomentum = Real;
    
#endif // USE_BOOST_UNITS
    
    constexpr inline Real StripUnit(const Real value)
    {
        return value;
    }
    
#ifdef USE_BOOST_UNITS
    
    template<typename Y>
    inline constexpr auto StripUnit(const boost::units::quantity<Y, Real> source)
    {
        return source.value();
    }

    template <>
    constexpr Angle GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Radian;
    }
    
    template <>
    constexpr Frequency GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Hertz;
    }
    
    template <>
    constexpr AngularVelocity GetInvalid() noexcept
    {
        return GetInvalid<Real>() * RadianPerSecond;
    }
    
    template <>
    constexpr Time GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Second;
    }
    
    template <>
    constexpr Length GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Meter;
    }
    
    template <>
    constexpr Mass GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Kilogram;
    }
    
    template <>
    constexpr InvMass GetInvalid() noexcept
    {
        return GetInvalid<Real>() / Kilogram;
    }
    
    template <>
    constexpr Momentum GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Kilogram * MeterPerSecond;
    }
    
    template <>
    constexpr Force GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Newton;
    }
    
    template <>
    constexpr Torque GetInvalid() noexcept
    {
        return GetInvalid<Real>() * NewtonMeter;
    }
    
    template <>
    constexpr LinearVelocity GetInvalid() noexcept
    {
        return GetInvalid<Real>() * MeterPerSecond;
    }
    
    template <>
    constexpr LinearAcceleration GetInvalid() noexcept
    {
        return GetInvalid<Real>() * MeterPerSquareSecond;
    }
    
    template <>
    constexpr AngularAcceleration GetInvalid() noexcept
    {
        return GetInvalid<Real>() * RadianPerSquareSecond;
    }
    
    template <>
    constexpr RotInertia GetInvalid() noexcept
    {
        // RotInertia is L^2  M    QP^-2
        return GetInvalid<Real>() * SquareMeter * Kilogram / SquareRadian;
    }
    
#endif

} // namespace playrho

#endif /* Units_hpp */
