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

#ifdef USE_BOOST_UNITS
#define PLAYRHO_QUANTITY(BoostDimension) boost::units::quantity<BoostDimension, Real>
#define PLAYRHO_UNIT(Quantity, BoostUnit) Quantity{BoostUnit * Real{1}}
#define PLAYRHO_DERIVED_UNIT(Quantity, BoostUnit, Ratio) Quantity{BoostUnit * Real{Ratio}}
#else
#define PLAYRHO_QUANTITY(BoostDimension) Real
#define PLAYRHO_UNIT(Quantity, BoostUnit) Real{1}
#define PLAYRHO_DERIVED_UNIT(Quantity, BoostUnit, Ratio) Real{Ratio}}
#endif

namespace playrho
{
    /// @brief Time quantity.
    using Time = PLAYRHO_QUANTITY(boost::units::si::time);

    /// @brief Second unit of Time.
    constexpr auto Second = PLAYRHO_UNIT(Time, boost::units::si::second);

    /// @brief Frequency quantity.
    using Frequency = PLAYRHO_QUANTITY(boost::units::si::frequency);

    /// @brief Hertz unit of Frequency.
    constexpr auto Hertz = PLAYRHO_UNIT(Frequency, boost::units::si::hertz);

    /// @brief Length type.
    using Length = PLAYRHO_QUANTITY(boost::units::si::length);

    /// @brief Meter unit of Length.
    constexpr auto Meter = PLAYRHO_UNIT(Length, boost::units::si::meter);

    /// @brief Linear velocity quantity.
    using LinearVelocity = PLAYRHO_QUANTITY(boost::units::si::velocity);

    /// @brief Meter per second unit of LinearVelocity.
    constexpr auto MeterPerSecond = PLAYRHO_UNIT(LinearVelocity, boost::units::si::meter_per_second);

    /// @brief Linear acceleration quantity.
    using LinearAcceleration = PLAYRHO_QUANTITY(boost::units::si::acceleration);

    /// @brief Meter per square second unit of LinearAcceleration.
    constexpr auto MeterPerSquareSecond = PLAYRHO_UNIT(LinearAcceleration, boost::units::si::meter_per_second_squared);

    /// @brief Mass quantity.
    using Mass = PLAYRHO_QUANTITY(boost::units::si::mass);

    /// @brief Kilogram unit of Mass.
    constexpr auto Kilogram = PLAYRHO_UNIT(Mass, boost::units::si::kilogram);

    /// @brief Inverse mass quantity.
    using InvMass = PLAYRHO_QUANTITY(boost::units::si::inverse_mass);
    
    /// @brief Area quantity.
    using Area = PLAYRHO_QUANTITY(boost::units::si::area);

    /// @brief Square meter unit of Area.
    constexpr auto SquareMeter = PLAYRHO_UNIT(Area, boost::units::si::square_meter);
    
    /// @brief Aereal/surface density quantity.
    using Density = PLAYRHO_QUANTITY(boost::units::si::surface_density);

    /// @brief Kilogram per square meter unit of Density.
    constexpr auto KilogramPerSquareMeter = PLAYRHO_UNIT(Density, boost::units::si::kilogram_per_square_meter);
    
    /// @brief Angle quantity.
    using Angle = PLAYRHO_QUANTITY(boost::units::si::plane_angle);

    /// @brief Radian unit of Angle.
    constexpr auto Radian = PLAYRHO_UNIT(Angle, boost::units::si::radian);
    
    /// @brief Degree unit of Angle quantity.
    constexpr auto Degree = Angle{Radian * Pi / Real{180}};
    
    /// @brief Square radian unit type.
    constexpr auto SquareRadian = Radian * Radian;
    
    /// @brief Angular velocity quantity.
    using AngularVelocity = PLAYRHO_QUANTITY(boost::units::si::angular_velocity);

    /// @brief Radian per second unit of AngularVelocity.
    constexpr auto RadianPerSecond = PLAYRHO_UNIT(AngularVelocity, boost::units::si::radian_per_second);
    
    /// @brief Degree per second unit of AngularVelocity.
    constexpr auto DegreePerSecond = AngularVelocity{RadianPerSecond * Degree / Radian};
    
    /// @brief Angular acceleration quantity.
    using AngularAcceleration = PLAYRHO_QUANTITY(boost::units::si::angular_acceleration);

    /// @brief Radian per square second unit of AngularAcceleration.
    constexpr auto RadianPerSquareSecond = Radian / (Second * Second);
    
    /// @brief Force quantity.
    using Force = PLAYRHO_QUANTITY(boost::units::si::force);

    /// @brief Newton unit of Force.
    constexpr auto Newton = PLAYRHO_UNIT(Force, boost::units::si::newton);
    
    /// @brief Torque quantity.
    using Torque = PLAYRHO_QUANTITY(boost::units::si::torque);

    /// @brief Newton meter unit of Torque.
    constexpr auto NewtonMeter = PLAYRHO_UNIT(Torque, boost::units::si::newton_meter);
    
    /// @brief Second moment of area quantity.
    using SecondMomentOfArea = PLAYRHO_QUANTITY(boost::units::si::second_moment_of_area);
    
    /// @brief Rotational inertia quantity.
    using RotInertia = PLAYRHO_QUANTITY(boost::units::si::moment_of_inertia);
    
    /// @brief Inverse rotational inertia quantity.
    /// @note Units of L^-2 M^-1 QP^2.
    using InvRotInertia = PLAYRHO_QUANTITY(boost::units::si::inverse_moment_of_inertia);
    
    /// @brief Momentum quantity.
    using Momentum = PLAYRHO_QUANTITY(boost::units::si::momentum);

    /// @brief Newton second unit.
    constexpr auto NewtonSecond = Newton * Second;
    
    /// @brief Angular momentum quantity.
    /// @note Units of L^2 M T^-1 QP^-1.
    using AngularMomentum = PLAYRHO_QUANTITY(boost::units::si::angular_momentum);
    
    /// @brief Strips the units off of the given value.
    constexpr inline Real StripUnit(const Real value)
    {
        return value;
    }
    
#ifdef USE_BOOST_UNITS
    
    /// @brief Strips the units off of the given value.
    template<typename Y>
    inline constexpr auto StripUnit(const boost::units::quantity<Y, Real> source)
    {
        return source.value();
    }

    /// @brief Gets an invalid value for the Angle type.
    template <>
    constexpr Angle GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Radian;
    }
    
    /// @brief Gets an invalid value for the Frequency type.
    template <>
    constexpr Frequency GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Hertz;
    }
    
    /// @brief Gets an invalid value for the AngularVelocity type.
    template <>
    constexpr AngularVelocity GetInvalid() noexcept
    {
        return GetInvalid<Real>() * RadianPerSecond;
    }
    
    /// @brief Gets an invalid value for the Time type.
    template <>
    constexpr Time GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Second;
    }
    
    /// @brief Gets an invalid value for the Length type.
    template <>
    constexpr Length GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Meter;
    }
    
    /// @brief Gets an invalid value for the Mass type.
    template <>
    constexpr Mass GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Kilogram;
    }
    
    /// @brief Gets an invalid value for the InvMass type.
    template <>
    constexpr InvMass GetInvalid() noexcept
    {
        return GetInvalid<Real>() / Kilogram;
    }
    
    /// @brief Gets an invalid value for the Momentum type.
    template <>
    constexpr Momentum GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Kilogram * MeterPerSecond;
    }
    
    /// @brief Gets an invalid value for the Force type.
    template <>
    constexpr Force GetInvalid() noexcept
    {
        return GetInvalid<Real>() * Newton;
    }
    
    /// @brief Gets an invalid value for the Torque type.
    template <>
    constexpr Torque GetInvalid() noexcept
    {
        return GetInvalid<Real>() * NewtonMeter;
    }
    
    /// @brief Gets an invalid value for the LinearVelocity type.
    template <>
    constexpr LinearVelocity GetInvalid() noexcept
    {
        return GetInvalid<Real>() * MeterPerSecond;
    }
    
    /// @brief Gets an invalid value for the LinearAcceleration type.
    template <>
    constexpr LinearAcceleration GetInvalid() noexcept
    {
        return GetInvalid<Real>() * MeterPerSquareSecond;
    }
    
    /// @brief Gets an invalid value for the AngularAcceleration type.
    template <>
    constexpr AngularAcceleration GetInvalid() noexcept
    {
        return GetInvalid<Real>() * RadianPerSquareSecond;
    }
    
    /// @brief Gets an invalid value for the RotInertia type.
    template <>
    constexpr RotInertia GetInvalid() noexcept
    {
        // RotInertia is L^2  M    QP^-2
        return GetInvalid<Real>() * SquareMeter * Kilogram / SquareRadian;
    }
    
#endif

} // namespace playrho

#undef PLAYRHO_QUANTITY
#undef PLAYRHO_UNIT

#endif /* Units_hpp */
