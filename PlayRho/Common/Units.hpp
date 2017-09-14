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
 *
 * @brief Units file.
 *
 * @details This file establishes quantity aliases, unit constants, and associated code
 *   for the expression of physical quantities using recognizably named units of those
 *   quantities. Quantities, as used herein, are types associated with physical quantities
 *   like time and length. Conceptually a given quantity is only expressable in units that
 *   relate to that quantity. For every quantity defined herein, there's typically at least
 *   one conceptually typed unit asscociated with it.
 *
 * @note In the simplest way that the PlayRho library can be configured, there's no compiler
 *   enforcement on the usage of the units for there associated quantities beyond the usage
 *   of the Real type.
 */

#ifndef PLAYRHO_UNITS_HPP
#define PLAYRHO_UNITS_HPP

#include <PlayRho/Common/RealConstants.hpp>
#include <PlayRho/Common/Templates.hpp>
#include <type_traits>

// #define USE_BOOST_UNITS
#if defined(USE_BOOST_UNITS)
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
#endif // defined(USE_BOOST_UNITS)

// Define quantity and unit related macros to abstract away C-preprocessor definitions
#if defined(USE_BOOST_UNITS)
#define PLAYRHO_QUANTITY(BoostDimension) boost::units::quantity<BoostDimension, Real>
#define PLAYRHO_UNIT(Quantity, BoostUnit) Quantity{BoostUnit * Real{1}}
#define PLAYRHO_DERIVED_UNIT(Quantity, BoostUnit, Ratio) Quantity{BoostUnit * Real{Ratio}}
#else // defined(USE_BOOST_UNITS)
#define PLAYRHO_QUANTITY(BoostDimension) Real
#define PLAYRHO_UNIT(Quantity, BoostUnit) Real{1}
#define PLAYRHO_DERIVED_UNIT(Quantity, BoostUnit, Ratio) Real{Ratio}}
#endif // defined(USE_BOOST_UNITS)

namespace playrho
{
    /// @defgroup Quantities Physical quantity type aliases.
    /// @details These are the type aliases for physical quantities like time and length
    ///   that are used by the PlayRho library.
    ///   Conceptually a given quantity is only expressable in the units that are defined
    ///   for that quantity.
    /// @sa Units.
    /// @sa https://en.wikipedia.org/wiki/List_of_physical_quantities
    /// @{
    
    /// @brief Time quantity.
    /// @details This is the type alias for the time base quantity.
    /// @note This quantity's dimension is: time (T).
    /// @note The SI unit of time is the second.
    /// @sa Second.
    /// @sa https://en.wikipedia.org/wiki/Time_in_physics
    using Time = PLAYRHO_QUANTITY(boost::units::si::time);
    
    /// @brief Frequency quantity.
    /// @details This is the type alias for the frequency quantity. It's a derived quantity
    ///   that's the inverse of time.
    /// @note This quantity's dimension is: inverse time (T^-1).
    /// @note The SI unit of frequency is the hertz.
    /// @sa Time.
    /// @sa Hertz.
    /// @sa https://en.wikipedia.org/wiki/Frequency
    using Frequency = PLAYRHO_QUANTITY(boost::units::si::frequency);
    
    /// @brief Length quantity.
    /// @details This is the type alias for the length base quantity.
    /// @note This quantity's dimension is: length (L).
    /// @note The SI unit of length is the meter.
    /// @sa Meter.
    /// @sa https://en.wikipedia.org/wiki/Length
    using Length = PLAYRHO_QUANTITY(boost::units::si::length);
    
    /// @brief Linear velocity quantity.
    /// @details This is the type alias for the linear velocity derived quantity.
    /// @note This quantity's dimensions are: length over time (L T^-1).
    /// @note The SI unit of linear velocity is meters per second.
    /// @sa Length.
    /// @sa Time.
    /// @sa MeterPerSecond.
    /// @sa https://en.wikipedia.org/wiki/Speed
    using LinearVelocity = PLAYRHO_QUANTITY(boost::units::si::velocity);
    
    /// @brief Linear acceleration quantity.
    /// @details This is the type alias for the linear acceleration derived quantity.
    /// @note This quantity's dimensions are: length over time squared (L T^-2).
    /// @note The SI unit of linear acceleration is meters per second squared.
    /// @sa Length.
    /// @sa Time.
    /// @sa LinearVelocity.
    /// @sa MeterPerSquareSecond.
    /// @sa https://en.wikipedia.org/wiki/Acceleration
    using LinearAcceleration = PLAYRHO_QUANTITY(boost::units::si::acceleration);
    
    /// @brief Mass quantity.
    /// @details This is the type alias for the mass base quantity.
    /// @note This quantity's dimension is: mass (M).
    /// @note The SI unit of mass is the kilogram.
    /// @sa Kilogram.
    /// @sa https://en.wikipedia.org/wiki/Mass
    using Mass = PLAYRHO_QUANTITY(boost::units::si::mass);
    
    /// @brief Inverse mass quantity.
    /// @details This is the type alias for the inverse mass quantity. It's a derived quantity
    ///   that's the inverse of mass.
    /// @note This quantity's dimension is: inverse mass (M^-1).
    /// @sa Mass.
    using InvMass = PLAYRHO_QUANTITY(boost::units::si::inverse_mass);
    
    /// @brief Area quantity.
    /// @details This is the type alias for the area quantity. It's a derived quantity.
    /// @note This quantity's dimension is: length squared (L^2).
    /// @note The SI unit of area is the square-meter.
    /// @sa Length.
    /// @sa SquareMeter.
    /// @sa https://en.wikipedia.org/wiki/Area
    using Area = PLAYRHO_QUANTITY(boost::units::si::area);
    
    /// @brief Aereal/surface density quantity.
    /// @details This is the type alias for the area density quantity. It's a derived quantity.
    /// @note This quantity's dimensions are: mass per area (M L^-2).
    /// @note The SI derived unit of area density is kilogram per meter-squared.
    /// @sa Mass.
    /// @sa Area.
    /// @sa KilogramPerSquareMeter.
    /// @sa https://en.wikipedia.org/wiki/Area_density
    using Density = PLAYRHO_QUANTITY(boost::units::si::surface_density);
    
    /// @brief Angle quantity.
    /// @details This is the type alias for the plane angle base quantity.
    /// @note This quantity's dimension is: plane angle (QP).
    /// @sa Radian.
    /// @sa Degree.
    using Angle = PLAYRHO_QUANTITY(boost::units::si::plane_angle);
    
    /// @brief Angular velocity quantity.
    /// @details This is the type alias for the plane angular velocity quantity. It's a
    ///   derived quantity.
    /// @note This quantity's dimensions are: plane angle per time (QP T^-1).
    /// @note The SI derived unit of angular velocity is the radian per second.
    /// @sa Angle.
    /// @sa Time.
    /// @sa RadianPerSecond.
    /// @sa https://en.wikipedia.org/wiki/Angular_velocity
    using AngularVelocity = PLAYRHO_QUANTITY(boost::units::si::angular_velocity);
    
    /// @brief Angular acceleration quantity.
    /// @details This is the type alias for the angular acceleration quantity. It's a
    ///   derived quantity.
    /// @note This quantity's dimensions are: plane angle per time squared (QP T^-2).
    /// @note The SI derived unit of angular acceleration is the radian per second-squared.
    /// @sa Angle.
    /// @sa Time.
    /// @sa AngularVelocity.
    /// @sa RadianPerSquareSecond.
    /// @sa https://en.wikipedia.org/wiki/Angular_acceleration
    using AngularAcceleration = PLAYRHO_QUANTITY(boost::units::si::angular_acceleration);

    /// @brief Force quantity.
    /// @details This is the type alias for the force quantity. It's a derived quantity.
    /// @note This quantity's dimensions are: length mass per time squared (L M T^-2).
    /// @note The SI derived unit of force is the newton.
    /// @sa Length.
    /// @sa Mass.
    /// @sa Time.
    /// @sa Newton.
    /// @sa https://en.wikipedia.org/wiki/Force
    using Force = PLAYRHO_QUANTITY(boost::units::si::force);
    
    /// @brief Torque quantity.
    /// @details This is the type alias for the torque quantity. It's a derived quantity
    ///   that's a rotational force.
    /// @note This quantity's dimensions are: length-squared mass per time-squared per
    ///   angle (L^2 M T^-2 QP^-1).
    /// @note The SI derived unit of torque is the newton meter.
    /// @sa Length.
    /// @sa Mass.
    /// @sa Time.
    /// @sa Angle.
    /// @sa NewtonMeter.
    /// @sa https://en.wikipedia.org/wiki/Torque
    using Torque = PLAYRHO_QUANTITY(boost::units::si::torque);
    
    /// @brief Second moment of area quantity.
    /// @details This is the type alias for the second moment of area quantity. It's a
    ///   derived quantity.
    /// @note This quantity's dimensions are: length-squared-squared (L^4).
    /// @sa Length.
    /// @sa https://en.wikipedia.org/wiki/Second_moment_of_area
    using SecondMomentOfArea = PLAYRHO_QUANTITY(boost::units::si::second_moment_of_area);
    
    /// @brief Rotational inertia quantity.
    /// @details This is the type alias for the rotational intertia quantity. It's a
    ///   derived quantity that's also called the moment of inertia or angular mass.
    /// @note This quantity's dimensions are: length-squared mass per angle-squared (L^2 M QP^-2).
    /// @note The SI derived unit of rotational inertia is the kilogram per meter-squared.
    /// @sa Length.
    /// @sa Mass.
    /// @sa Angle.
    /// @sa InvRotInertia.
    /// @sa https://en.wikipedia.org/wiki/Moment_of_inertia
    using RotInertia = PLAYRHO_QUANTITY(boost::units::si::moment_of_inertia);
    
    /// @brief Inverse rotational inertia quantity.
    /// @details This is the type alias for the inverse rotational inertia quantity. It's
    ///   a derived quantity.
    /// @note This quantity's dimensions are: angle-squared per length-squared per mass
    ///    (L^-2 M^-1 QP^2).
    /// @sa Length.
    /// @sa Mass.
    /// @sa Angle.
    /// @sa RotInertia.
    using InvRotInertia = PLAYRHO_QUANTITY(boost::units::si::inverse_moment_of_inertia);
    
    /// @brief Momentum quantity.
    /// @details This is the type alias for the momentum quantity. It's a derived quantity.
    /// @note This quantity's dimensions are: length mass per time (L M T^-1).
    /// @note The SI derived unit of momentum is the kilogram meter per second.
    /// @note If <code>p</code> is momentum, <code>m</code> is mass, and <code>v</code> is
    ///   velocity, then <code>p = m * v</code>.
    /// @sa Length.
    /// @sa Mass.
    /// @sa Time.
    /// @sa NewtonSecond.
    /// @sa https://en.wikipedia.org/wiki/Momentum
    using Momentum = PLAYRHO_QUANTITY(boost::units::si::momentum);
    
    /// @brief Angular momentum quantity.
    /// @details This is the type alias for the angular momentum quantity. It's a derived
    ///   quantity.
    /// @note This quantity's dimensions are: length-squared mass per time per angle
    ///    (L^2 M T^-1 QP^-1).
    /// @note The SI derived unit of angular momentum is the kilogram meter-squared per second.
    /// @sa Length.
    /// @sa Mass.
    /// @sa Time.
    /// @sa Angle.
    /// @sa Momentum.
    /// @sa https://en.wikipedia.org/wiki/Angular_momentum
    using AngularMomentum = PLAYRHO_QUANTITY(boost::units::si::angular_momentum);
    
    /// @}

    /// @defgroup Units Unit definitions for expressing quantities.
    /// @details These are the unit definitions for physical quantities like time and length.
    ///   Conceptually a given unit is only usable with the quantities that are made up of
    ///   the dimensions which the unit is associated with.
    /// @sa Quantities.
    /// @{

    /// @brief Second unit of Time.
    /// @sa Time.
    constexpr auto Second = PLAYRHO_UNIT(Time, boost::units::si::second);

    /// @brief Hertz unit of Frequency.
    /// @details Represents the hertz unit of Frequency (Hz).
    /// @sa Frequency.
    /// @sa https://en.wikipedia.org/wiki/Hertz
    constexpr auto Hertz = PLAYRHO_UNIT(Frequency, boost::units::si::hertz);

    /// @brief Meter unit of Length.
    /// @details A unit of the Length quantity.
    /// @sa Length.
    constexpr auto Meter = PLAYRHO_UNIT(Length, boost::units::si::meter);

    /// @brief Meter per second unit of LinearVelocity.
    /// @sa LinearVelocity.
    constexpr auto MeterPerSecond = PLAYRHO_UNIT(LinearVelocity,
        boost::units::si::meter_per_second);

    /// @brief Meter per square second unit of LinearAcceleration.
    /// @sa LinearAcceleration.
    constexpr auto MeterPerSquareSecond = PLAYRHO_UNIT(LinearAcceleration,
        boost::units::si::meter_per_second_squared);

    /// @brief Kilogram unit of Mass.
    /// @sa Mass.
    constexpr auto Kilogram = PLAYRHO_UNIT(Mass, boost::units::si::kilogram);

    /// @brief Square meter unit of Area.
    /// @sa Area.
    constexpr auto SquareMeter = PLAYRHO_UNIT(Area, boost::units::si::square_meter);

    /// @brief Kilogram per square meter unit of Density.
    /// @sa Density.
    constexpr auto KilogramPerSquareMeter = PLAYRHO_UNIT(Density,
        boost::units::si::kilogram_per_square_meter);

    /// @brief Radian unit of Angle.
    /// @sa Angle.
    /// @sa Degree.
    constexpr auto Radian = PLAYRHO_UNIT(Angle, boost::units::si::radian);
    
    /// @brief Degree unit of Angle quantity.
    /// @sa Angle.
    /// @sa Radian.
    constexpr auto Degree = Angle{Radian * Pi / Real{180}};
    
    /// @brief Square radian unit type.
    /// @sa Angle.
    /// @sa Radian.
    constexpr auto SquareRadian = Radian * Radian;

    /// @brief Radian per second unit of AngularVelocity.
    /// @sa AngularVelocity.
    constexpr auto RadianPerSecond = PLAYRHO_UNIT(AngularVelocity,
        boost::units::si::radian_per_second);
    
    /// @brief Degree per second unit of AngularVelocity.
    /// @sa AngularVelocity.
    constexpr auto DegreePerSecond = AngularVelocity{RadianPerSecond * Degree / Radian};

    /// @brief Radian per square second unit of AngularAcceleration.
    /// @sa AngularAcceleration.
    constexpr auto RadianPerSquareSecond = Radian / (Second * Second);

    /// @brief Newton unit of Force.
    /// @sa Force.
    constexpr auto Newton = PLAYRHO_UNIT(Force, boost::units::si::newton);

    /// @brief Newton meter unit of Torque.
    /// @sa Torque.
    constexpr auto NewtonMeter = PLAYRHO_UNIT(Torque, boost::units::si::newton_meter);

    /// @brief Newton second unit of Momentum.
    /// @sa Momentum.
    constexpr auto NewtonSecond = Newton * Second;
    
    /// @}

    /// @brief Strips the units off of the given value.
    constexpr inline Real StripUnit(const Real value)
    {
        return value;
    }
    
#if defined(USE_BOOST_UNITS)
    
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
    
#endif // defined(USE_BOOST_UNITS)

} // namespace playrho

#if defined(USE_BOOST_UNITS)
namespace boost {
namespace units {

// Define division and multiplication templated operators in boost::units namespace since
//   boost::units is the consistent namespace of operands for these and this aids with
//   argument dependent lookup (ADL).
//
// Note that while boost::units already defines division and multiplication operator support,
//   that's only for division or multiplication with the same type that the quantity is based
//   on. For example when Real is float, Length{0.0f} * 2.0f is already supported but
//   Length{0.0f} * 2 is not.

/// @brief Division operator.
///
/// @details Supports the division of a playrho::Real based boost::units::quantity
///   by any arithmetic type except playrho::Real.
/// @note This intentionally excludes the playrho::Real type since the playrho::Real
///   type is already supported and supporting it again in this template causes
///   ambiguous overload support.
///
template <class Dimension, typename X, typename = std::enable_if_t<
    std::is_arithmetic<X>::value && !std::is_same<X, playrho::Real>::value &&
    std::is_same<decltype(playrho::Real{} / X{}), playrho::Real>::value >
>
auto operator/ (quantity<Dimension, playrho::Real> lhs, X rhs)
{
    return lhs / playrho::Real(rhs);
}

template <class Dimension, typename X, typename = std::enable_if_t<
    std::is_arithmetic<X>::value && !std::is_same<X, playrho::Real>::value &&
    std::is_same<decltype(X{} / playrho::Real{}), playrho::Real>::value >
>
auto operator/ (X lhs, quantity<Dimension, playrho::Real> rhs)
{
    return playrho::Real(lhs) / rhs;
}

/// @brief Multiplication operator.
///
/// @details Supports the multiplication of a playrho::Real based boost::units::quantity
///   by any arithmetic type except playrho::Real.
/// @note This intentionally excludes the playrho::Real type since the playrho::Real
///   type is already supported and supporting it again in this template causes
///   ambiguous overload support.
///
template <class Dimension, typename X, typename = std::enable_if_t<
    std::is_arithmetic<X>::value && !std::is_same<X, playrho::Real>::value &&
    std::is_same<decltype(playrho::Real{} * X{}), playrho::Real>::value> >
auto operator* (quantity<Dimension, playrho::Real> lhs, X rhs)
{
    return lhs * playrho::Real(rhs);
}

/// @brief Multiplication operator.
///
/// @details Supports the multiplication of a playrho::Real based boost::units::quantity
///   by any arithmetic type except playrho::Real.
/// @note This intentionally excludes the playrho::Real type since the playrho::Real
///   type is already supported and supporting it again in this template causes
///   ambiguous overload support.
///
template <class Dimension, typename X, typename = std::enable_if_t<
    std::is_arithmetic<X>::value && !std::is_same<X, playrho::Real>::value &&
    std::is_same<decltype(playrho::Real{} * X{}), playrho::Real>::value> >
auto operator* (X lhs, quantity<Dimension, playrho::Real> rhs)
{
    return playrho::Real(lhs) * rhs;
}

} // namespace units
} // namespace boost
#endif // defined(USE_BOOST_UNITS)

#undef PLAYRHO_QUANTITY
#undef PLAYRHO_UNIT

#endif /* PLAYRHO_UNITS_HPP */
