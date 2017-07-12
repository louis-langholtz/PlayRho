/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef B2_MASS_DATA_HPP
#define B2_MASS_DATA_HPP

/// @file
/// Declaration of the MassData structure and associated free functions.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {
    
    class Fixture;
    class Body;
    class Shape;
    class PolygonShape;
    class EdgeShape;
    class DiskShape;
    class ChainShape;

    /// Mass data.
    /// @details This holds the mass data computed for a shape.
    /// @note This data structure is 16-bytes large (on at least one 64-bit platform).
    struct MassData
    {
        MassData() = default;
        
        /// @brief Initializing constructor.
        /// @param m Non-negative mass in kg.
        /// @param c Position of the shape's centroid relative to the shape's origin.
        /// @param i Non-negative rotational inertia of the shape about the local origin.
        constexpr MassData(NonNegative<Mass> m, Length2D c, NonNegative<RotInertia> i) noexcept:
            mass{m}, center{c}, I{i}
        {
            // Intentionally empty.
        }
                
        /// @brief Position of the shape's centroid relative to the shape's origin.
        Length2D center = Length2D(0, 0);
        
        /// @brief Mass of the shape in kilograms.
        NonNegative<Mass> mass = Mass{0};

        /// @brief Rotational inertia, a.k.a. moment of inertia.
        /// @details This is the rotational inertia of the shape about the local origin.
        /// @sa https://en.wikipedia.org/wiki/Moment_of_inertia
        NonNegative<RotInertia> I = RotInertia{0};
    };
    
    Area GetAreaOfCircle(Length radius);

    Area GetAreaOfPolygon(Span<const Length2D> vertices);
    
    /// Gets the polar moment of the area enclosed by the given vertices.
    ///
    /// @warning Behavior is undefined if given collection has less than 3 vertices.
    ///
    /// @param vertices Collection of three or more vertices.
    ///
    SecondMomentOfArea GetPolarMoment(Span<const Length2D> vertices);

    /// Computes the mass data for a circular shape.
    ///
    /// @param r Radius of the circlular shape.
    /// @param density Areal density of mass.
    /// @param location Location of the center of the shape.
    ///
    MassData GetMassData(const Length r, const NonNegative<Density> density,
                         const Length2D location);

    /// Computes the mass data for a linear shape.
    ///
    /// @param r Radius of the vertices of the linear shape.
    /// @param density Areal density of mass.
    /// @param v0 Location of vertex zero.
    /// @param v1 Location of vertex one.
    ///
    MassData GetMassData(const Length r, const NonNegative<Density> density,
                         const Length2D v0, const Length2D v1);

    /// Computes the mass data for the given fixture.
    ///
    /// @details
    /// The mass data is based on the density and the shape of the fixture.
    /// The rotational inertia is about the shape's origin.
    /// @note This operation may be expensive.
    ///
    /// @param f Fixture to compute the mass data for.
    ///
    MassData GetMassData(const Fixture& f);
    
    /// Computes the body's mass data.
    /// @details This basically accumulates the mass data over all fixtures.
    /// @note The center is the mass weighted sum of all fixture centers. Divide it by the
    ///   mass to get the averaged center.
    /// @return accumalated mass data for all fixtures associated with the given body.
    MassData ComputeMassData(const Body& body) noexcept;
    
    
    /// Gets the mass data of the body.
    /// @return a struct containing the mass, inertia and center of the body.
    MassData GetMassData(const Body& body) noexcept;
    
}

#endif /* B2_MASS_DATA_HPP */
