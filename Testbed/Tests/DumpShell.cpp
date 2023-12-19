/*
 * Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "../Framework/Test.hpp"
#include <playrho/DynamicMemory.hpp>

namespace testbed {

// This test holds worlds dumped using Dump(World&).
class DumpShell : public Test
{
public:
    static inline const auto registered = RegisterTest("Dump Shell", MakeUniqueTest<DumpShell>);

    DumpShell()
    {
        // Source code dump of Box scene: issue304-minimal-case.rube
        //
        //  Created by R.U.B.E 1.3.0
        //  Using Box version 2.3.0
        //  Wed April 3 2013 04:33:28
        //
        //  This code is originally intended for use in the Box testbed,
        //  but you can easily use it in other applications by providing
        //  a World for use as the 'GetWorld()' variable in the code below.

        LinearAcceleration2 g(Real{0.000000000000000e+00f} * MeterPerSquareSecond,
                              Real{-1.000000000000000e+01f} * MeterPerSquareSecond);
        SetGravity(g);
        BodyID* bodies = (BodyID*)Alloc(3 * sizeof(BodyID));
        JointID* joints = (JointID*)Alloc(0 * sizeof(JointID));
        {
            BodyConf bd;
            bd.type = BodyType(0);
            bd.UseLocation(Vec2(2.587699890136719e-02f, 5.515012264251709e+00f) * 1_m);
            bd.UseAngle(0_rad);
            bd.linearVelocity = Vec2(0.000000000000000e+00f, 0.000000000000000e+00f) * 1_mps;
            bd.angularVelocity = 0_rad / 1_s;
            bd.linearDamping = 0_Hz;
            bd.angularDamping = 0_Hz;
            bd.allowSleep = bool(4);
            bd.awake = bool(2);
            bd.fixedRotation = bool(0);
            bd.bullet = bool(0);
            bd.enabled = bool(32);
            bodies[0] = CreateBody(GetWorld(), bd);

            {
                auto shape = PolygonShapeConf{};
                Length2 vs[4];
                vs[0] = Vec2(7.733039855957031e-01f, -1.497260034084320e-01f) * 1_m;
                vs[1] = Vec2(-4.487270116806030e-01f, 1.138330027461052e-01f) * 1_m;
                vs[2] = Vec2(-1.880589962005615e+00f, -1.365900039672852e-01f) * 1_m;
                vs[3] = Vec2(3.972740173339844e-01f, -3.897832870483398e+00f) * 1_m;
                shape.Set(vs);
                shape.UseFriction(Real(2.000000029802322e-01f));
                shape.UseRestitution(Real(0.000000000000000e+00f));
                shape.UseDensity(Real{1.000000000000000e+00f} * 1_kgpm2);
                shape.UseIsSensor(false);
                shape.UseFilter(
                    Filter{Filter::bits_type(1), Filter::bits_type(65535), Filter::index_type(0)});
                Attach(GetWorld(), bodies[0], CreateShape(GetWorld(), shape));
            }
        }
        {
            BodyConf bd;
            bd.type = BodyType(2);
            bd.UseLocation(Vec2(-3.122138977050781e-02f, 7.535382270812988e+00f) * 1_m);
            bd.UseAngle(-1.313644275069237e-02_rad);
            bd.linearVelocity = Vec2(8.230687379837036e-01f, 7.775862514972687e-02f) * 1_mps;
            bd.angularVelocity = 3.705333173274994e-02_rad / 1_s;
            bd.linearDamping = 0_Hz;
            bd.angularDamping = 0_Hz;
            bd.allowSleep = bool(4);
            bd.awake = bool(2);
            bd.fixedRotation = bool(0);
            bd.bullet = bool(0);
            bd.enabled = bool(32);
            bodies[1] = CreateBody(GetWorld(), bd);

            {
                auto shape = PolygonShapeConf{};
                Length2 vs[6];
                vs[0] = Vec2(3.473900079727173e+00f, -2.009889930486679e-01f) * 1_m;
                vs[1] = Vec2(3.457079887390137e+00f, 3.694039955735207e-02f) * 1_m;
                vs[2] = Vec2(-3.116359949111938e+00f, 2.348500071093440e-03f) * 1_m;
                vs[3] = Vec2(-3.109960079193115e+00f, -3.581250011920929e-01f) * 1_m;
                vs[4] = Vec2(-2.590820074081421e+00f, -5.472509860992432e-01f) * 1_m;
                vs[5] = Vec2(2.819370031356812e+00f, -5.402340292930603e-01f) * 1_m;
                shape.Set(vs);
                shape.UseFriction(Real(5.000000000000000e-01f));
                shape.UseRestitution(Real(0.000000000000000e+00f));
                shape.UseDensity(Real{5.000000000000000e+00f} * 1_kgpm2);
                shape.UseIsSensor(false);
                shape.UseFilter(
                    Filter{Filter::bits_type(1), Filter::bits_type(65535), Filter::index_type(0)});
                Attach(GetWorld(), bodies[1], CreateShape(GetWorld(), shape));
            }
        }
        {
            BodyConf bd;
            bd.type = BodyType(2);
            bd.UseLocation(Vec2(-7.438077926635742e-01f, 6.626811981201172e+00f) * 1_m);
            bd.UseAngle(-1.884713363647461e+01_rad);
            bd.linearVelocity = Vec2(1.785794943571091e-01f, 3.799796104431152e-07f) * 1_mps;
            bd.angularVelocity = -5.908820639888290e-06_rad / 1_s;
            bd.linearDamping = 0_Hz;
            bd.angularDamping = 0_Hz;
            bd.allowSleep = bool(4);
            bd.awake = bool(2);
            bd.fixedRotation = bool(0);
            bd.bullet = bool(0);
            bd.enabled = bool(32);
            bodies[2] = CreateBody(GetWorld(), bd);
            {
                auto shape = PolygonShapeConf{};
                Length2 vs[4];
                vs[0] = Vec2(1.639146506786346e-01f, 4.428443685173988e-02f) * 1_m;
                vs[1] = Vec2(-1.639146655797958e-01f, 4.428443685173988e-02f) * 1_m;
                vs[2] = Vec2(-1.639146655797958e-01f, -4.428443312644958e-02f) * 1_m;
                vs[3] = Vec2(1.639146357774734e-01f, -4.428444057703018e-02f) * 1_m;
                shape.Set(vs);
                shape.UseFriction(Real(9.499999880790710e-01f));
                shape.UseRestitution(Real(0.000000000000000e+00f));
                shape.UseDensity(Real{1.000000000000000e+01f} * 1_kgpm2);
                shape.UseIsSensor(false);
                shape.UseFilter(
                    Filter{Filter::bits_type(1), Filter::bits_type(65535), Filter::index_type(-3)});
                Attach(GetWorld(), bodies[2], CreateShape(GetWorld(), shape));
            }
        }

        SetAccelerations(GetWorld(), GetGravity());
        Free(joints);
        Free(bodies);
        joints = nullptr;
        bodies = nullptr;
    }
};

} // namespace testbed
