/*
 * Original work Copyright (c) Chris Campbell - www.iforce2d.net
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef IFORCE2D_TRAJECTORIES_HPP
#define IFORCE2D_TRAJECTORIES_HPP

namespace testbed {

/// @brief iforce2d's Trajectories demo.
/// @details This is a port of iforce2d's Trajectories demo to the PlayRho Testbed.
/// @sa http://www.iforce2d.net/b2dtut/projected-trajectory
class iforce2d_Trajectories : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.seeAlso = "https://www.iforce2d.net/b2dtut/projected-trajectory";
        conf.credits = "Originally written by Chris Campbell for Box2D. Ported to PlayRho by Louis Langholtz.";
        conf.description = "Rotate the circle on the left to change launch direction.";
        return conf;
    }
    
    iforce2d_Trajectories(): Test(GetTestConf()), m_groundBody{m_world.CreateBody()}
    {
        //add four walls to the ground body
        FixtureConf myFixtureConf;
        auto polygonShape = PolygonShapeConf{};
        polygonShape.SetAsBox(20_m, 1_m); //ground
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        polygonShape.SetAsBox(20_m, 1_m, Vec2(0, 40) * 1_m, 0_rad); //ceiling
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        polygonShape.SetAsBox(1_m, 20_m, Vec2(-20, 20) * 1_m, 0_rad); //left wall
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        polygonShape.SetAsBox(1_m, 20_m, Vec2(20, 20) * 1_m, 0_rad); //right wall
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        
        //small ledges for target practice
        polygonShape.UseFriction(Real(0.95f));
        polygonShape.SetAsBox(1.5_m, 0.25_m, Vec2(3, 35) * 1_m, 0_rad);
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        polygonShape.SetAsBox(1.5_m, 0.25_m, Vec2(13, 30) * 1_m, 0_rad);
        m_groundBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        
        //another ledge which we can move with the mouse
        BodyConf kinematicBody;
        kinematicBody.type = BodyType::Kinematic;
        kinematicBody.location = Length2{11_m, 22_m};
        m_targetBody = m_world.CreateBody(kinematicBody);
        const auto w = BallSize * 1_m;
        Length2 verts[3];
        verts[0] = Length2(  0_m, -2*w);
        verts[1] = Length2(  w,    0_m);
        verts[2] = Length2(  0_m,   -w);
        polygonShape.Set(Span<const Length2>(verts, 3));
        m_targetBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        verts[0] = Length2(  0_m, -2*w);
        verts[2] = Length2(  0_m,   -w);
        verts[1] = Length2( -w,    0_m);
        polygonShape.Set(Span<const Length2>(verts, 3));
        m_targetBody->CreateFixture(Shape(polygonShape), myFixtureConf);
        
        //create dynamic circle body
        BodyConf myBodyConf;
        myBodyConf.type = BodyType::Dynamic;
        myBodyConf.location = Vec2(-15, 5) * 1_m;
        m_launcherBody = m_world.CreateBody(myBodyConf);
        m_launcherBody->CreateFixture(DiskShapeConf{}.UseRadius(2_m).UseFriction(Real(0.95f)).UseDensity(1_kgpm2),
                                      myFixtureConf);
        
        //pin the circle in place
        RevoluteJointConf revoluteJointConf;
        revoluteJointConf.bodyA = m_groundBody;
        revoluteJointConf.bodyB = m_launcherBody;
        revoluteJointConf.localAnchorA = Length2(-15_m, 5_m);
        revoluteJointConf.localAnchorB = Length2(0_m, 0_m);
        revoluteJointConf.enableMotor = true;
        revoluteJointConf.maxMotorTorque = 250_Nm;
        revoluteJointConf.motorSpeed = 0;
        m_world.CreateJoint( revoluteJointConf );
        
        //create dynamic box body to fire
        myBodyConf.location = Length2(0_m, -5_m);//will be positioned later
        m_littleBox = m_world.CreateBody(myBodyConf);
        polygonShape.SetAsBox( 0.5_m, 0.5_m );
        polygonShape.UseDensity(1_kgpm2);
        m_littleBox->CreateFixture(Shape(polygonShape), myFixtureConf);
        
        //ball for computer 'player' to fire
        m_littleBox2 = m_world.CreateBody(myBodyConf);
        m_littleBox2->CreateFixture(DiskShapeConf{}.UseRadius(BallSize * 1_m)
                                    .UseFriction(Real(0.95f)).UseDensity(1_kgpm2),
                                    myFixtureConf);
        
        m_firing = false;
        m_littleBox->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
        m_launchSpeed = 10_mps;
        
        m_firing2 = false;
        m_littleBox2->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
        m_littleBox2->SetVelocity(Velocity{});

        SetMouseWorld(Vec2(11,22) * 1_m);//sometimes is not set
        
        RegisterForKey(GLFW_KEY_Q, GLFW_PRESS, 0, "Launch projectile.", [&](KeyActionMods) {
            const auto launchSpeed = LinearVelocity2(m_launchSpeed, 0_mps);
            m_littleBox->SetAwake();
            m_littleBox->SetAcceleration(Gravity, AngularAcceleration{});
            m_littleBox->SetVelocity(Velocity{});
            m_littleBox->SetTransform(GetWorldPoint(*m_launcherBody, Vec2(3,0) * 1_m),
                                      m_launcherBody->GetAngle());
            const auto rotVec = GetWorldVector(*m_launcherBody, UnitVec::GetRight());
            const auto speed = Rotate(launchSpeed, rotVec);
            SetLinearVelocity(*m_littleBox, speed);
            m_firing = true;
        });
        RegisterForKey(GLFW_KEY_W, GLFW_PRESS, 0, "Reset projectile.", [&](KeyActionMods) {
            m_littleBox->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
            m_littleBox->SetVelocity(Velocity{});
            m_firing = false;
        });
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Increase launch speed.", [&](KeyActionMods) {
            m_launchSpeed *= 1.02f;
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Decrease launch speed.", [&](KeyActionMods) {
            m_launchSpeed *= 0.98f;
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Launch computer controlled projectile.", [&](KeyActionMods) {
            m_littleBox2->SetAwake();
            m_littleBox2->SetAcceleration(Gravity, AngularAcceleration{});
            m_littleBox2->SetVelocity(Velocity{});
            const auto launchVel = getComputerLaunchVelocity();
            const auto computerStartingPosition = Vec2(15,5) * 1_m;
            m_littleBox2->SetTransform(computerStartingPosition, 0_rad);
            SetLinearVelocity(*m_littleBox2, launchVel );
            m_firing2 = true;
        });
        RegisterForKey(GLFW_KEY_F, GLFW_PRESS, 0, "Reset computer controlled projectile.", [&](KeyActionMods) {
            m_littleBox2->SetAcceleration(LinearAcceleration2{}, AngularAcceleration{});
            m_littleBox2->SetVelocity(Velocity{});
            m_firing2 = false;
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Hold down & use left mouse button to move the computer's target", [&](KeyActionMods) {
            m_targetBody->SetTransform(GetMouseWorld(), 0_rad);
        });
    }
    
    //this just returns the current top edge of the golf-tee thingy
    Length2 getComputerTargetPosition()
    {
        return GetLocation(*m_targetBody) + Vec2(0, BallSize + 0.01f ) * 1_m;
    }
    
    //basic trajectory 'point at timestep n' formula
    Length2 getTrajectoryPoint(Length2 startingPosition, LinearVelocity2 startingVelocity,
                                float n /*time steps*/ )
    {
        const auto t = 1_s / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world.GetGravity(); // m/s/s
        
        return startingPosition + n * stepVelocity + 0.5f * (n*n+n) * stepGravity;
    }
    
    //find out how many timesteps it will take for projectile to reach maximum height
    float getTimestepsToTop(LinearVelocity2 startingVelocity)
    {
        const auto t = 1_s / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world.GetGravity(); // m/s/s
        return -float(Real(GetY(stepVelocity) / GetY(stepGravity))) - 1;
    }
    
    //find out the maximum height for this parabola
    Length getMaxHeight(Length2 startingPosition, LinearVelocity2 startingVelocity)
    {
        if ( GetY(startingVelocity) < 0_mps)
            return GetY(startingPosition);
        
        const auto t = 1_s / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world.GetGravity(); // m/s/s
        
        const auto n = -GetY(stepVelocity) / GetY(stepGravity) - 1;
        
        return GetY(startingPosition) + n * GetY(stepVelocity) + 0.5f * (n*n+n) * GetY(stepGravity);
    }
    
    //find the initial velocity necessary to reach a specified maximum height
    LinearVelocity calculateVerticalVelocityForHeight(Length desiredHeight)
    {
        if ( desiredHeight <= 0_m)
            return 0_mps;
        
        const auto t = 1_s / 60.0f;
        const auto stepGravity = t * t * m_world.GetGravity(); // m/s/s
        
        //quadratic equation setup
        const auto a = 0.5f / GetY(stepGravity);
        const auto b = 0.5f;
        const auto c = desiredHeight;
        
        const auto quadraticSolution1 = ( -b - sqrt( b*b - 4*a*c ) ) / (2*a);
        const auto quadraticSolution2 = ( -b + sqrt( b*b - 4*a*c ) ) / (2*a);
        
        auto v = quadraticSolution1;
        if ( v < decltype(v){} )
            v = quadraticSolution2;
        
        return 60 * v / 1_s;
    }
    
    //calculate how the computer should launch the ball with the current target location
    LinearVelocity2 getComputerLaunchVelocity()
    {
        const auto targetLocation = getComputerTargetPosition();
        const auto verticalVelocity = calculateVerticalVelocityForHeight(GetY(targetLocation) - 5_m);//computer projectile starts at y = 5
        const auto startingVelocity = LinearVelocity2(0,verticalVelocity);//only interested in vertical here
        const auto timestepsToTop = getTimestepsToTop(startingVelocity);
        auto targetEdgePos = GetX(GetLocation(*m_targetBody));
        if ( targetEdgePos > 15_m )
            targetEdgePos -= BallSize * 1_m;
        else
            targetEdgePos += BallSize * 1_m;
        const auto distanceToTargetEdge = targetEdgePos - 15_m;
        const auto horizontalVelocity = 60 * distanceToTargetEdge / timestepsToTop / 1_s;
        return LinearVelocity2(horizontalVelocity, verticalVelocity);
    }
    
    void PreStep(const Settings&, Drawer& drawer) override
    {
        const auto startingPosition = GetWorldPoint(*m_launcherBody, Vec2(3,0) * 1_m);
        const auto rotVec = GetWorldVector(*m_launcherBody, UnitVec::GetRight());
        const auto startingVelocity = Rotate(LinearVelocity2(m_launchSpeed, 0_mps), rotVec);
        
        if ( !m_firing )
            m_littleBox->SetTransform( startingPosition, m_launcherBody->GetAngle() );

        auto hit = false;
        auto point = Length2{};

        //draw predicted trajectory
        auto lastTP = startingPosition;
        for (auto i = 0; i < 300; ++i) { //5 seconds, should be long enough to hit something
            const auto trajectoryPosition = getTrajectoryPoint(startingPosition, startingVelocity, i * 1.0f);
            
            if (i > 0) {
                m_world.RayCast(lastTP, trajectoryPosition, [&](Fixture* f, ChildCounter,
                                                                 Length2 p, UnitVec) {
                    if (f->GetBody() == m_littleBox)
                    {
                        return World::RayCastOpcode::IgnoreFixture;
                    }
                    hit = true;
                    point = p;
                    return World::RayCastOpcode::Terminate;
                });
                if (hit) {
                    if (i % 2 == 0)
                    {
                        drawer.DrawSegment(trajectoryPosition, point, Color(1, 1, 0));
                    }
                    break;
                }
            }
            
            if (i % 2 == 0)
            {
                drawer.DrawSegment(lastTP, trajectoryPosition, Color(1, 1, 0));
            }
            lastTP = trajectoryPosition;
        }
        
        if (hit) {
            //draw raycast intersect location
            drawer.DrawPoint(point, 5, Color(0, 1, 1));
        }
        
        //draw dot in center of fired box
        const auto littleBoxPos = GetLocation(*m_littleBox);
        drawer.DrawPoint(littleBoxPos, 5, Color(0, 1, 0));
        
        //draw maximum height line
        const auto maxHeight = getMaxHeight( startingPosition, startingVelocity );
        drawer.DrawSegment(Length2(-20_m, maxHeight), Length2(+20_m, maxHeight),
                           Color(1, 1, 1, 0.5f));
        
        //draw line to indicate velocity computer player will fire at
        const auto launchVel = getComputerLaunchVelocity();
        const auto computerStartingPosition = Vec2(15,5) * 1_m;
        const auto displayVelEndPoint = computerStartingPosition + launchVel * 0.1_s;
        drawer.DrawSegment(computerStartingPosition, Color(1, 0, 0),
                           displayVelEndPoint, Color(0, 1, 0));
        
        if (!m_firing2)
            m_littleBox2->SetTransform(computerStartingPosition, 0_rad);
    }
    
    static Test* Create()
    {
        return new iforce2d_Trajectories;
    }
    
    Body* m_groundBody;
    Body* m_launcherBody;
    Body* m_littleBox;
    Body* m_littleBox2;
    Body* m_targetBody;
    bool m_firing;
    bool m_firing2;
    LinearVelocity m_launchSpeed;
    const LinearAcceleration2 Gravity{0 * MeterPerSquareSecond, -10 * MeterPerSquareSecond};
    const Real BallSize = 0.25f;
};

} // namespace testbed

#endif // IFORCE2D_TRAJECTORIES_HPP
