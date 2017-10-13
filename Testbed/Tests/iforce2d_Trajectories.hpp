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

#if defined(__APPLE_CC__)
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

namespace playrho {

/// @brief iforce2d's Trajectories demo.
/// @details This is a port of iforce2d's Trajectories demo to the PlayRho Testbed.
/// @sa http://www.iforce2d.net/b2dtut/projected-trajectory
class iforce2d_Trajectories : public Test
{
public:
    iforce2d_Trajectories(): m_groundBody{m_world->CreateBody()}
    {
        //add four walls to the ground body
        FixtureDef myFixtureDef;
        PolygonShape polygonShape;
        polygonShape.SetAsBox(20 * Meter, 1 * Meter); //ground
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        SetAsBox(polygonShape, 20 * Meter,  1 * Meter, Vec2(0, 40) * Meter, 0 * Radian); //ceiling
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        SetAsBox(polygonShape,  1 * Meter, 20 * Meter, Vec2(-20, 20) * Meter, 0 * Radian); //left wall
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        SetAsBox(polygonShape,  1 * Meter, 20 * Meter, Vec2(20, 20) * Meter, 0 * Radian); //right wall
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        
        //small ledges for target practice
        polygonShape.SetFriction(0.95f);
        SetAsBox(polygonShape, 1.5f * Meter, 0.25f * Meter, Vec2(3, 35) * Meter, 0 * Radian);
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        SetAsBox(polygonShape, 1.5f * Meter, 0.25f * Meter, Vec2(13, 30) * Meter, 0 * Radian);
        m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        
        //another ledge which we can move with the mouse
        BodyDef kinematicBody;
        kinematicBody.type = BodyType::Kinematic;
        kinematicBody.location = Length2D{11 * Meter, 22 * Meter};
        m_targetBody = m_world->CreateBody(kinematicBody);
        const auto w = BallSize * Meter;
        Length2D verts[3];
        verts[0] = Length2D(  0 * Meter, -2*w);
        verts[1] = Length2D(  w,    0 * Meter);
        verts[2] = Length2D(  0 * Meter,   -w);
        polygonShape.Set(Span<const Length2D>(verts, 3));
        m_targetBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        verts[0] = Length2D(  0 * Meter, -2*w);
        verts[2] = Length2D(  0 * Meter,   -w);
        verts[1] = Length2D( -w,    0 * Meter);
        polygonShape.Set(Span<const Length2D>(verts, 3));
        m_targetBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        
        //create dynamic circle body
        BodyDef myBodyDef;
        myBodyDef.type = BodyType::Dynamic;
        myBodyDef.location = Vec2(-15, 5) * Meter;
        m_launcherBody = m_world->CreateBody(myBodyDef);
        DiskShape circleShape{DiskShape::Conf{}
            .UseVertexRadius(2 * Meter)
            .UseFriction(0.95f)
            .UseDensity(1 * KilogramPerSquareMeter)};
        m_launcherBody->CreateFixture(std::make_shared<DiskShape>(circleShape), myFixtureDef);
        
        //pin the circle in place
        RevoluteJointDef revoluteJointDef;
        revoluteJointDef.bodyA = m_groundBody;
        revoluteJointDef.bodyB = m_launcherBody;
        revoluteJointDef.localAnchorA = Length2D(-15 * Meter, 5 * Meter);
        revoluteJointDef.localAnchorB = Length2D(0 * Meter, 0 * Meter);
        revoluteJointDef.enableMotor = true;
        revoluteJointDef.maxMotorTorque = 250 * NewtonMeter;
        revoluteJointDef.motorSpeed = 0;
        m_world->CreateJoint( revoluteJointDef );
        
        //create dynamic box body to fire
        myBodyDef.location = Length2D(0 * Meter, -5 * Meter);//will be positioned later
        m_littleBox = m_world->CreateBody(myBodyDef);
        polygonShape.SetAsBox( 0.5f * Meter, 0.5f * Meter );
        polygonShape.SetDensity(1 * KilogramPerSquareMeter);
        m_littleBox->CreateFixture(std::make_shared<PolygonShape>(polygonShape), myFixtureDef);
        
        //ball for computer 'player' to fire
        m_littleBox2 = m_world->CreateBody(myBodyDef);
        circleShape.SetRadius(BallSize * Meter);
        m_littleBox2->CreateFixture(std::make_shared<DiskShape>(circleShape), myFixtureDef);
        
        m_firing = false;
        m_littleBox->SetAcceleration(LinearAcceleration2D{}, AngularAcceleration{});
        m_launchSpeed = 10 * MeterPerSecond;
        
        m_firing2 = false;
        m_littleBox2->SetAcceleration(LinearAcceleration2D{}, AngularAcceleration{});
        m_littleBox2->SetVelocity(Velocity{});

        SetMouseWorld(Vec2(11,22) * Meter);//sometimes is not set
    }
    
    //this just returns the current top edge of the golf-tee thingy
    Length2D getComputerTargetPosition()
    {
        return GetLocation(*m_targetBody) + Vec2(0, BallSize + 0.01f ) * Meter;
    }
    
    //basic trajectory 'point at timestep n' formula
    Length2D getTrajectoryPoint(Length2D startingPosition, LinearVelocity2D startingVelocity,
                                float n /*time steps*/ )
    {
        const auto t = Second / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world->GetGravity(); // m/s/s
        
        return startingPosition + n * stepVelocity + 0.5f * (n*n+n) * stepGravity;
    }
    
    //find out how many timesteps it will take for projectile to reach maximum height
    float getTimestepsToTop(LinearVelocity2D startingVelocity)
    {
        const auto t = Second / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world->GetGravity(); // m/s/s
        return -GetY(stepVelocity) / GetY(stepGravity) - 1;
    }
    
    //find out the maximum height for this parabola
    Length getMaxHeight(Length2D startingPosition, LinearVelocity2D startingVelocity)
    {
        if ( GetY(startingVelocity) < 0 * MeterPerSecond)
            return GetY(startingPosition);
        
        const auto t = Second / 60.0f;
        const auto stepVelocity = t * startingVelocity; // m/s
        const auto stepGravity = t * t * m_world->GetGravity(); // m/s/s
        
        const auto n = -GetY(stepVelocity) / GetY(stepGravity) - 1;
        
        return GetY(startingPosition) + n * GetY(stepVelocity) + 0.5f * (n*n+n) * GetY(stepGravity);
    }
    
    //find the initial velocity necessary to reach a specified maximum height
    LinearVelocity calculateVerticalVelocityForHeight(Length desiredHeight)
    {
        if ( desiredHeight <= 0 * Meter)
            return 0 * MeterPerSecond;
        
        const auto t = Second / 60.0f;
        const auto stepGravity = t * t * m_world->GetGravity(); // m/s/s
        
        //quadratic equation setup
        const auto a = 0.5f / GetY(stepGravity);
        const auto b = 0.5f;
        const auto c = desiredHeight;
        
        const auto quadraticSolution1 = ( -b - Sqrt( b*b - 4*a*c ) ) / (2*a);
        const auto quadraticSolution2 = ( -b + Sqrt( b*b - 4*a*c ) ) / (2*a);
        
        auto v = quadraticSolution1;
        if ( v < decltype(v){} )
            v = quadraticSolution2;
        
        return v * 60.0f / Second;
    }
    
    //calculate how the computer should launch the ball with the current target location
    LinearVelocity2D getComputerLaunchVelocity()
    {
        const auto targetLocation = getComputerTargetPosition();
        const auto verticalVelocity = calculateVerticalVelocityForHeight(GetY(targetLocation) - 5 * Meter);//computer projectile starts at y = 5
        const auto startingVelocity = LinearVelocity2D(0,verticalVelocity);//only interested in vertical here
        const auto timestepsToTop = getTimestepsToTop(startingVelocity);
        auto targetEdgePos = GetX(GetLocation(*m_targetBody));
        if ( targetEdgePos > 15 * Meter )
            targetEdgePos -= BallSize * Meter;
        else
            targetEdgePos += BallSize * Meter;
        const auto distanceToTargetEdge = targetEdgePos - 15 * Meter;
        const auto horizontalVelocity = 60 * distanceToTargetEdge / timestepsToTop / Second;
        return LinearVelocity2D(horizontalVelocity, verticalVelocity);
    }
    
    void KeyboardDown(Key key) override
    {
        switch (key) {
            case Key_Q:
            {
                const auto launchSpeed = LinearVelocity2D(m_launchSpeed, 0 * MeterPerSecond);
                m_littleBox->SetAwake();
                m_littleBox->SetAcceleration(Gravity, AngularAcceleration{});
                m_littleBox->SetVelocity(Velocity{});
                m_littleBox->SetTransform(GetWorldPoint(*m_launcherBody, Vec2(3,0) * Meter),
                                          m_launcherBody->GetAngle());
                const auto rotVec = GetWorldVector(*m_launcherBody, UnitVec2::GetRight());
                const auto speed = Rotate(launchSpeed, rotVec);
                SetLinearVelocity(*m_littleBox, speed);
                m_firing = true;
                break;
            }
            case Key_W:
                m_littleBox->SetAcceleration(LinearAcceleration2D{}, AngularAcceleration{});
                m_littleBox->SetVelocity(Velocity{});
                m_firing = false;
                break;
            case Key_A:
                m_launchSpeed *= 1.02f;
                break;
            case Key_S:
                m_launchSpeed *= 0.98f;
                break;
                
            case Key_D:
            {
                m_littleBox2->SetAwake();
                m_littleBox2->SetAcceleration(Gravity, AngularAcceleration{});
                m_littleBox2->SetVelocity(Velocity{});
                const auto launchVel = getComputerLaunchVelocity();
                const auto computerStartingPosition = Vec2(15,5) * Meter;
                m_littleBox2->SetTransform(computerStartingPosition, 0 * Radian);
                SetLinearVelocity(*m_littleBox2, launchVel );
                m_firing2 = true;
                break;
            }
            case Key_F:
                m_littleBox2->SetAcceleration(LinearAcceleration2D{}, AngularAcceleration{});
                m_littleBox2->SetVelocity(Velocity{});
                m_firing2 = false;
                break;
                
            case Key_M:
                m_targetBody->SetTransform(GetMouseWorld(), 0 * Radian); //m_mouseWorld is from Test class
                break;
                
            default:
                break;
        }
    }
    
    void PreStep(const Settings&, Drawer& drawer) override
    {
        const auto startingPosition = GetWorldPoint(*m_launcherBody, Vec2(3,0) * Meter);
        const auto rotVec = GetWorldVector(*m_launcherBody, UnitVec2::GetRight());
        const auto startingVelocity = Rotate(LinearVelocity2D(m_launchSpeed, 0 * MeterPerSecond), rotVec);
        
        if ( !m_firing )
            m_littleBox->SetTransform( startingPosition, m_launcherBody->GetAngle() );

        auto hit = false;
        auto point = Length2D{};

        //draw predicted trajectory
        auto lastTP = startingPosition;
        for (auto i = 0; i < 300; ++i) { //5 seconds, should be long enough to hit something
            const auto trajectoryPosition = getTrajectoryPoint(startingPosition, startingVelocity, i);
            
            if (i > 0) {
                m_world->RayCast(lastTP, trajectoryPosition, [&](Fixture* f, ChildCounter,
                                                                 Length2D p, UnitVec2) {
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
        drawer.DrawSegment(Length2D(-20 * Meter, maxHeight), Length2D(+20 * Meter, maxHeight),
                           Color(1, 1, 1, 0.5f));
        
        //draw line to indicate velocity computer player will fire at
        const auto launchVel = getComputerLaunchVelocity();
        const auto computerStartingPosition = Vec2(15,5) * Meter;
        const auto displayVelEndPoint = computerStartingPosition + 0.1f * launchVel * Second;
        drawer.DrawSegment(computerStartingPosition, Color(1, 0, 0),
                           displayVelEndPoint, Color(0, 1, 0));
        
        if (!m_firing2)
            m_littleBox2->SetTransform(computerStartingPosition, 0 * Radian);
    }
    
    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Rotate the circle on the left to change launch direction");
        m_textLine += 15;
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Use a/s to change the launch speed");
        m_textLine += 15;
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Use q/w to launch and reset the projectile");
        m_textLine += 15;
        m_textLine += 15;
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Use d/f to launch and reset the computer controlled projectile");
        m_textLine += 15;
        drawer.DrawString(5, m_textLine, Drawer::Left,
                          "Hold down m and use the left mouse button to move the computer's target");
        m_textLine += 15;
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
    const LinearAcceleration2D Gravity{0 * MeterPerSquareSecond, -10 * MeterPerSquareSecond};
    const Real BallSize = 0.25f;
};

} // namespace playrho

#endif // IFORCE2D_TRAJECTORIES_HPP
