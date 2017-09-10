/*
 * Author: Chris Campbell - www.iforce2d.net
 *
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef IFORCE2D_TOPDOWN_CAR_HPP
#define IFORCE2D_TOPDOWN_CAR_HPP

#include "../Framework/Test.hpp"
#include <vector>
#include <set>

namespace playrho {

using ControlStateType = unsigned int;

enum ControlState: ControlStateType
{
    TDC_LEFT     = 0x1,
    TDC_RIGHT    = 0x2,
    TDC_UP       = 0x4,
    TDC_DOWN     = 0x8
};


//types of fixture user data
enum fixtureUserDataType
{
    FUD_CAR_TIRE,
    FUD_GROUND_AREA
};

//a class to allow subclassing of different fixture user data
class FixtureUserData
{
private:
    fixtureUserDataType m_type;

protected:
    FixtureUserData(fixtureUserDataType type) : m_type{type} {}

public:
    virtual fixtureUserDataType getType() { return m_type; }
    virtual ~FixtureUserData() {}
};

//class to allow marking a fixture as a car tire
class CarTireFUD : public FixtureUserData
{
public:
    CarTireFUD() : FixtureUserData{FUD_CAR_TIRE} {}
};

//class to allow marking a fixture as a ground area
class GroundAreaFUD : public FixtureUserData
{
public:
    float frictionModifier;
    bool outOfCourse;
    
    GroundAreaFUD(float fm, bool ooc):
        FixtureUserData{FUD_GROUND_AREA},
        frictionModifier{fm},
        outOfCourse{ooc}
    {
    }
};


class TDTire
{
private:
    Body* m_body;
    std::set<GroundAreaFUD*> m_groundAreas;
    Force m_maxDriveForce = Force{0};
    LinearVelocity m_maxForwardSpeed = LinearVelocity{0};
    LinearVelocity m_maxBackwardSpeed = LinearVelocity{0};
    Momentum m_maxLateralImpulse = Momentum{0};
    Real m_currentTraction = 1;
    
public:
    
    TDTire(World* world, std::shared_ptr<PolygonShape> tireShape)
    {
        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        m_body = world->CreateBody(bodyDef);
        
        const auto fixture = m_body->CreateFixture(tireShape);
        fixture->SetUserData( new CarTireFUD() );
        
        m_body->SetUserData( this );
    }
    
    ~TDTire()
    {
        m_body->GetWorld()->Destroy(m_body);
    }
    
    void setCharacteristics(LinearVelocity maxForwardSpeed, LinearVelocity maxBackwardSpeed, Force maxDriveForce, Momentum maxLateralImpulse)
    {
        m_maxForwardSpeed = maxForwardSpeed;
        m_maxBackwardSpeed = maxBackwardSpeed;
        m_maxDriveForce = maxDriveForce;
        m_maxLateralImpulse = maxLateralImpulse;
    }
    
    void addGroundArea(GroundAreaFUD* ga) { m_groundAreas.insert(ga); updateTraction(); }
    void removeGroundArea(GroundAreaFUD* ga) { m_groundAreas.erase(ga); updateTraction(); }
    
    void updateTraction()
    {
        if ( m_groundAreas.empty() )
            m_currentTraction = 1;
        else
        {
            //find area with highest traction
            m_currentTraction = 0;
            auto it = m_groundAreas.begin();
            while (it != m_groundAreas.end())
            {
                const auto ga = *it;
                if ( ga->frictionModifier > m_currentTraction )
                    m_currentTraction = ga->frictionModifier;
                ++it;
            }
        }
    }
    
    Body* GetBody() const
    {
        return m_body;
    }
    
    LinearVelocity2D getLateralVelocity() const
    {
        const auto currentRightNormal = GetWorldVector(*m_body, UnitVec2::GetRight());
        const auto vel = GetLinearVelocity(*m_body);
        return Dot(currentRightNormal, vel) * currentRightNormal;
    }
    
    LinearVelocity2D getForwardVelocity() const
    {
        const auto currentForwardNormal = GetWorldVector(*m_body, UnitVec2::GetTop());
        const auto vel = GetLinearVelocity(*m_body);
        return Dot(currentForwardNormal, vel) * currentForwardNormal;
    }
    
    void updateFriction()
    {
        //lateral linear velocity
        auto impulse = Momentum2D{GetMass(*m_body) * -getLateralVelocity()};
        const auto length = GetLength(GetVec2(impulse)) * Kilogram * MeterPerSecond;
        if ( length > m_maxLateralImpulse )
            impulse *= m_maxLateralImpulse / length;
        ApplyLinearImpulse(*m_body, m_currentTraction * impulse, m_body->GetWorldCenter());
        
        //angular velocity
        const auto rotInertia = GetRotInertia(*m_body);
        constexpr auto Tenth = Real{1} / Real{10};
        ApplyAngularImpulse(*m_body, m_currentTraction * Tenth * rotInertia * -GetAngularVelocity(*m_body));
        
        //forward linear velocity
        const auto forwardVelocity = getForwardVelocity();
        auto currentForwardSpeed = LinearVelocity{0};
        const auto forwardDir = GetUnitVector(forwardVelocity, currentForwardSpeed, UnitVec2::GetZero());
        const auto dragForceMagnitude = Real{-2} * currentForwardSpeed;
        const auto newForce = Force2D{m_currentTraction * dragForceMagnitude * forwardDir * Kilogram / Second};
        SetForce(*m_body, newForce, m_body->GetWorldCenter());
    }
    
    void updateDrive(ControlStateType controlState)
    {
        //find desired speed
        auto desiredSpeed = LinearVelocity{0};
        switch ( controlState & (TDC_UP|TDC_DOWN) ) {
            case TDC_UP:   desiredSpeed = m_maxForwardSpeed;  break;
            case TDC_DOWN: desiredSpeed = m_maxBackwardSpeed; break;
            default: return;//do nothing
        }
        
        //find current speed in forward direction
        const auto currentForwardNormal = GetWorldVector(*m_body, UnitVec2::GetTop());
        const auto currentSpeed = Dot(getForwardVelocity(), currentForwardNormal);
        
        //apply necessary force
        auto forceMagnitude = Force{0};
        if (desiredSpeed > currentSpeed)
            forceMagnitude = m_maxDriveForce;
        else if (desiredSpeed < currentSpeed)
            forceMagnitude = -m_maxDriveForce;
        else
            return;
        
        const auto newForce = Force2D{m_currentTraction * forceMagnitude * currentForwardNormal};
        SetForce(*m_body, newForce, m_body->GetWorldCenter());
    }
    
    void updateTurn(ControlStateType controlState)
    {
        auto desiredTorque = Real{0} * NewtonMeter;
        switch (controlState & (TDC_LEFT|TDC_RIGHT))
        {
            case TDC_LEFT:  desiredTorque = Real{+15} * NewtonMeter; break;
            case TDC_RIGHT: desiredTorque = Real{-15} * NewtonMeter; break;
            default: ;//nothing
        }
        SetTorque(*m_body, desiredTorque);
    }
};


class TDCar
{
private:
    Body* m_body;
    std::vector<TDTire*> m_tires;
    RevoluteJoint *flJoint, *frJoint;

public:
    TDCar(World* world)
    {
        //create car body
        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        m_body = world->CreateBody(bodyDef);
        m_body->SetAngularDamping(Real(3) * Hertz);
        
        Length2D vertices[8];
        vertices[0] = Vec2(+1.5f,  +0.0f) * Meter;
        vertices[1] = Vec2(+3.0f,  +2.5f) * Meter;
        vertices[2] = Vec2(+2.8f,  +5.5f) * Meter;
        vertices[3] = Vec2(+1.0f, +10.0f) * Meter;
        vertices[4] = Vec2(-1.0f, +10.0f) * Meter;
        vertices[5] = Vec2(-2.8f,  +5.5f) * Meter;
        vertices[6] = Vec2(-3.0f,  +2.5f) * Meter;
        vertices[7] = Vec2(-1.5f,  +0.0f) * Meter;
        PolygonShape polygonShape;
        polygonShape.Set(Span<const Length2D>(vertices, 8));
        polygonShape.SetDensity(Real{0.1f} * KilogramPerSquareMeter);
        m_body->CreateFixture(std::make_shared<PolygonShape>(polygonShape));
        
        //prepare common joint parameters
        RevoluteJointDef jointDef;
        jointDef.bodyA = m_body;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = Angle{0};
        jointDef.upperAngle = Angle{0};
        jointDef.localAnchorB = Vec2{0, 0} * Meter; //center of tire
        
        const auto maxForwardSpeed = Real{250.0f} * MeterPerSecond;
        const auto maxBackwardSpeed = Real{-40.0f} * MeterPerSecond;
        const auto backTireMaxDriveForce = Real{950.0f} * Newton; // 300.0f;
        const auto frontTireMaxDriveForce = Real{400.0f} * Newton; // 500.0f;
        const auto backTireMaxLateralImpulse = Real{9.0f} * Kilogram * MeterPerSecond; // 8.5f;
        const auto frontTireMaxLateralImpulse = Real{9.0f} * Kilogram * MeterPerSecond; // 7.5f;

        PolygonShape tireShape;
        tireShape.SetAsBox(Real{0.5f} * Meter, Real{1.25f} * Meter);
        tireShape.SetDensity(Real{1} * KilogramPerSquareMeter);
        const auto sharedTireShape = std::make_shared<PolygonShape>(tireShape);

        TDTire* tire;

        //back left tire (starts at absolute 0, 0 but pulled into place by joint)
        tire = new TDTire{world, sharedTireShape};
        tire->setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire->GetBody();
        jointDef.localAnchorA = Vec2(-3, 0.75f) * Meter; // sets car relative location of tire
        world->CreateJoint(jointDef);
        m_tires.push_back(tire);
        
        //back right tire (starts at absolute 0, 0 but pulled into place by joint)
        tire = new TDTire{world, sharedTireShape};
        tire->setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire->GetBody();
        jointDef.localAnchorA = Vec2(+3, 0.75f) * Meter; // sets car relative location of tire
        world->CreateJoint(jointDef);
        m_tires.push_back(tire);
        
        //front left tire (starts at absolute 0, 0 but pulled into place by joint)
        tire = new TDTire{world, sharedTireShape};
        tire->setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire->GetBody();
        jointDef.localAnchorA = Vec2(-3, 8.5f) * Meter; // sets car relative location of tire
        flJoint = static_cast<RevoluteJoint*>(world->CreateJoint(jointDef));
        m_tires.push_back(tire);
        
        //front right tire (starts at absolute 0, 0 but pulled into place by joint)
        tire = new TDTire{world, sharedTireShape};
        tire->setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire->GetBody();
        jointDef.localAnchorA = Vec2(+3, 8.5f) * Meter; // sets car relative location of tire
        frJoint = static_cast<RevoluteJoint*>(world->CreateJoint(jointDef));
        m_tires.push_back(tire);
    }
    
    ~TDCar()
    {
        for (auto i = decltype(m_tires.size()){0}; i < m_tires.size(); i++)
            delete m_tires[i];
    }
    
    void update(ControlStateType controlState)
    {
        for (auto i = decltype(m_tires.size()){0}; i < m_tires.size(); i++)
        {
            m_tires[i]->updateFriction();
        }
        for (auto i = decltype(m_tires.size()){0}; i < m_tires.size(); i++)
        {
            m_tires[i]->updateDrive(controlState);
        }
        
        //control steering
        const auto lockAngle = Real{35.0f} * Degree;
        const auto turnSpeedPerSec = Real{160.0f} * Degree;//from lock to lock in 0.5 sec
        const auto turnPerTimeStep = turnSpeedPerSec / Real{60.0f};
        auto desiredAngle = Angle{0};
        switch ( controlState & (TDC_LEFT|TDC_RIGHT) ) {
            case TDC_LEFT:  desiredAngle = lockAngle;  break;
            case TDC_RIGHT: desiredAngle = -lockAngle; break;
            default: ;//nothing
        }
        const auto angleNow = GetJointAngle(*flJoint);
        const auto desiredAngleToTurn = desiredAngle - angleNow;
        const auto angleToTurn = Clamp(desiredAngleToTurn, -turnPerTimeStep, turnPerTimeStep);
        if (angleToTurn != Angle{0})
        {
            const auto newAngle = angleNow + angleToTurn;
            flJoint->SetLimits( newAngle, newAngle );
            frJoint->SetLimits( newAngle, newAngle );
        }
    }
};


class MyDestructionListener :  public DestructionListener
{
    void SayGoodbye(Fixture& fixture) override
    {
        const auto fud = static_cast<FixtureUserData*>(fixture.GetUserData());
        if ( fud )
            delete fud;
    }
    
    //(unused but must implement all pure virtual functions)
    void SayGoodbye(Joint&) override {}
};


class iforce2d_TopdownCar : public Test
{
public:
    iforce2d_TopdownCar()
    {
        m_world->SetGravity(Vec2{0, 0} * MeterPerSquareSecond);
        m_world->SetDestructionListener(&m_destructionListener);
        
        //set up ground areas
        {
            Fixture* groundAreaFixture;

            BodyDef bodyDef;
            m_groundBody = m_world->CreateBody(bodyDef);
            
            PolygonShape polygonShape;
            FixtureDef fixtureDef;
            fixtureDef.isSensor = true;
            
            SetAsBox(polygonShape, Real{9} * Meter, Real{7} * Meter, Vec2(-10,15) * Meter, Real{20.0f} * Degree );
            groundAreaFixture = m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), fixtureDef);
            groundAreaFixture->SetUserData( new GroundAreaFUD( 0.5f, false ) );
            
            SetAsBox(polygonShape, Real{9} * Meter, Real{5} * Meter, Vec2(5,20) * Meter, Real{-40.0f} * Degree );
            groundAreaFixture = m_groundBody->CreateFixture(std::make_shared<PolygonShape>(polygonShape), fixtureDef);
            groundAreaFixture->SetUserData( new GroundAreaFUD( 0.2f, false ) );
        }
        
        //m_tire = new TDTire(m_world);
        //m_tire->setCharacteristics(100, -20, 150);
        
        m_car = new TDCar{m_world};
        m_controlState = 0;
    }
    
    ~iforce2d_TopdownCar()
    {
        //delete m_tire;
        delete m_car;
    }
    
    void KeyboardDown(Key key) override
    {
        switch (key) {
            case Key_A: m_controlState |= TDC_LEFT; break;
            case Key_D: m_controlState |= TDC_RIGHT; break;
            case Key_W: m_controlState |= TDC_UP; break;
            case Key_S: m_controlState |= TDC_DOWN; break;
            default: break;
        }
    }
    
    void KeyboardUp(Key key) override
    {
        switch (key) {
            case Key_A: m_controlState &= ~TDC_LEFT; break;
            case Key_D: m_controlState &= ~TDC_RIGHT; break;
            case Key_W: m_controlState &= ~TDC_UP; break;
            case Key_S: m_controlState &= ~TDC_DOWN; break;
            default: break;
        }
    }
    
    void handleContact(Contact* contact, bool began)
    {
        const auto fA = contact->GetFixtureA();
        const auto fB = contact->GetFixtureB();
        const auto fudA = (FixtureUserData*)fA->GetUserData();
        const auto fudB = (FixtureUserData*)fB->GetUserData();
        
        if ( !fudA || !fudB )
            return;
        
        if ( fudA->getType() == FUD_CAR_TIRE || fudB->getType() == FUD_GROUND_AREA )
            tire_vs_groundArea(fA, fB, began);
        else if ( fudA->getType() == FUD_GROUND_AREA || fudB->getType() == FUD_CAR_TIRE )
            tire_vs_groundArea(fB, fA, began);
    }
    
    void BeginContact(Contact& contact) override { handleContact(&contact, true); }
    void EndContact(Contact& contact) override { handleContact(&contact, false); }
    
    void tire_vs_groundArea(Fixture* tireFixture, Fixture* groundAreaFixture, bool began)
    {
        const auto tire = (TDTire*)tireFixture->GetBody()->GetUserData();
        const auto gaFud = (GroundAreaFUD*)groundAreaFixture->GetUserData();
        if ( began )
            tire->addGroundArea( gaFud );
        else
            tire->removeGroundArea( gaFud );
    }
    
    void PreStep(const Settings&, Drawer&) override
    {
        /*m_tire->updateFriction();
         m_tire->updateDrive(m_controlState);
         m_tire->updateTurn(m_controlState);*/
        
        m_car->update(m_controlState);
    }
    
    void PostStep(const Settings&, Drawer& drawer) override
    {
        //show some useful info
        drawer.DrawString(5, m_textLine, "Press w/a/s/d to control the car");
        m_textLine += 15;
        
        //drawer.DrawString(5, m_textLine, "Tire traction: %.2f", m_tire->m_currentTraction);
        //m_textLine += 15;
    }
    
    ControlStateType m_controlState;
    MyDestructionListener m_destructionListener;
    Body* m_groundBody;
    //TDTire* m_tire;
    TDCar* m_car;
};

} // namespace playrho

#endif
