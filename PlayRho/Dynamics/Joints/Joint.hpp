/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP

#include <PlayRho/Common/Math.hpp>

#include <PlayRho/Dynamics/Joints/JointType.hpp>
#include <PlayRho/Dynamics/Joints/LimitState.hpp>
#include <PlayRho/Dynamics/BodyID.hpp>

#include <memory> // for std::unique_ptr
#include <vector>
#include <utility>
#include <stdexcept> // for std::bad_cast

namespace playrho {

class StepConf;
struct ConstraintSolverConf;

namespace d2 {

class Joint;
class BodyConstraint;

/// @brief Gets the identifier of the type of data this can be casted to.
JointType GetType(const Joint& object) noexcept;

/// @brief Gets the first body attached to this joint.
BodyID GetBodyA(const Joint& object) noexcept;

/// @brief Gets the second body attached to this joint.
BodyID GetBodyB(const Joint& object) noexcept;

/// Get the anchor point on body-A in local coordinates.
Length2 GetLocalAnchorA(const Joint& object) noexcept;

/// Get the anchor point on body-B in local coordinates.
Length2 GetLocalAnchorB(const Joint& object) noexcept;

/// Get the linear reaction on body-B at the joint anchor.
Momentum2 GetLinearReaction(const Joint& object) noexcept;

/// Get the angular reaction on body-B.
AngularMomentum GetAngularReaction(const Joint& object) noexcept;

/// @brief Gets collide connected.
/// @note Modifying the collide connect flag won't work correctly because
///   the flag is only checked when fixture AABBs begin to overlap.
bool GetCollideConnected(const Joint& object) noexcept;

/// @brief Shifts the origin for any points stored in world coordinates.
/// @return <code>true</code> if shift done, <code>false</code> otherwise.
bool ShiftOrigin(Joint& object, Length2 value) noexcept;

/// @brief Whether this joint is in the is-in-island state.
bool IsIslanded(const Joint& object) noexcept;

/// @brief Sets this joint to be in the is-in-island state.
void SetIslanded(Joint& object) noexcept;

/// @brief Unsets this joint from being in the is-in-island state.
void UnsetIslanded(Joint& object) noexcept;

/// Gets the user data pointer.
void* GetUserData(const Joint& object) noexcept;

/// Sets the user data pointer.
void SetUserData(Joint& object, void* data) noexcept;

/// @brief Initializes velocity constraint data based on the given solver data.
/// @note This MUST be called prior to calling <code>SolveVelocity</code>.
/// @see SolveVelocity.
void InitVelocity(Joint& object, std::vector<BodyConstraint>& bodies,
                  const StepConf& step,
                  const ConstraintSolverConf& conf);

/// @brief Solves velocity constraint.
/// @pre <code>InitVelocity</code> has been called.
/// @see InitVelocity.
/// @return <code>true</code> if velocity is "solved", <code>false</code> otherwise.
bool SolveVelocity(Joint& object, std::vector<BodyConstraint>& bodies,
                   const StepConf& step);

/// @brief Solves the position constraint.
/// @return <code>true</code> if the position errors are within tolerance.
bool SolvePosition(const Joint& object, std::vector<BodyConstraint>& bodies,
                   const ConstraintSolverConf& conf);

/// @defgroup JointsGroup Joint Classes
/// @brief The user creatable classes that specify constraints on one or more body instances.
/// @ingroup ConstraintsGroup

/// @brief Joint class.
/// @details Joints are constraints that are used to constrain one or more bodies in various
///   fashions. Some joints also feature limits and motors.
/// @ingroup JointsGroup
/// @ingroup PhysicalEntities
class Joint
{
public:
    using BodyConstraintsMap = std::vector<BodyConstraint>;

    Joint() noexcept = default;

    /// @brief Initializing constructor.
    template <typename T>
    Joint(T arg): m_self{std::make_unique<Model<T>>(std::move(arg))}
    {
        // Intentionally empty.
    }

    /// @brief Copy constructor.
    Joint(const Joint& other): m_self{other.m_self? other.m_self->Clone_(): nullptr}
    {
        // Intentionally empty.
    }

    /// @brief Move constructor.
    Joint(Joint&& other) noexcept: m_self{std::move(other.m_self)}
    {
        // Intentionally empty.
    }

    /// @brief Copy assignment.
    Joint& operator= (const Joint& other)
    {
        m_self = other.m_self? other.m_self->Clone_(): nullptr;
        return *this;
    }

    /// @brief Move assignment.
    Joint& operator= (Joint&& other) noexcept
    {
        m_self = std::move(other.m_self);
        return *this;
    }

    /// @brief Move assignment support for any valid underlying configuration.
    template <typename T, typename Tp = std::decay_t<T>,
        typename = std::enable_if_t<
            !std::is_same<Tp, Joint>::value && std::is_copy_constructible<Tp>::value
        >
    >
    Joint& operator= (T&& other) noexcept
    {
        Joint(std::forward<T>(other)).swap(*this);
        return *this;
    }

    void swap(Joint& other) noexcept
    {
        std::swap(m_self, other.m_self);
    }

    friend JointType GetType(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetType_(): GetTypeID<void>();
    }

    template <typename T>
    friend auto TypeCast(const Joint* value) noexcept
    {
        if (!value || (GetType(*value) != GetTypeID<std::remove_pointer_t<T>>())) {
            return static_cast<T>(nullptr);
        }
        return static_cast<T>(value->m_self->GetData_());
    }

    friend BodyID GetBodyA(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetBodyA_(): InvalidBodyID;
    }

    friend BodyID GetBodyB(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetBodyB_(): InvalidBodyID;
    }

    friend Length2 GetLocalAnchorA(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetLocalAnchorA_(): Length2{};
    }

    friend Length2 GetLocalAnchorB(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetLocalAnchorB_(): Length2{};
    }

    friend Momentum2 GetLinearReaction(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetLinearReaction_(): Momentum2{};
    }

    friend AngularMomentum GetAngularReaction(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetAngularReaction_(): AngularMomentum{};
    }

    friend void* GetUserData(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetUserData_(): nullptr;
    }

    friend void SetUserData(Joint& object, void* data) noexcept
    {
        if (object.m_self) object.m_self->SetUserData_(data);
    }

    friend bool GetCollideConnected(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->GetCollideConnected_(): false;
    }

    friend bool ShiftOrigin(Joint& object, Length2 value) noexcept
    {
        return object.m_self? object.m_self->ShiftOrigin_(value): false;
    }

    friend void InitVelocity(Joint& object, BodyConstraintsMap& bodies,
                             const playrho::StepConf& step,
                             const ConstraintSolverConf& conf)
    {
        if (object.m_self) object.m_self->InitVelocity_(bodies, step, conf);
    }

    friend bool SolveVelocity(Joint& object, BodyConstraintsMap& bodies,
                              const playrho::StepConf& step)
    {
        return object.m_self? object.m_self->SolveVelocity_(bodies, step): false;
    }

    friend bool SolvePosition(const Joint& object, BodyConstraintsMap& bodies,
                              const ConstraintSolverConf& conf)
    {
        return object.m_self? object.m_self->SolvePosition_(bodies, conf): false;
    }

    friend bool IsIslanded(const Joint& object) noexcept
    {
        return object.m_self? object.m_self->IsIslanded_(): false;
    }

    friend void SetIslanded(Joint& object) noexcept
    {
        if (object.m_self) object.m_self->SetIslanded_();
    }

    friend void UnsetIslanded(Joint& object) noexcept
    {
        if (object.m_self) object.m_self->UnsetIslanded_();
    }

private:
    /// @brief Internal configuration concept.
    /// @note Provides the interface for runtime value polymorphism.
    struct Concept
    {
        virtual ~Concept() = default;

        /// @brief Clones this concept and returns a pointer to a mutable copy.
        /// @note This may throw <code>std::bad_alloc</code> or any exception that's thrown
        ///   by the constructor for the model's underlying data type.
        /// @throws std::bad_alloc if there's a failure allocating storage.
        virtual std::unique_ptr<Concept> Clone_() const = 0;

        /// @brief Gets the use type information.
        /// @return Type info of the underlying value's type.
        virtual TypeID GetType_() const noexcept = 0;

        /// @brief Gets the data for the underlying configuration.
        virtual const void* GetData_() const noexcept = 0;

        virtual BodyID GetBodyA_() const noexcept = 0;
        virtual BodyID GetBodyB_() const noexcept = 0;

        virtual bool GetCollideConnected_() const noexcept = 0;

        virtual bool ShiftOrigin_(Length2 value) noexcept = 0;

        virtual bool IsIslanded_() const noexcept = 0;
        virtual void SetIslanded_() noexcept = 0;
        virtual void UnsetIslanded_() noexcept = 0;

        virtual Length2 GetLocalAnchorA_() const noexcept = 0;
        virtual Length2 GetLocalAnchorB_() const noexcept = 0;
        virtual Momentum2 GetLinearReaction_() const noexcept = 0;
        virtual AngularMomentum GetAngularReaction_() const noexcept = 0;

        virtual void* GetUserData_() const noexcept = 0;
        virtual void SetUserData_(void* value) noexcept = 0;

        virtual void InitVelocity_(BodyConstraintsMap& bodies,
                                   const playrho::StepConf& step,
                                   const ConstraintSolverConf& conf) = 0;
        virtual bool SolveVelocity_(BodyConstraintsMap& bodies,
                                    const playrho::StepConf& step) = 0;
        virtual bool SolvePosition_(BodyConstraintsMap& bodies,
                                    const ConstraintSolverConf& conf) const = 0;
    };

    /// @brief Internal model configuration concept.
    /// @note Provides the implementation for runtime value polymorphism.
    template <typename T>
    struct Model final: Concept
    {
        /// @brief Type alias for the type of the data held.
        using data_type = T;

        /// @brief Initializing constructor.
        Model(T arg): data{std::move(arg)} {}

        std::unique_ptr<Concept> Clone_() const override
        {
            return std::make_unique<Model>(data);
        }

        TypeID GetType_() const noexcept override
        {
            return GetTypeID<data_type>();
        }

        const void* GetData_() const noexcept override
        {
            // Note address of "data" not necessarily same as address of "this" since
            // base class is virtual.
            return &data;
        }

        BodyID GetBodyA_() const noexcept override
        {
            return GetBodyA(data);
        }

        BodyID GetBodyB_() const noexcept override
        {
            return GetBodyB(data);
        }

        bool GetCollideConnected_() const noexcept override
        {
            return GetCollideConnected(data);
        }

        bool ShiftOrigin_(Length2 value) noexcept override
        {
            return ShiftOrigin(data, value);
        }

        bool IsIslanded_() const noexcept override
        {
            return IsIslanded(data);
        }

        void SetIslanded_() noexcept override
        {
            SetIslanded(data);
        }

        void UnsetIslanded_() noexcept override
        {
            UnsetIslanded(data);
        }

        Length2 GetLocalAnchorA_() const noexcept override
        {
            return GetLocalAnchorA(data);
        }

        Length2 GetLocalAnchorB_() const noexcept override
        {
            return GetLocalAnchorB(data);
        }

        Momentum2 GetLinearReaction_() const noexcept override
        {
            return GetLinearReaction(data);
        }

        AngularMomentum GetAngularReaction_() const noexcept override
        {
            return GetAngularReaction(data);
        }

        void* GetUserData_() const noexcept override
        {
            return GetUserData(data);
        }

        void SetUserData_(void* value) noexcept override
        {
            SetUserData(data, value);
        }

        void InitVelocity_(BodyConstraintsMap& bodies,
                                   const playrho::StepConf& step,
                                   const ConstraintSolverConf& conf) override
        {
            InitVelocity(data, bodies, step, conf);
        }

        bool SolveVelocity_(BodyConstraintsMap& bodies,
                                    const playrho::StepConf& step) override
        {
            return SolveVelocity(data, bodies, step);
        }

        bool SolvePosition_(BodyConstraintsMap& bodies,
                                    const ConstraintSolverConf& conf) const override
        {
            return SolvePosition(data, bodies, conf);
        }

        data_type data; ///< Data.
    };

    std::unique_ptr<Concept> m_self; ///< Self pointer.
};

// Free functions...

/// @brief Provides referenced access to the identified element of the given container.
BodyConstraint& At(std::vector<BodyConstraint>& container, BodyID key);

/// @brief Increment motor speed.
/// @details Template function for incrementally changing the motor speed of a joint that has
///   the <code>SetMotorSpeed</code> and <code>GetMotorSpeed</code> methods.
template <class T>
inline void IncMotorSpeed(T& j, AngularVelocity delta)
{
    j.SetMotorSpeed(j.GetMotorSpeed() + delta);
}

/// @relatedalso Joint
template <typename T>
inline auto TypeCast(const Joint& value)
{
    auto tmp = TypeCast<std::add_pointer_t<std::add_const_t<T>>>(&value);
    if (tmp == nullptr)
        throw std::bad_cast();
    return *tmp;
}

/// @relatedalso Joint
Angle GetReferenceAngle(const Joint& object);

/// @relatedalso Joint
UnitVec GetLocalXAxisA(const Joint& object);

/// @relatedalso Joint
UnitVec GetLocalYAxisA(const Joint& object);

/// @relatedalso Joint
AngularVelocity GetMotorSpeed(const Joint& object);

/// @relatedalso Joint
void SetMotorSpeed(Joint& object, AngularVelocity value);

/// @relatedalso Joint
Force GetMaxForce(const Joint& object);

/// @relatedalso Joint
Torque GetMaxTorque(const Joint& object);

/// @relatedalso Joint
Force GetMaxMotorForce(const Joint& object);

/// @relatedalso Joint
void SetMaxMotorForce(Joint& object, Force value);

/// @relatedalso Joint
Torque GetMaxMotorTorque(const Joint& object);

/// @relatedalso Joint
void SetMaxMotorTorque(Joint& object, Torque value);

/// @relatedalso Joint
RotInertia GetAngularMass(const Joint& object);

/// @relatedalso Joint
Real GetRatio(const Joint& object);

/// @relatedalso Joint
Real GetDampingRatio(const Joint& object);

/// @brief Gets the frequency of the joint if it has this property.
/// @relatedalso Joint
Frequency GetFrequency(const Joint& object);

/// @brief Sets the frequency of the joint if it has this property.
/// @relatedalso Joint
void SetFrequency(Joint& object, Frequency value);

/// @brief Gets the angular motor impulse of the joint if it has this property.
/// @relatedalso Joint
AngularMomentum GetAngularMotorImpulse(const Joint& object);

/// @relatedalso Joint
Length2 GetTarget(const Joint& object);

/// @relatedalso Joint
void SetTarget(Joint& object, Length2 value);

/// Gets the lower linear joint limit.
/// @relatedalso Joint
Length GetLinearLowerLimit(const Joint& object);

/// Gets the upper linear joint limit.
/// @relatedalso Joint
Length GetLinearUpperLimit(const Joint& object);

/// Sets the joint limits.
/// @relatedalso Joint
void SetLinearLimits(Joint& object, Length lower, Length upper);

/// Gets the lower joint limit.
/// @relatedalso Joint
Angle GetAngularLowerLimit(const Joint& object);

/// Gets the upper joint limit.
/// @relatedalso Joint
Angle GetAngularUpperLimit(const Joint& object);

/// Sets the joint limits.
/// @relatedalso Joint
void SetAngularLimits(Joint& object, Angle lower, Angle upper);

/// @relatedalso Joint
bool IsLimitEnabled(const Joint& object);

/// @relatedalso Joint
void EnableLimit(Joint& object, bool value);

/// @relatedalso Joint
bool IsMotorEnabled(const Joint& object);

/// @relatedalso Joint
void EnableMotor(Joint& object, bool value);

/// @relatedalso Joint
Length2 GetLinearOffset(const Joint& object);

/// @relatedalso Joint
void SetLinearOffset(Joint& object, Length2 value);

/// @relatedalso Joint
Angle GetAngularOffset(const Joint& object);

/// @relatedalso Joint
void SetAngularOffset(Joint& object, Angle value);

/// @relatedalso Joint
LimitState GetLimitState(const Joint& object);

/// @relatedalso Joint
Length2 GetGroundAnchorA(const Joint& object);

/// @relatedalso Joint
Length2 GetGroundAnchorB(const Joint& object);

/// @relatedalso Joint
Momentum GetLinearMotorImpulse(const Joint& object);

/// @brief Gets the current motor torque for the given joint given the inverse time step.
/// @relatedalso Joint
inline Torque GetMotorTorque(const Joint& joint, Frequency inv_dt)
{
    return GetAngularMotorImpulse(joint) * inv_dt;
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP
