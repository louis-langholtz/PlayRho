/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_DYNAMICS_WORLD_HPP
#define PLAYRHO_DYNAMICS_WORLD_HPP

/// @file
/// Declarations of the World class and associated free functions.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Range.hpp>
#include <PlayRho/Dynamics/WorldDef.hpp>
#include <PlayRho/Dynamics/BodyDef.hpp>
#include <PlayRho/Dynamics/BodyAtty.hpp>
#include <PlayRho/Dynamics/FixtureDef.hpp>
#include <PlayRho/Dynamics/WorldCallbacks.hpp>
#include <PlayRho/Dynamics/StepStats.hpp>
#include <PlayRho/Collision/DynamicTree.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>
#include <PlayRho/Dynamics/Contacts/ContactKey.hpp>
#include <PlayRho/Dynamics/ContactAtty.hpp>
#include <PlayRho/Dynamics/JointAtty.hpp>

#include <vector>
#include <map>
#include <unordered_set>
#include <memory>
#include <stdexcept>
#include <functional>

namespace playrho {

struct BodyDef;
struct JointDef;
struct FixtureDef;
class Body;
class Contact;
class Fixture;
class Joint;
struct Island;
class StepConf;
class Shape;
enum class BodyType;

/// @defgroup PhysicalEntities Physical Entity Classes
///
/// @brief Classes representing physical entities typically created/destroyed via factory methods.
///
/// @details Classes of creatable and destroyable managed instances that associate
///   physical properties to simulations. These instances are typically created via a
///   method whose name begins with the prefix of <code>Create</code>. Similarly, these
///   instances are typically destroyed using a method whose name begins with the prefix
///   of <code>Destroy</code>.
///
/// @sa World::CreateBody
/// @sa World::CreateJoint
/// @sa World::Destroy
/// @sa Body::CreateFixture
/// @sa Body::DestroyFixture
/// @sa Body::DestroyFixtures

/// @brief Definition of an independent and simulatable "world".
///
/// @details The world class manages physics entities, dynamic simulation, and queries.
///   In a physical sense, perhaps this is more like a universe in that entities in a
///   world have no interaction with entities in other worlds. In any case, there's
///   precedence, from a physics-engine standpoint, for this being called a world.
///
/// @note World instances are composed of &mdash; i.e. contain and own &mdash; Body, Joint,
///   and Contact instances.
/// @note This data structure is 352-bytes large (with 4-byte Real on at least one 64-bit
///   platform).
///
/// @sa Body, Joint, Contact
///
class World
{
public:
    
    /// @brief Proxy size type.
    using proxy_size_type = std::remove_const<decltype(MaxContacts)>::type;
    
    /// @brief Bodies container type.
    using Bodies = std::vector<Body*>;

    /// @brief Contacts container type.
    using Contacts = std::vector<KeyedContactPtr>;
    
    /// @brief Joints container type.
    /// @note Cannot be container of Joint instances since joints are polymorphic types.
    using Joints = std::vector<Joint*>;
    
    /// @brief Constructs a world object.
    /// @throws InvalidArgument if the given max vertex radius is less than the min.
    explicit World(const WorldDef& def = GetDefaultWorldDef());

    /// @brief Copy constructor.
    World(const World& other);

    /// @brief Assignment operator.
    World& operator= (const World& other);

    /// @brief Destructor.
    /// @details
    /// All physics entities are destroyed and all dynamically allocated memory is released.
    ~World();

    /// @brief Clears this world.
    void Clear() noexcept;

    /// Register a destruction listener. The listener is owned by you and must
    /// remain in scope.
    void SetDestructionListener(DestructionListener* listener) noexcept;

    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used. The listener is
    /// owned by you and must remain in scope. 
    void SetContactFilter(ContactFilter* filter) noexcept;

    /// Register a contact event listener. The listener is owned by you and must
    /// remain in scope.
    void SetContactListener(ContactListener* listener) noexcept;

    /// @brief Creates a rigid body given a definition.
    /// @note No reference to the definition is retained.
    /// @warning This function is locked during callbacks.
    /// @return Pointer to newly created body.
    /// @throws WrongState if this method is called while the world is locked.
    /// @throws LengthError if this operation would create more than MaxBodies.
    /// @sa PhysicalEntities
    Body* CreateBody(const BodyDef& def = GetDefaultBodyDef());

    /// @brief Destroys the given body.
    /// @note This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    /// @warning Behavior is undefined if given a null body.
    /// @warning Behavior is undefined if the passed body was not created by this world.
    /// @throws WrongState if this method is called while the world is locked.
    /// @sa PhysicalEntities
    void Destroy(Body* body);

    /// @brief Creates a joint to constrain bodies together.
    /// @details No reference to the definition is retained. This may cause the
    ///   connected bodies to cease colliding.
    /// @warning This function is locked during callbacks.
    /// @return Pointer to newly created joint.
    /// @throws WrongState if this method is called while the world is locked.
    /// @throws LengthError if this operation would create more than MaxJoints.
    /// @throws InvalidArgument if the given definition is not allowed.
    /// @sa PhysicalEntities
    Joint* CreateJoint(const JointDef& def);

    /// @brief Destroys a joint.
    /// @details This may cause the connected bodies to begin colliding.
    /// @warning This function is locked during callbacks.
    /// @warning Behavior is undefined if the passed joint was not created by this world.
    /// @param joint Joint, created by this world, to destroy.
    /// @throws WrongState if this method is called while the world is locked.
    /// @sa PhysicalEntities
    void Destroy(Joint* joint);

    /// @brief Steps the world simulation according to the given configuration.
    ///
    /// @details
    /// Performs position and velocity updating, sleeping of non-moving bodies, updating
    /// of the contacts, and notifying the contact listener of begin-contact, end-contact,
    /// pre-solve, and post-solve events.
    ///
    /// @warning Behavior is undefined if given a negative step time delta.
    /// @warning Varying the step time delta may lead to non-physical behaviors.
    ///
    /// @note Calling this with a zero step time delta results only in fixtures and bodies
    ///   registered for proxy handling being processed. No physics is performed.
    /// @note If the given velocity and position iterations are zero, this method doesn't
    ///   do velocity or position resolutions respectively of the contacting bodies.
    /// @note While body velocities are updated accordingly (per the sum of forces acting on them),
    ///   body positions (barring any collisions) are updated as if they had moved the entire time
    ///   step at those resulting velocities. In other words, a body initially at p0 going v0 fast
    ///   with a sum acceleration of a, after time t and barring any collisions, will have a new
    ///   velocity (v1) of v0 + (a * t) and a new position (p1) of p0 + v1 * t.
    ///
    /// @post Static bodies are unmoved.
    /// @post Kinetic bodies are moved based on their previous velocities.
    /// @post Dynamic bodies are moved based on their previous velocities, gravity, applied
    ///   forces, applied impulses, masses, damping, and the restitution and friction values
    ///   of their fixtures when they experience collisions.
    ///
    /// @param conf Configuration for the simulation step.
    ///
    /// @return Statistics for the step.
    ///
    /// @throws WrongState if this method is called while the world is locked.
    ///
    StepStats Step(const StepConf& conf);

    /// @brief Query AABB for fixtures callback function type.
    /// @note Returning true will continue the query. Returning false will terminate the query.
    using QueryFixtureCallback = std::function<bool(Fixture* fixture, ChildCounter child)>;

    /// @brief Queries the world for all fixtures that potentially overlap the provided AABB.
    /// @param aabb the query box.
    /// @param callback User implemented callback function.
    void QueryAABB(const AABB2D& aabb, QueryFixtureCallback callback) const;

    /// @brief Ray-cast operation code.
    ///
    /// @details Instructs the <code>RayCast</code> method on what to do next.
    ///
    enum class RayCastOpcode;

    /// @brief Ray cast callback function signature.
    using RayCastCallback = std::function<RayCastOpcode(Fixture* fixture,
                                                        ChildCounter child,
                                                        Length2 point,
                                                        UnitVec2 normal)>;

    /// @brief Ray-cast the world for all fixtures in the path of the ray.
    ///
    /// @note The callback controls whether you get the closest point, any point, or n-points.
    /// @note The ray-cast ignores shapes that contain the starting point.
    ///
    /// @param point1 Ray starting point.
    /// @param point2 Ray ending point.
    /// @param callback A user implemented callback function.
    ///
    void RayCast(Length2 point1, Length2 point2, RayCastCallback callback) const;

    /// @brief Gets the world body range for this world.
    /// @return Body range that can be iterated over using its begin and end methods
    ///   or using ranged-based for-loops.
    SizedRange<Bodies::iterator> GetBodies() noexcept;

    /// @brief Gets the world body range for this constant world.
    /// @return Body range that can be iterated over using its begin and end methods
    ///   or using ranged-based for-loops.
    SizedRange<Bodies::const_iterator> GetBodies() const noexcept;

    /// @brief Gets the world joint range.
    /// @return World joints sized-range.
    SizedRange<Joints::const_iterator> GetJoints() const noexcept;

    /// @brief Gets the world joint range.
    /// @return World joints sized-range.
    SizedRange<Joints::iterator> GetJoints() noexcept;

    /// @brief Gets the world contact range.
    /// @warning contacts are created and destroyed in the middle of a time step.
    /// Use ContactListener to avoid missing contacts.
    /// @return World contacts sized-range.
    SizedRange<Contacts::const_iterator> GetContacts() const noexcept;
    
    /// @brief Gets whether or not sub-stepping is enabled.
    bool GetSubStepping() const noexcept;

    /// @brief Enables/disables single stepped continuous physics.
    /// @note This is for testing.
    void SetSubStepping(bool flag) noexcept;

    /// @brief Gets access to the broad-phase dynamic tree information.
    const DynamicTree& GetTree() const noexcept;
    
    /// @brief Changes the global gravity vector.
    void SetGravity(LinearAcceleration2 gravity) noexcept;
    
    /// @brief Gets the global gravity vector.
    LinearAcceleration2 GetGravity() const noexcept;

    /// @brief Is the world locked (in the middle of a time step).
    bool IsLocked() const noexcept;

    /// @brief Shifts the world origin.
    /// @note Useful for large worlds.
    /// @note The body shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    /// @throws WrongState if this method is called while the world is locked.
    void ShiftOrigin(Length2 newOrigin);

    /// @brief Gets the minimum vertex radius that shapes in this world can be.
    Length GetMinVertexRadius() const noexcept;
    
    /// @brief Gets the maximum vertex radius that shapes in this world can be.
    Length GetMaxVertexRadius() const noexcept;

    /// @brief Gets the inverse delta time.
    Frequency GetInvDeltaTime() const noexcept;
    
    /// @brief Sets the type of the given body.
    /// @note This may alter the body's mass and velocity.
    /// @throws WrongState if this method is called while the world is locked.
    void SetType(Body& body, BodyType type);

    /// @brief Register for proxies for the given fixture.
    bool RegisterForProxies(Fixture* fixture);
    
    /// @brief Register for proxies for the given body.
    bool RegisterForProxies(Body* body);

    /// @brief Creates a fixture with the given parameters.
    /// @throws InvalidArgument if called for a body that doesn't belong to this world.
    /// @throws InvalidArgument if called without a shape.
    /// @throws InvalidArgument if called for a shape with a vertex radius less than the
    ///    minimum vertex radius.
    /// @throws InvalidArgument if called for a shape with a vertex radius greater than the
    ///    maximum vertex radius.
    /// @throws WrongState if this method is called while the world is locked.
    Fixture* CreateFixture(Body& body, const Shape& shape,
                           const FixtureDef& def = GetDefaultFixtureDef(),
                           bool resetMassData = true);

    /// @brief Destroys a fixture.
    ///
    /// @details This removes the fixture from the broad-phase and destroys all contacts
    ///   associated with this fixture.
    ///   All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    ///
    /// @warning This function is locked during callbacks.
    /// @note Make sure to explicitly call ResetMassData after fixtures have been destroyed.
    ///
    /// @param fixture the fixture to be removed.
    /// @param resetMassData Whether or not to reset the mass data of the associated body.
    ///
    /// @sa ResetMassData.
    ///
    /// @throws WrongState if this method is called while the world is locked.
    ///
    bool DestroyFixture(Fixture* fixture, bool resetMassData = true);
    
    /// @brief Touches each proxy of the given fixture.
    /// @note Fixture must belong to a body that belongs to this world or this method will
    ///   return false.
    /// @note This sets things up so that pairs may be created for potentially new contacts.
    bool TouchProxies(Fixture& fixture) noexcept;
    
    /// @brief Sets new fixtures flag.
    void SetNewFixtures() noexcept;

private:

    /// @brief Flags type data type.
    using FlagsType = std::uint32_t;

    /// @brief Proxy ID type alias.
    using ProxyId = DynamicTree::Size;

    /// @brief Contact key queue type alias.
    using ContactKeyQueue = std::vector<ContactKey>;
    
    /// @brief Proxy queue type alias.
    using ProxyQueue = std::vector<ProxyId>;
    
    /// @brief Fixture queue type alias.
    using FixtureQueue = std::vector<Fixture*>;
    
    /// @brief Body queue type alias.
    using BodyQueue = std::vector<Body*>;
    
    /// @brief Flag enumeration.
    enum Flag: FlagsType
    {
        /// New fixture.
        e_newFixture    = 0x0001,

        /// Locked.
        e_locked        = 0x0002,

        /// Substepping.
        e_substepping   = 0x0020,
        
        /// Step complete. @details Used for sub-stepping. @sa e_substepping.
        e_stepComplete  = 0x0040,
    };

    /// @brief Island solver results.
    struct IslandSolverResults
    {
        Length minSeparation = std::numeric_limits<Length>::infinity(); ///< Minimum separation.
        Momentum maxIncImpulse = 0; ///< Maximum incremental impulse.
        BodyCounter bodiesSlept = 0; ///< Bodies slept.
        ContactCounter contactsUpdated = 0; ///< Contacts updated.
        ContactCounter contactsSkipped = 0; ///< Contacts skipped.
        bool solved = false; ///< Solved. <code>true</code> if position constraints solved, <code>false</code> otherwise.
        TimestepIters positionIterations = 0; ///< Position iterations actually performed.
        TimestepIters velocityIterations = 0; ///< Velocity iterations actually performed.
    };
    
    /// @brief Updates the given regular step statistics.
    static RegStepStats& Update(RegStepStats& lhs, const IslandSolverResults& rhs) noexcept;

    /// @brief Copies bodies.
    void CopyBodies(std::map<const Body*, Body*>& bodyMap,
                    std::map<const Fixture*, Fixture*>& fixtureMap,
                    SizedRange<World::Bodies::const_iterator> range);
    
    /// @brief Copies joints.
    void CopyJoints(const std::map<const Body*, Body*>& bodyMap,
                    SizedRange<World::Joints::const_iterator> range);
    
    /// @brief Copies contacts.
    void CopyContacts(const std::map<const Body*, Body*>& bodyMap,
                      const std::map<const Fixture*, Fixture*>& fixtureMap,
                      SizedRange<World::Contacts::const_iterator> range);
    
    /// @brief Internal destroy.
    /// @warning Behavior is undefined if passed a null pointer for the joint.
    void InternalDestroy(Joint* joint);

    /// @brief Solves the step.
    /// @details Finds islands, integrates and solves constraints, solves position constraints.
    /// @note This may miss collisions involving fast moving bodies and allow them to tunnel
    ///   through each other.
    RegStepStats SolveReg(const StepConf& conf);

    /// @brief Solves the given island (regularly).
    ///
    /// @details This:
    ///   1. Updates every island-body's sweep.pos0 to its sweep.pos1.
    ///   2. Updates every island-body's sweep.pos1 to the new normalized "solved" position for it.
    ///   3. Updates every island-body's velocity to the new accelerated, dampened, and "solved"
    ///      velocity for it.
    ///   4. Synchronizes every island-body's transform (by updating it to transform one of the
    ///      body's sweep).
    ///   5. Reports to the listener (if non-null).
    ///
    /// @param conf Time step configuration information.
    /// @param island Island of bodies, contacts, and joints to solve for. Must contain at least
    ///   one body, contact, or joint.
    ///
    /// @warning Behavior is undefined if the given island doesn't have at least one body,
    ///   contact, or joint.
    ///
    /// @return Island solver results.
    ///
    IslandSolverResults SolveRegIslandViaGS(const StepConf& conf, Island island);
    
    /// @brief Adds to the island based off of a given "seed" body.
    /// @post Contacts are listed in the island in the order that bodies provide those contacts.
    /// @post Joints are listed the island in the order that bodies provide those joints.
    void AddToIsland(Island& island, Body& seed,
                     Bodies::size_type& remNumBodies,
                     Contacts::size_type& remNumContacts,
                     Joints::size_type& remNumJoints);

    /// @brief Body stack.
    /// @note Using a std::stack<Body*, std::vector<Body*>> would be nice except it doesn't
    ///   support the reserve method.
    using BodyStack = std::vector<Body*>;

    /// @brief Adds to the island.
    void AddToIsland(Island& island, BodyStack& stack,
                     Bodies::size_type& remNumBodies,
                     Contacts::size_type& remNumContacts,
                     Joints::size_type& remNumJoints);
    
    /// @brief Adds contacts to the island.
    void AddContactsToIsland(Island& island, BodyStack& stack, const Body* b);

    /// @brief Adds joints to the island.
    void AddJointsToIsland(Island& island, BodyStack& stack, const Body* b);
    
    /// @brief Removes unspeedables from the is islanded state.
    Bodies::size_type RemoveUnspeedablesFromIslanded(const std::vector<Body*>& bodies);

    /// @brief Solves the step using successive time of impact (TOI) events.
    /// @details Used for continuous physics.
    /// @note This is intended to detect and prevent the tunneling that the faster Solve method
    ///    may miss.
    /// @param conf Time step configuration to use.
    ToiStepStats SolveToi(const StepConf& conf);

    /// @brief Solves collisions for the given time of impact.
    ///
    /// @param conf Time step configuration to solve for.
    /// @param contact Contact.
    ///
    /// @note Precondition 1: there is no contact having a lower TOI in this time step that has
    ///   not already been solved for.
    /// @note Precondition 2: there is not a lower TOI in the time step for which collisions have
    ///   not already been processed.
    ///
    IslandSolverResults SolveToi(const StepConf& conf, Contact& contact);
    
    /// @brief Solves the time of impact for bodies 0 and 1 of the given island.
    ///
    /// @details This:
    ///   1. Updates pos0 of the sweeps of bodies 0 and 1.
    ///   2. Updates pos1 of the sweeps, the transforms, and the velocities of the other bodies
    ///      in this island.
    ///
    /// @pre <code>island.m_bodies</code> contains at least two bodies, the first two of which
    ///   are bodies 0 and 1.
    /// @pre <code>island.m_bodies</code> contains appropriate other bodies of the contacts of
    ///   the two bodies.
    /// @pre <code>island.m_contacts</code> contains the contact that specified the two identified
    ///   bodies.
    /// @pre <code>island.m_contacts</code> contains appropriate other contacts of the two bodies.
    ///
    /// @param conf Time step configuration information.
    /// @param island Island to do time of impact solving for.
    ///
    /// @return Island solver results.
    ///
    IslandSolverResults SolveToiViaGS(const StepConf& conf, Island& island);

    /// @brief Updates the given body.
    /// @details Updates the given body's velocity, sweep position 1, and its transformation.
    /// @param body Body to update.
    /// @param pos New position to set the given body to.
    /// @param vel New velocity to set the given body to.
    static void UpdateBody(Body& body, const Position2D& pos, const Velocity2D& vel);

    /// @brief Reset bodies for solve TOI.
    void ResetBodiesForSolveTOI();

    /// @brief Reset contacts for solve TOI.
    void ResetContactsForSolveTOI();
    
    /// @brief Reset contacts for solve TOI.
    void ResetContactsForSolveTOI(Body& body);

    /// @brief Process contacts output.
    struct ProcessContactsOutput
    {
        ContactCounter contactsUpdated = 0; ///< Contacts updated.
        ContactCounter contactsSkipped = 0; ///< Contacts skipped.
    };

    /// @brief Processes the contacts of a given body for TOI handling.
    /// @details This does the following:
    ///   1. Advances the appropriate associated other bodies to the given TOI (advancing
    ///      their sweeps and synchronizing their transforms to their new sweeps).
    ///   2. Updates the contact manifolds and touching statuses and notifies listener (if one given) of
    ///      the appropriate contacts of the body.
    ///   3. Adds those contacts that are still enabled and still touching to the given island
    ///      (or resets the other bodies advancement).
    ///   4. Adds to the island, those other bodies that haven't already been added of the contacts that got added.
    /// @note Precondition: there should be no lower TOI for which contacts have not already been processed.
    /// @param[in,out] island Island. On return this may contain additional contacts or bodies.
    /// @param[in,out] body A dynamic/accelerable body.
    /// @param[in] toi Time of impact (TOI). Value between 0 and 1.
    /// @param[in] conf Step configuration data.
    ProcessContactsOutput ProcessContactsForTOI(Island& island, Body& body, Real toi,
                                                const StepConf& conf);

    /// @brief Adds the given joint to this world.
    /// @note This also adds the joint to the bodies of the joint.
    bool Add(Joint* j);

    /// @brief Removes the given body from this world.
    bool Remove(const Body& b);
 
    /// @brief Removes the given joint from this world.
    bool Remove(Joint& j);

    /// @brief Whether or not "step" is complete.
    /// @details The "step" is completed when there are no more TOI events for the current time step.
    /// @sa <code>SetStepComplete</code>.
    bool IsStepComplete() const noexcept;

    /// @brief Sets the step complete state.
    void SetStepComplete(bool value) noexcept;

    /// @brief Sets the allow sleeping state.
    void SetAllowSleeping() noexcept;

    /// @brief Unsets the allow sleeping state.
    void UnsetAllowSleeping() noexcept;
    
    /// @brief Update contacts statistics.
    struct UpdateContactsStats
    {
        /// @brief Number of contacts ignored (because both bodies were asleep).
        ContactCounter ignored = 0;

        /// @brief Number of contacts updated.
        ContactCounter updated = 0;
        
        /// @brief Number of contacts skipped because they weren't marked as needing updating.
        ContactCounter skipped = 0;
    };
    
    /// @brief Destroy contacts statistics.
    struct DestroyContactsStats
    {
        ContactCounter ignored = 0; ///< Ignored.
        ContactCounter erased = 0; ///< Erased.
    };
    
    /// @brief Contacts TOI data.
    struct ContactToiData
    {
        std::vector<Contact*> contacts; ///< Contacts for which the time of impact is relavant.
        Real toi = std::numeric_limits<Real>::infinity(); ///< Time of impact (TOI) as a fractional value between 0 and 1.
    };

    /// @brief Update contacts data.
    struct UpdateContactsData
    {
        ContactCounter numAtMaxSubSteps = 0; ///< # at max sub-steps (lower the better).
        ContactCounter numUpdatedTOI = 0; ///< # updated TOIs (made valid).
        ContactCounter numValidTOI = 0; ///< # already valid TOIs.
    
        /// @brief Distance iterations type alias.
        using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;

        /// @brief TOI iterations type alias.
        using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
        
        /// @brief Root iterations type alias.
        using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
        
        dist_iter_type maxDistIters = 0; ///< Max distance iterations.
        toi_iter_type maxToiIters = 0; ///< Max TOI iterations.
        root_iter_type maxRootIters = 0; ///< Max root iterations.
    };
    
    /// @brief Updates the contact times of impact.
    UpdateContactsData UpdateContactTOIs(const StepConf& conf);

    /// @brief Gets the soonest contact.
    /// @details This finds the contact with the lowest (soonest) time of impact.
    /// @return Contacts with the least time of impact and its time of impact, or null contact.
    ///  These contacts will all be enabled, not have sensors, be active, and impenetrable.
    ContactToiData GetSoonestContacts(std::size_t reserveSize);

    /// @brief Determines whether this world has new fixtures.
    bool HasNewFixtures() const noexcept;
    
    /// @brief Unsets the new fixtures state.
    void UnsetNewFixtures() noexcept;
    
    /// @brief Finds new contacts.
    /// @details Finds and adds new valid contacts to the contacts container.
    /// @note The new contacts will all have overlapping AABBs.
    ContactCounter FindNewContacts();
    
    /// @brief Processes the narrow phase collision for the contacts collection.
    /// @details
    /// This finds and destroys the contacts that need filtering and no longer should collide or
    /// that no longer have AABB-based overlapping fixtures. Those contacts that persist and
    /// have active bodies (either or both) get their Update methods called with the current
    /// contact listener as its argument.
    /// Essentially this really just purges contacts that are no longer relevant.
    DestroyContactsStats DestroyContacts(Contacts& contacts);
    
    /// @brief Update contacts.
    UpdateContactsStats UpdateContacts(Contacts& contacts, const StepConf& conf);
    
    /// @brief Determines whether the two fixtures should collide.
    bool ShouldCollide(const Fixture* fixtureA, const Fixture* fixtureB);

    /// @brief Destroys the given contact and removes it from its container.
    /// @details This updates the contacts container, returns the memory to the allocator,
    ///   and decrements the contact manager's contact count.
    /// @param contact Contact to destroy.
    /// @param from From body.
    void Destroy(Contact* contact, Body* from);
    
    /// @brief Adds a contact for the proxies identified by the key if appropriate.
    /// @details Adds a new contact object to represent a contact between proxy A and proxy B
    /// if all of the following are true:
    ///   1. The bodies of the fixtures of the proxies are not the one and the same.
    ///   2. No contact already exists for these two proxies.
    ///   3. The bodies of the proxies should collide (according to Body::ShouldCollide).
    ///   4. The contact filter says the fixtures of the proxies should collide.
    ///   5. There exists a contact-create function for the pair of shapes of the proxies.
    /// @post The size of the <code>m_contacts</code> collection is one greater-than it was
    ///   before this method is called if it returns <code>true</code>.
    /// @param key ID's of dynamic tree entries identifying the fixture proxies involved.
    /// @return <code>true</code> if a new contact was indeed added (and created),
    ///   else <code>false</code>.
    /// @sa bool Body::ShouldCollide(const Body* other) const
    bool Add(ContactKey key);
    
    /// @brief Registers the given dynamic tree ID for processing.
    void RegisterForProcessing(ProxyId pid) noexcept;

    /// @brief Unregisters the given dynamic tree ID from processing.
    void UnregisterForProcessing(ProxyId pid) noexcept;

    /// @brief Destroys the given contact.
    void InternalDestroy(Contact* contact, Body* from = nullptr);

    /// @brief Creates proxies for every child of the given fixture's shape.
    /// @note This sets the proxy count to the child count of the shape.
    void CreateProxies(Fixture& fixture, Length aabbExtension);

    /// @brief Destroys the given fixture's proxies.
    /// @note This resets the proxy count to 0.
    void DestroyProxies(Fixture& fixture);

    /// @brief Touches each proxy of the given fixture.
    /// @note This sets things up so that pairs may be created for potentially new contacts.
    void InternalTouchProxies(Fixture& fixture) noexcept;
    
    /// @brief Synchronizes the given body.
    /// @details This updates the broad phase dynamic tree data for all of the given
    ///   body's fixtures.
    ContactCounter Synchronize(Body& body,
                               Transformation2D xfm1, Transformation2D xfm2,
                               Real multiplier, Length extension);

    /// @brief Synchronizes the given fixture.
    /// @details This updates the broad phase dynamic tree data for all of the given
    ///   fixture shape's children.
    ContactCounter Synchronize(Fixture& fixture,
                               Transformation2D xfm1, Transformation2D xfm2,
                               Length2 displacement, Length extension);
    
    /// @brief Creates and destroys proxies.
    void CreateAndDestroyProxies(const StepConf& conf);

    /// @brief Creates and destroys proxies for the given fixture.
    void CreateAndDestroyProxies(Fixture& fixture, const StepConf& conf);
    
    /// @brief Synchronizes proxies of the bodies for proxies.
    PreStepStats::counter_type SynchronizeProxies(const StepConf& conf);

    /// @brief Whether the given body is in an island.
    bool IsIslanded(const Body* body) const noexcept;

    /// @brief Whether the given contact is in an island.
    bool IsIslanded(const Contact* contact) const noexcept;

    /// @brief Whether the given joint is in an island.
    bool IsIslanded(const Joint* joint) const noexcept;

    /// @brief Sets the given body to the in an island state.
    void SetIslanded(Body* body) noexcept;

    /// @brief Sets the given contact to the in an island state.
    void SetIslanded(Contact* contact) noexcept;

    /// @brief Sets the given joint to the in an island state.
    void SetIslanded(Joint* joint) noexcept;

    /// @brief Unsets the given body's in island state.
    void UnsetIslanded(Body* body) noexcept;

    /// @brief Unsets the given contact's in island state.
    void UnsetIslanded(Contact* contact) noexcept;
    
    /// @brief Unsets the given joint's in island state.
    void UnsetIslanded(Joint* joint) noexcept;

    /******** Member variables. ********/
    
    DynamicTree m_tree; ///< Dynamic tree.
    
    ContactKeyQueue m_proxyKeys; ///< Proxy keys.
    ProxyQueue m_proxies; ///< Proxies queue.
    FixtureQueue m_fixturesForProxies; ///< Fixtures for proxies queue.
    BodyQueue m_bodiesForProxies; ///< Bodies for proxies queue.

    ContactFilter m_defaultFilter; ///< Default contact filter. 8-bytes.
    
    Bodies m_bodies; ///< Body collection.

    Joints m_joints; ///< Joint collection.

    /// @brief Container of contacts.
    /// @note In the "AddPair" stress-test, 401 bodies can have some 31000 contacts
    ///   during a given time step.
    Contacts m_contacts;

    LinearAcceleration2 m_gravity; ///< Gravity setting. 8-bytes.

    DestructionListener* m_destructionListener = nullptr; ///< Destruction listener. 8-bytes.
    
    ContactListener* m_contactListener = nullptr; ///< Contact listener. 8-bytes.
    
    ContactFilter* m_contactFilter = &m_defaultFilter; ///< Contact filter. 8-bytes.
    
    FlagsType m_flags = e_stepComplete; ///< Flags.

    /// Inverse delta-t from previous step.
    /// @details Used to compute time step ratio to support a variable time step.
    /// @note 4-bytes large.
    /// @sa Step.
    Frequency m_inv_dt0 = 0;

    /// @brief Minimum vertex radius.
    Positive<Length> m_minVertexRadius;

    /// @brief Maximum vertex radius.
    /// @details
    /// This is the maximum shape vertex radius that any bodies' of this world should create
    /// fixtures for. Requests to create fixtures for shapes with vertex radiuses bigger than
    /// this must be rejected. As an upper bound, this value prevents shapes from getting
    /// associated with this world that would otherwise not be able to be simulated due to
    /// numerical issues. It can also be set below this upper bound to constrain the differences
    /// between shape vertex radiuses to possibly more limited visual ranges.
    Positive<Length> m_maxVertexRadius;
};

/// @example HelloWorld.cpp
///   This is an example of how to use the World class.
///

/// @brief World ray cast opcode enumeration.
enum class World::RayCastOpcode
{
    /// @brief End the ray-cast search for fixtures.
    /// @details Use this to stop searching for fixtures.
    Terminate,
    
    /// @brief Ignore the current fixture.
    /// @details Use this to continue searching for fixtures along the ray.
    IgnoreFixture,
    
    /// @brief Clip the ray end to the current point.
    /// @details Use this shorten the ray to the current point and to continue searching
    ///   for fixtures now along the newly shortened ray.
    ClipRay,
    
    /// @brief Reset the ray end back to the second point.
    /// @details Use this to restore the ray to its full length and to continue searching
    ///    for fixtures now along the restored full length ray.
    ResetRay
};

inline SizedRange<World::Bodies::iterator> World::GetBodies() noexcept
{
    return {m_bodies.begin(), m_bodies.end(), m_bodies.size()};
}

inline SizedRange<World::Bodies::const_iterator> World::GetBodies() const noexcept
{
    return {m_bodies.begin(), m_bodies.end(), m_bodies.size()};
}

inline SizedRange<World::Joints::const_iterator> World::GetJoints() const noexcept
{
    return {m_joints.begin(), m_joints.end(), m_joints.size()};
}

inline SizedRange<World::Joints::iterator> World::GetJoints() noexcept
{
    return {m_joints.begin(), m_joints.end(), m_joints.size()};
}

inline SizedRange<World::Contacts::const_iterator> World::GetContacts() const noexcept
{
    return {m_contacts.begin(), m_contacts.end(), m_contacts.size()};
}

inline LinearAcceleration2 World::GetGravity() const noexcept
{
    return m_gravity;
}

inline bool World::IsLocked() const noexcept
{
    return (m_flags & e_locked) == e_locked;
}

inline bool World::IsStepComplete() const noexcept
{
    return (m_flags & e_stepComplete) != 0u;
}

inline void World::SetStepComplete(bool value) noexcept
{
    if (value)
    {
        m_flags |= e_stepComplete;
    }
    else
    {
        m_flags &= ~e_stepComplete;        
    }
}

inline bool World::GetSubStepping() const noexcept
{
    return (m_flags & e_substepping) != 0u;
}

inline void World::SetSubStepping(bool flag) noexcept
{
    if (flag)
    {
        m_flags |= e_substepping;
    }
    else
    {
        m_flags &= ~e_substepping;
    }
}

inline bool World::HasNewFixtures() const noexcept
{
    return (m_flags & e_newFixture) != 0u;
}

inline void World::SetNewFixtures() noexcept
{
    m_flags |= e_newFixture;
}

inline void World::UnsetNewFixtures() noexcept
{
    m_flags &= ~e_newFixture;
}

inline Length World::GetMinVertexRadius() const noexcept
{
    return m_minVertexRadius;
}

inline Length World::GetMaxVertexRadius() const noexcept
{
    return m_maxVertexRadius;
}

inline Frequency World::GetInvDeltaTime() const noexcept
{
    return m_inv_dt0;
}

inline const DynamicTree& World::GetTree() const noexcept
{
    return m_tree;
}

inline void World::SetDestructionListener(DestructionListener* listener) noexcept
{
    m_destructionListener = listener;
}

inline void World::SetContactFilter(ContactFilter* filter) noexcept
{
    m_contactFilter = filter;
}

inline void World::SetContactListener(ContactListener* listener) noexcept
{
    m_contactListener = listener;
}

inline bool World::ShouldCollide(const Fixture *fixtureA, const Fixture *fixtureB)
{
    return (m_contactFilter == nullptr) || m_contactFilter->ShouldCollide(fixtureA, fixtureB);
}

inline bool World::IsIslanded(const Body* body) const noexcept
{
    return BodyAtty::IsIslanded(*body);
}

inline bool World::IsIslanded(const Contact* contact) const noexcept
{
    return ContactAtty::IsIslanded(*contact);
}

inline bool World::IsIslanded(const Joint* joint) const noexcept
{
    return JointAtty::IsIslanded(*joint);
}

inline void World::SetIslanded(Body* body) noexcept
{
    BodyAtty::SetIslanded(*body);
}

inline void World::SetIslanded(Contact* contact) noexcept
{
    ContactAtty::SetIslanded(*contact);
}

inline void World::SetIslanded(Joint* joint) noexcept
{
    JointAtty::SetIslanded(*joint);
}

inline void World::UnsetIslanded(Body* body) noexcept
{
    BodyAtty::UnsetIslanded(*body);
}

inline void World::UnsetIslanded(Contact* contact) noexcept
{
    ContactAtty::UnsetIslanded(*contact);
}

inline void World::UnsetIslanded(Joint* joint) noexcept
{
    JointAtty::UnsetIslanded(*joint);
}

inline void World::RegisterForProcessing(ProxyId pid) noexcept
{
    assert(pid != DynamicTree::GetInvalidSize());
    m_proxies.push_back(pid);
}

inline RegStepStats& World::Update(RegStepStats& lhs, const World::IslandSolverResults& rhs) noexcept
{
    lhs.maxIncImpulse = std::max(lhs.maxIncImpulse, rhs.maxIncImpulse);
    lhs.minSeparation = std::min(lhs.minSeparation, rhs.minSeparation);
    lhs.islandsSolved += rhs.solved;
    lhs.sumPosIters += rhs.positionIterations;
    lhs.sumVelIters += rhs.velocityIterations;
    lhs.bodiesSlept += rhs.bodiesSlept;
    return lhs;
}

// Free functions.

/// @brief Gets the body count in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline BodyCounter GetBodyCount(const World& world) noexcept
{
    return static_cast<BodyCounter>(world.GetBodies().size());
}

/// Gets the count of joints in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline JointCounter GetJointCount(const World& world) noexcept
{
    return static_cast<JointCounter>(world.GetJoints().size());
}

/// @brief Gets the count of contacts in the given world.
/// @note Not all contacts are for shapes that are actually touching. Some contacts are for
///   shapes which merely have overlapping AABBs.
/// @return 0 or higher.
/// @relatedalso World
inline ContactCounter GetContactCount(const World& world) noexcept
{
    return static_cast<ContactCounter>(world.GetContacts().size());
}

/// @brief Gets the touching count for the given world.
/// @relatedalso World
ContactCounter GetTouchingCount(const World& world) noexcept;

/// @brief Steps the world ahead by a given time amount.
///
/// @details Performs position and velocity updating, sleeping of non-moving bodies, updating
///   of the contacts, and notifying the contact listener of begin-contact, end-contact,
///   pre-solve, and post-solve events.
///   If the given velocity and position iterations are more than zero, this method also
///   respectively performs velocity and position resolution of the contacting bodies.
///
/// @note While body velocities are updated accordingly (per the sum of forces acting on them),
///   body positions (barring any collisions) are updated as if they had moved the entire time
///   step at those resulting velocities. In other words, a body initially at p0 going v0 fast
///   with a sum acceleration of a, after time t and barring any collisions, will have a new
///   velocity (v1) of v0 + (a * t) and a new position (p1) of p0 + v1 * t.
///
/// @warning Varying the time step may lead to non-physical behaviors.
///
/// @post Static bodies are unmoved.
/// @post Kinetic bodies are moved based on their previous velocities.
/// @post Dynamic bodies are moved based on their previous velocities, gravity,
/// applied forces, applied impulses, masses, damping, and the restitution and friction values
/// of their fixtures when they experience collisions.
///
/// @param world World to step.
/// @param delta Time to simulate as a delta from the current state. This should not vary.
/// @param velocityIterations Number of iterations for the velocity constraint solver.
/// @param positionIterations Number of iterations for the position constraint solver.
///   The position constraint solver resolves the positions of bodies that overlap.
///
/// @relatedalso World
///
StepStats Step(World& world, Time delta,
               TimestepIters velocityIterations = 8,
               TimestepIters positionIterations = 3);

/// @brief Gets the count of fixtures in the given world.
/// @relatedalso World
std::size_t GetFixtureCount(const World& world) noexcept;

/// @brief Gets the count of unique shapes in the given world.
/// @relatedalso World
std::size_t GetShapeCount(const World& world) noexcept;

/// @brief Gets the count of awake bodies in the given world.
/// @relatedalso World
BodyCounter GetAwakeCount(const World& world) noexcept;

/// @brief Awakens all of the bodies in the given world.
/// @details Calls all of the world's bodies' <code>SetAwake</code> method.
/// @return Sum total of calls to bodies' <code>SetAwake</code> method that returned true.
/// @sa Body::SetAwake.
/// @relatedalso World
BodyCounter Awaken(World& world) noexcept;

/// @brief Sets the accelerations of all the world's bodies.
/// @relatedalso World
void SetAccelerations(World& world, std::function<Acceleration2D(const Body& b)> fn) noexcept;

/// @brief Sets the accelerations of all the world's bodies to the given value.
/// @relatedalso World
void SetAccelerations(World& world, Acceleration2D acceleration) noexcept;

/// @brief Clears forces.
/// @details Manually clear the force buffer on all bodies.
/// @relatedalso World
inline void ClearForces(World& world) noexcept
{
    SetAccelerations(world, Acceleration2D{world.GetGravity(), 0 * RadianPerSquareSecond});
}

/// @brief Creates a rectanglular enclosure.
/// @relatedalso World
Body* CreateRectangularEnclosingBody(World& world, Length2 dimensions,
                                     const ShapeDef& baseConf);

/// @brief Creates a square enclosure.
/// @relatedalso World
inline Body* CreateSquareEnclosingBody(World& world, Length size, const ShapeDef& baseConf)
{
    return CreateRectangularEnclosingBody(world, Length2{size, size}, baseConf);
}
    
/// @brief Finds body in given world that's closest to the given location.
/// @relatedalso World
Body* FindClosestBody(const World& world, Length2 location) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLD_HPP
