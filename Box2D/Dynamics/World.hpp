/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_WORLD_H
#define B2_WORLD_H

/// @file
/// Declarations of the World class and associated free functions.

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Range.hpp>
#include <Box2D/Dynamics/WorldDef.hpp>
#include <Box2D/Dynamics/BodyDef.hpp>
#include <Box2D/Dynamics/FixtureDef.hpp>
#include <Box2D/Dynamics/WorldCallbacks.hpp>
#include <Box2D/Dynamics/StepStats.hpp>
#include <Box2D/Collision/BroadPhase.hpp>

#include <vector>
#include <map>
#include <list>
#include <unordered_set>
#include <memory>
#include <stdexcept>

namespace box2d {

class AABB;
struct BodyDef;
struct JointDef;
struct FixtureDef;
struct FixtureProxy;
class Body;
class Contact;
class Fixture;
class Joint;
class Island;
class StepConf;
class Shape;
enum class BodyType;

/// @brief World.
///
/// @details The world class manages all physics entities, dynamic simulation, and queries.
///
/// @note From a memory management perspective, world instances own Body, Joint, and Contact
///   instances.
/// @note This data structure is 352-bytes large (with 4-byte RealNum on at least one 64-bit
///   platform).
///
class World
{
public:
    
    /// @brief Proxy size type.
    using proxy_size_type = std::remove_const<decltype(MaxContacts)>::type;

    /// @brief Time step iteration type.
    using ts_iters_type = ts_iters_t;
    
    /// @brief Bodies container type.
    using Bodies = std::list<Body>;

    /// @brief Contacts container type.
    using Contacts = std::list<Contact>;
    
    /// @brief Joints container type.
    /// @note Cannot be container of Joint instances since joints are polymorphic types.
    using Joints = std::list<Joint*>;
    
    class LockedError;

    /// @brief Constructs a world object.
    /// @throws InvalidArgument if the given max vertex radius is less than the min.
    World(const WorldDef& def = GetDefaultWorldDef());

    World(const World& copy);

    World& operator= (const World& other);

    /// @brief Destructor.
    /// @details
    /// All physics entities are destroyed and all dynamically allocated memory is released.
    ~World();

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
    Body* CreateBody(const BodyDef& def = GetDefaultBodyDef());

    /// @brief Destroys the given body.
    /// @note This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    void Destroy(Body* body);

    /// @brief Creates a joint to constrain bodies together.
    /// @details No reference to the definition
    /// is retained. This may cause the connected bodies to cease colliding.
    /// @warning This function is locked during callbacks.
    /// @return <code>nullptr</code> if world has <code>MaxJoints</code>,
    ///   else pointer to newly created joint.
    Joint* CreateJoint(const JointDef& def);

    /// @brief Destroys a joint.
    ///
    /// @details This may cause the connected bodies to begin colliding.
    ///
    /// @warning This function is locked during callbacks.
    /// @warning Behavior is undefined if the passed joint was not created by this world.
    ///
    /// @param joint Joint, created by this world, to destroy.
    ///
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
    /// @post Dynamic bodies are moved based on their previous velocities, gravity,
    /// applied forces, applied impulses, masses, damping, and the restitution and friction values
    /// of their fixtures when they experience collisions.
    ///
    /// @param conf Configuration for the simulation step.
    ///
    /// @return Statistics for the step.
    ///
    StepStats Step(const StepConf& conf);

    /// @brief Query AABB for fixtures callback function type.
    /// @note Returning true will continue the query. Returning false will terminate the query.
    using QueryFixtureCallback = std::function<bool(Fixture* fixture,
                                                    const child_count_t child)>;

    /// @brief Queries the world for all fixtures that potentially overlap the provided AABB.
    /// @param aabb the query box.
    /// @param callback User implemented callback function.
    void QueryAABB(const AABB& aabb, QueryFixtureCallback callback);

    /// @brief Ray-cast operation code.
    ///
    /// @details Instructs the <code>RayCast</code> method on what to do next.
    ///
    enum class RayCastOpcode;

    /// @brief Ray cast callback function signature.
    using RayCastCallback = std::function<RayCastOpcode(Fixture* fixture,
                                                        const child_count_t child,
                                                        const Length2D& point,
                                                        const UnitVec2& normal)>;

    /// @brief Ray-cast the world for all fixtures in the path of the ray.
    ///
    /// @note The callback controls whether you get the closest point, any point, or n-points.
    /// @note The ray-cast ignores shapes that contain the starting point.
    ///
    /// @param point1 Ray starting point.
    /// @param point2 Ray ending point.
    /// @param callback A user implemented callback function.
    ///
    void RayCast(const Length2D& point1, const Length2D& point2, RayCastCallback callback);

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

    /// @brief Gets the number of broad-phase proxies.
    proxy_size_type GetProxyCount() const noexcept;

    /// @brief Gets the height of the dynamic tree.
    proxy_size_type GetTreeHeight() const noexcept;

    /// @brief Gets the balance of the dynamic tree.
    proxy_size_type GetTreeBalance() const;

    /// @brief Gets the quality metric of the dynamic tree.
    /// @details The smaller the better.
    /// @return Value of zero or more.
    RealNum GetTreeQuality() const;

    /// @brief Changes the global gravity vector.
    void SetGravity(const LinearAcceleration2D gravity) noexcept;
    
    /// @brief Gets the global gravity vector.
    LinearAcceleration2D GetGravity() const noexcept;

    /// @brief Is the world locked (in the middle of a time step).
    bool IsLocked() const noexcept;

    /// Shift the world origin. Useful for large worlds.
    /// The body shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const Length2D newOrigin);

    /// @brief Gets the minimum vertex radius that shapes in this world can be.
    Length GetMinVertexRadius() const noexcept;
    
    /// @brief Gets the maximum vertex radius that shapes in this world can be.
    Length GetMaxVertexRadius() const noexcept;

    /// @brief Gets the inverse delta time.
    Frequency GetInvDeltaTime() const noexcept;

    /// @brief Gets the fat AABB for a proxy.
    /// @warning Behavior is undefined if the given proxy ID is not a valid ID.
    AABB GetFatAABB(proxy_size_type proxyId) const;
    
    /// @brief Sets the type of the given body.
    /// @note This may alter the body's mass and velocity.
    void SetType(Body& body, BodyType type);

    bool RegisterForProxies(Fixture* fixture);
    bool RegisterForProxies(Body* body);

    Fixture* CreateFixture(Body& body, std::shared_ptr<const Shape> shape,
                           const FixtureDef& def = GetDefaultFixtureDef(),
                           bool resetMassData = true);

    /// @brief Destroys a fixture.
    ///
    /// @details This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    ///
    /// @warning This function is locked during callbacks.
    /// @note Make sure to explicitly call ResetMassData after fixtures have been destroyed.
    ///
    /// @param fixture the fixture to be removed.
    /// @param resetMassData Whether or not to reset the mass data of the associated body.
    ///
    /// @sa ResetMassData.
    ///
    bool DestroyFixture(Fixture* fixture, bool resetMassData = true);
    
    bool IsValid(std::shared_ptr<const Shape> shape) const noexcept;
    
    /// @brief Touches each proxy of the given fixture.
    /// @note Fixture must belong to a body that belongs to this world or this method will
    ///   return false.
    /// @note This sets things up so that pairs may be created for potentially new contacts.
    bool TouchProxies(Fixture& fixture) noexcept;
    
    void SetNewFixtures() noexcept;

private:

    /// @brief Flags type data type.
    using FlagsType = std::uint32_t;

    using BodySet = std::unordered_set<const Body*>;
    using JointSet = std::unordered_set<const Joint*>;
    using ContactSet = std::unordered_set<const Contact*>;
    using FixtureQueue = std::vector<Fixture*>;
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
        Length minSeparation = std::numeric_limits<RealNum>::infinity() * Meter; ///< Minimum separation.
        Momentum maxIncImpulse = 0; ///< Maximum incremental impulse.
        body_count_t bodiesSlept = 0;
        contact_count_t contactsUpdated = 0;
        contact_count_t contactsSkipped = 0;
        bool solved = false; ///< Solved. <code>true</code> if position constraints solved, <code>false</code> otherwise.
        ts_iters_t positionIterations = 0; ///< Position iterations actually performed.
        ts_iters_t velocityIterations = 0; ///< Velocity iterations actually performed.
    };
    
    void CopyBodies(std::map<const Body*, Body*>& bodyMap,
                    std::map<const Fixture*, Fixture*>& fixtureMap,
                    SizedRange<World::Bodies::const_iterator> range);
    void CopyJoints(const std::map<const Body*, Body*>& bodyMap,
                    SizedRange<World::Joints::const_iterator> range);
    void CopyContacts(const std::map<const Body*, Body*>& bodyMap,
                      const std::map<const Fixture*, Fixture*>& fixtureMap,
                      SizedRange<World::Contacts::const_iterator> range);

    void InternalDestroy(Joint* joint);

    /// @brief Solves the step.
    /// @details Finds islands, integrates and solves constraints, solves position constraints.
    /// @note This may miss collisions involving fast moving bodies and allow them to tunnel through each other.
    RegStepStats SolveReg(const StepConf& conf);

    /// @brief Solves the given island (regularly).
    ///
    /// @details This:
    ///   1. Updates every island-body's sweep.pos0 to its sweep.pos1.
    ///   2. Updates every island-body's sweep.pos1 to the new "solved" position for it.
    ///   3. Updates every island-body's velocity to the new accelerated, dampened, and "solved" velocity for it.
    ///   4. Synchronizes every island-body's transform (by updating it to transform one of the body's sweep).
    ///   5. Reports to the listener (if non-null).
    ///
    /// @param step Time step information.
    /// @param island Island of bodies, contacts, and joints to solve for.
    ///
    /// @return Island solver results.
    ///
    IslandSolverResults SolveRegIsland(const StepConf& step, Island island);
    
    /// @brief Adds to the island based off of a given "seed" body.
    /// @post Contacts are listed in the island in the order that bodies list those contacts.
    /// @post Joints are listed the island in the order that bodies list those joints.
    void AddToIsland(Island& island, Body& seed,
                       Bodies::size_type& remNumBodies,
                       Contacts::size_type& remNumContacts,
                       Joints::size_type& remNumJoints);

    Bodies::size_type RemoveUnspeedablesFromIslanded(const std::vector<Body*>& bodies);

    /// @brief Solves the step using successive time of impact (TOI) events.
    /// @details Used for continuous physics.
    /// @note This is intended to detect and prevent the tunneling that the faster Solve method may miss.
    /// @param conf Time step configuration to use.
    ToiStepStats SolveTOI(const StepConf& conf);

    /// @brief Solves collisions for the given time of impact.
    ///
    /// @param step Time step to solve for.
    /// @param contact Contact.
    ///
    /// @note Precondition 1: there is no contact having a lower TOI in this time step that has not already been solved for.
    /// @note Precondition 2: there is not a lower TOI in the time step for which collisions have not already been processed.
    ///
    IslandSolverResults SolveTOI(const StepConf& step, Contact& contact);
    
    /// @brief Solves the time of impact for bodies 0 and 1 of the given island.
    ///
    /// @details This:
    ///   1. Updates pos0 of the sweeps of bodies 0 and 1.
    ///   2. Updates pos1 of the sweeps, the transforms, and the velocities of the other bodies in this island.
    ///
    /// @pre <code>island.m_bodies</code> contains at least two bodies, the first two of which are bodies 0 and 1.
    /// @pre <code>island.m_bodies</code> contains appropriate other bodies of the contacts of the two bodies.
    /// @pre <code>island.m_contacts</code> contains the contact that specified the two identified bodies.
    /// @pre <code>island.m_contacts</code> contains appropriate other contacts of the two bodies.
    ///
    /// @param step Time step information.
    /// @param island Island to do time of impact solving for.
    ///
    /// @return Island solver results.
    ///
    IslandSolverResults SolveTOI(const StepConf& step, Island& island);

    static void UpdateBody(Body& body, const Position& pos, const Velocity& vel);

    void ResetBodiesForSolveTOI();
    void ResetContactsForSolveTOI();
    void ResetContactsForSolveTOI(Body& body);

    struct ProcessContactsOutput
    {
        contact_count_t contactsUpdated = 0;
        contact_count_t contactsSkipped = 0;
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
    ProcessContactsOutput ProcessContactsForTOI(Island& island, Body& body, RealNum toi,
                                                const StepConf& conf);
    
    bool Add(Joint* j, Body* bodyA, Body* bodyB);

    bool Remove(Body& b);
    bool Remove(Joint& j);

    /// @brief Whether or not "step" is complete.
    /// @details The "step" is completed when there are no more TOI events for the current time step.
    /// @sa <code>SetStepComplete</code>.
    bool IsStepComplete() const noexcept;

    void SetStepComplete(bool value) noexcept;

    void SetAllowSleeping() noexcept;
    void UnsetAllowSleeping() noexcept;
    
    struct UpdateContactsStats
    {
        /// @brief Number of contacts ignored (because both bodies were asleep).
        contact_count_t ignored = 0;

        /// @brief Number of contacts updated.
        contact_count_t updated = 0;
        
        /// @brief Number of contacts skipped because they weren't marked as needing updating.
        contact_count_t skipped = 0;
    };
    
    struct DestroyContactsStats
    {
        contact_count_t ignored = 0;
        contact_count_t filteredOut = 0;
        contact_count_t notOverlapping = 0;
    };
    
    struct ContactToiData
    {
        std::vector<Contact*> contacts; ///< Contacts for which the time of impact is relavant.
        RealNum toi = std::numeric_limits<RealNum>::infinity(); ///< Time of impact (TOI) as a fractional value between 0 and 1.
    };

    struct UpdateContactsData
    {
        contact_count_t numAtMaxSubSteps = 0;
        contact_count_t numUpdatedTOI = 0; ///< # updated TOIs (made valid).
        contact_count_t numValidTOI = 0; ///< # already valid TOIs.
    
        using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
        using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
        using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
        
        dist_iter_type maxDistIters = 0;
        toi_iter_type maxToiIters = 0;
        root_iter_type maxRootIters = 0;
    };
    
    /// @brief Updates the contact times of impact.
    UpdateContactsData UpdateContactTOIs(const StepConf& conf);

    /// @brief Gets the soonest contact.
    /// @details This finds the contact with the lowest (soonest) time of impact.
    /// @return Contacts with the least time of impact and its time of impact, or null contact.
    ///  These contacts will all be enabled, not have sensors, be active, and impenetrable.
    ContactToiData GetSoonestContacts(const size_t reserveSize);

    bool HasNewFixtures() const noexcept;
    
    void UnsetNewFixtures() noexcept;
    
    /// @brief Finds new contacts.
    /// @details Finds and adds new valid contacts to the contacts container.
    /// @note The new contacts will all have overlapping AABBs.
    contact_count_t FindNewContacts();
    
    /// @brief Processes the narrow phase collision for the contact list.
    /// @details
    /// This finds and destroys the contacts that need filtering and no longer should collide or
    /// that no longer have AABB-based overlapping fixtures. Those contacts that persist and
    /// have active bodies (either or both) get their Update methods called with the current
    /// contact listener as its argument.
    /// Essentially this really just purges contacts that are no longer relevant.
    DestroyContactsStats DestroyContacts(Contacts& contacts);
    
    UpdateContactsStats UpdateContacts(Contacts& contacts, const StepConf& conf);
    
    bool ShouldCollide(const Fixture* fixtureA, const Fixture* fixtureB);

    /// @brief Destroys the given contact and removes it from its list.
    /// @details This updates the contact list, returns the memory to the allocator,
    ///   and decrements the contact manager's contact count.
    /// @param c Contact to destroy.
    void Destroy(Contact* c, Body* from);
    
    /// @brief Adds a contact for proxyA and proxyB if appropriate.
    /// @details Adds a new contact object to represent a contact between proxy A and proxy B if
    /// all of the following are true:
    ///   1. The bodies of the fixtures of the proxies are not the one and the same.
    ///   2. No contact already exists for these two proxies.
    ///   3. The bodies of the proxies should collide (according to Body::ShouldCollide).
    ///   4. The contact filter says the fixtures of the proxies should collide.
    ///   5. There exists a contact-create function for the pair of shapes of the proxies.
    /// @param proxyA Proxy A.
    /// @param proxyB Proxy B.
    /// @return <code>true</code> if a new contact was indeed added (and created), else <code>false</code>.
    /// @sa bool Body::ShouldCollide(const Body* other) const
    bool Add(const FixtureProxy& proxyA, const FixtureProxy& proxyB);
    
    void InternalDestroy(Contact* contact, Body* from = nullptr);
    bool Erase(Contact* contact);
    
    /// @brief Creates proxies for every child of the given fixture's shape.
    /// @note This sets the proxy count to the child count of the shape.
    void CreateProxies(Fixture& fixture, const Length aabbExtension);

    /// @brief Destroys the given fixture's proxies.
    /// @note This resets the proxy count to 0.
    void DestroyProxies(Fixture& fixture);

    /// @brief Touches each proxy of the given fixture.
    /// @note This sets things up so that pairs may be created for potentially new contacts.
    void InternalTouchProxies(Fixture& fixture) noexcept;

    child_count_t Synchronize(Fixture& fixture,
                              const Transformation xfm1, const Transformation xfm2,
                              const RealNum multiplier, const Length extension);

    contact_count_t Synchronize(Body& body,
                                const Transformation& xfm1, const Transformation& xfm2,
                                const RealNum multiplier, const Length aabbExtension);
    
    void CreateAndDestroyProxies(const StepConf& conf);
    void CreateAndDestroyProxies(Fixture& fixture, const StepConf& conf);
    
    PreStepStats::counter_type SynchronizeProxies(const StepConf& conf);

    bool IsIslanded(const Body* body);
    bool IsIslanded(const Contact* contact);
    bool IsIslanded(const Joint* joint);

    void SetIslanded(Body* body);
    void SetIslanded(Contact* contact);
    void SetIslanded(Joint* joint);

    void UnsetIslanded(Body* body);
    void UnsetIslanded(Contact* contact);
    void UnsetIslanded(Joint* joint);

    /******** Member variables. ********/
    
    BroadPhase m_broadPhase{BroadPhase::Conf{4096, 1024, 1024}}; ///< Broad phase data. 72-bytes.
    
    //ContactKeySet m_contactKeySet{100000};
    
    BodySet m_bodiesIslanded;
    ContactSet m_contactsIslanded;
    JointSet m_jointsIslanded;

    FixtureQueue m_fixturesForProxies;
    BodyQueue m_bodiesForProxies;

    ContactFilter m_defaultFilter; ///< Default contact filter. 8-bytes.
    
    Bodies m_bodies; ///< Body collection.
    Joints m_joints; ///< Joint collection.

    /// Container of contacts.
    /// @note In the "AddPair" stress-test, 401 bodies can have some 31000 contacts
    ///   during a given time step.
    Contacts m_contacts;

    LinearAcceleration2D m_gravity; ///< Gravity setting. 8-bytes.

    DestructionListener* m_destructionListener = nullptr; ///< Destruction listener. 8-bytes.
    
    ContactListener* m_contactListener = nullptr; ///< Contact listener. 8-bytes.
    
    ContactFilter* m_contactFilter = &m_defaultFilter; ///< Contact filter. 8-bytes.
    
    FlagsType m_flags = e_stepComplete;

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

class World::LockedError: public std::logic_error
{
public:
    LockedError():
    	std::logic_error{"world is locked"}
    {
        // Intentionally empty.
    }
};

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
    return SizedRange<World::Bodies::iterator>(m_bodies.begin(), m_bodies.end(),
                                               m_bodies.size());
}

inline SizedRange<World::Bodies::const_iterator> World::GetBodies() const noexcept
{
    return SizedRange<World::Bodies::const_iterator>(m_bodies.begin(), m_bodies.end(),
                                                     m_bodies.size());
}

inline SizedRange<World::Joints::const_iterator> World::GetJoints() const noexcept
{
    return SizedRange<World::Joints::const_iterator>(m_joints.begin(), m_joints.end(),
                                                     m_joints.size());
}

inline SizedRange<World::Joints::iterator> World::GetJoints() noexcept
{
    return SizedRange<World::Joints::iterator>(m_joints.begin(), m_joints.end(),
                                               m_joints.size());
}

inline SizedRange<World::Contacts::const_iterator> World::GetContacts() const noexcept
{
    return SizedRange<World::Contacts::const_iterator>(m_contacts.begin(), m_contacts.end(),
                                                       m_contacts.size());
}

inline LinearAcceleration2D World::GetGravity() const noexcept
{
    return m_gravity;
}

inline bool World::IsLocked() const noexcept
{
    return (m_flags & e_locked) == e_locked;
}

inline bool World::IsStepComplete() const noexcept
{
    return m_flags & e_stepComplete;
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
    return m_flags & e_substepping;
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
    return m_flags & e_newFixture;
}

inline void World::SetNewFixtures() noexcept
{
    m_flags |= World::e_newFixture;
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

inline AABB World::GetFatAABB(proxy_size_type proxyId) const
{
    return m_broadPhase.GetFatAABB(proxyId);
}

inline World::proxy_size_type World::GetProxyCount() const noexcept
{
    return m_broadPhase.GetProxyCount();
}

inline World::proxy_size_type World::GetTreeHeight() const noexcept
{
    return m_broadPhase.GetTreeHeight();
}

inline World::proxy_size_type World::GetTreeBalance() const
{
    return m_broadPhase.GetTreeBalance();
}

inline RealNum World::GetTreeQuality() const
{
    return m_broadPhase.GetTreeQuality();
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
    return !m_contactFilter || m_contactFilter->ShouldCollide(fixtureA, fixtureB);
}

inline bool World::IsIslanded(const Body* key)
{
    return m_bodiesIslanded.count(key) != 0;
}

inline bool World::IsIslanded(const Contact* key)
{
    return m_contactsIslanded.count(key) != 0;
}

inline bool World::IsIslanded(const Joint* key)
{
    return m_jointsIslanded.count(key) != 0;
}

inline void World::SetIslanded(Body* key)
{
    m_bodiesIslanded.insert(key);
}

inline void World::SetIslanded(Contact* key)
{
    m_contactsIslanded.insert(key);
}

inline void World::SetIslanded(Joint* key)
{
    m_jointsIslanded.insert(key);
}

inline void World::UnsetIslanded(Body* key)
{
    m_bodiesIslanded.erase(key);
}

inline void World::UnsetIslanded(Contact* key)
{
    m_contactsIslanded.erase(key);
}

inline void World::UnsetIslanded(Joint* key)
{
    m_jointsIslanded.erase(key);
}

// Free functions.

/// Gets the body count in the given world.
/// @return 0 or higher.
inline body_count_t GetBodyCount(const World& world) noexcept
{
    return static_cast<body_count_t>(world.GetBodies().size());
}

/// Gets the count of joints in the given world.
/// @return 0 or higher.
inline joint_count_t GetJointCount(const World& world) noexcept
{
    return static_cast<joint_count_t>(world.GetJoints().size());
}

/// Gets the count of contacts in the given world.
/// @note Not all contacts are for shapes that are actually touching. Some contacts are for
///   shapes which merely have overlapping AABBs.
/// @return 0 or higher.
inline contact_count_t GetContactCount(const World& world) noexcept
{
    return static_cast<contact_count_t>(world.GetContacts().size());
}

contact_count_t GetTouchingCount(const World& world) noexcept;

/// Steps the world ahead by a given time amount.
///
/// @details
/// Performs position and velocity updating, sleeping of non-moving bodies, updating
/// of the contacts, and notifying the contact listener of begin-contact, end-contact,
/// pre-solve, and post-solve events.
/// If the given velocity and position iterations are more than zero,
/// this method also respectively performs velocity and position resolution of the contacting bodies.
///
/// @note While body velocities are updated accordingly (per the sum of forces acting on them),
/// body positions (barring any collisions) are updated as if they had moved the entire time step
/// at those resulting velocities.
/// In other words, a body initially at p0 going v0 fast with a sum acceleration of a,
/// after time t and barring any collisions,
/// will have a new velocity (v1) of v0 + (a * t) and a new position (p1) of p0 + v1 * t.
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
/// @param timeStep Amount of time to simulate (in seconds). This should not vary.
/// @param velocityIterations Number of iterations for the velocity constraint solver.
/// @param positionIterations Number of iterations for the position constraint solver.
///   The position constraint solver resolves the positions of bodies that overlap.
///
StepStats Step(World& world, Time timeStep,
               World::ts_iters_type velocityIterations = 8, World::ts_iters_type positionIterations = 3);

/// Gets the count of fixtures in the given world.
size_t GetFixtureCount(const World& world) noexcept;

/// Gets the count of unique shapes in the given world.
size_t GetShapeCount(const World& world) noexcept;

/// Gets the count of awake bodies in the given world.
size_t GetAwakeCount(const World& world) noexcept;

/// Awakens all of the bodies in the given world.
/// @details Calls all of the world's bodies' <code>SetAwake</code> method.
/// @return Sum total of calls to bodies' <code>SetAwake</code> method that returned true.
/// @sa Body::SetAwake.
size_t Awaken(World& world) noexcept;

/// Clears forces.
/// @details
/// Manually clear the force buffer on all bodies.
void ClearForces(World& world) noexcept;

bool IsActive(const Contact& contact) noexcept;

} // namespace box2d

#endif
