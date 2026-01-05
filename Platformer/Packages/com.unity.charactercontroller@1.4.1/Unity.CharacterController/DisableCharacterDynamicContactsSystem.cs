using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;

namespace Unity.CharacterController
{
    /// <summary>
    /// Singleton that enables the character dynamic contacts filtering job to run
    /// </summary>
    public struct DisableCharacterDynamicContacts : IComponentData
    { }

    /// <summary>
    /// 现象：KCC 是通过代码改坐标移动的（Kinematic），但你身上挂了一个 Collider。
    ///     如果你撞到了一个木箱子（Dynamic Rigidbody），物理引擎会觉得“两个刚体重叠了！”，然后试图用巨大的力把箱子弹飞，或者把你弹飞。
    ///     
    /// 解决：这个 System 实际上是一个 "过滤器"。
    ///     它在物理引擎计算碰撞之前运行。
    ///     它遍历所有的碰撞对（Contacts）。
    ///     如果发现：A 是 Character && B 是 Dynamic Body。
    ///     它会把这个碰撞对 Disable 掉（JacobianFlags.Disabled）。
    ///     
    /// 结果：物理引擎完全忽略你和箱子的碰撞。
    /// System scheduling a job that disables contacts between dynamic characters and dynamic colliders
    /// </summary>
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateContactsGroup))]
    [UpdateBefore(typeof(PhysicsCreateJacobiansGroup))]
    [RequireMatchingQueriesForUpdate]
    public partial struct DisableCharacterDynamicContactsSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // Create singleton
            Entity singleton = state.EntityManager.CreateEntity();
            state.EntityManager.AddComponentData(singleton, new DisableCharacterDynamicContacts());

            EntityQuery characterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder().Build(ref state);

            state.RequireForUpdate(characterQuery);
            state.RequireForUpdate<DisableCharacterDynamicContacts>();
            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<SimulationSingleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            PhysicsWorld physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().ValueRW.PhysicsWorld;
            SimulationSingleton simulationSingleton = SystemAPI.GetSingleton<SimulationSingleton>();

            if (physicsWorld.Bodies.Length > 0)
            {
                DisableCharacterDynamicContactsJob job = new DisableCharacterDynamicContactsJob
                {
                    PhysicsWorld = physicsWorld,
                    StoredKinematicCharacterDataLookup = SystemAPI.GetComponentLookup<StoredKinematicCharacterData>(true),
                };
                state.Dependency = job.Schedule(simulationSingleton, ref physicsWorld, state.Dependency);
            }
        }

        /// <summary>
        /// Disables body pairs between dynamic characters and dynamic bodies
        /// </summary>
        [BurstCompile]
        public struct DisableCharacterDynamicContactsJob : IContactsJob
        {
            /// <summary>
            /// The physics world that the characters belong to
            /// </summary>
            [ReadOnly]
            public PhysicsWorld PhysicsWorld;
            /// <summary>
            /// Lookup for <see cref="StoredKinematicCharacterData"/>
            /// </summary>
            [ReadOnly]
            public ComponentLookup<StoredKinematicCharacterData> StoredKinematicCharacterDataLookup;

            public unsafe void Execute(ref ModifiableContactHeader manifold, ref ModifiableContactPoint contact)
            {
                // Both should be non-static
                if (manifold.BodyIndexA < PhysicsWorld.NumDynamicBodies && manifold.BodyIndexB < PhysicsWorld.NumDynamicBodies)
                {
                    bool aIsKinematic = PhysicsWorld.MotionVelocities[manifold.BodyIndexA].IsKinematic;
                    bool bIsKinematic = PhysicsWorld.MotionVelocities[manifold.BodyIndexB].IsKinematic;

                    // One should be kinematic and the other should be dynamic
                    if (aIsKinematic != bIsKinematic)
                    {
                        Entity kinematicEntity;
                        int dynamicBodyIndex;
                        ColliderKey dynamicBodyColliderKey;
                        if (aIsKinematic)
                        {
                            kinematicEntity = manifold.EntityA;
                            dynamicBodyIndex = manifold.BodyIndexB;
                            dynamicBodyColliderKey = manifold.ColliderKeyB;
                        }
                        else
                        {
                            kinematicEntity = manifold.EntityB;
                            dynamicBodyIndex = manifold.BodyIndexA;
                            dynamicBodyColliderKey = manifold.ColliderKeyA;
                        }

                        // Disable only if dynamic entity is collidable
                        CollisionResponsePolicy dynamicBodyCollisionResponse = PhysicsWorld.Bodies[dynamicBodyIndex].Collider.Value.GetCollisionResponse(dynamicBodyColliderKey);
                        if (dynamicBodyCollisionResponse == CollisionResponsePolicy.Collide || dynamicBodyCollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                        {
                            // Disable only if kinematic entity is character and is simulated dynamic
                            if (StoredKinematicCharacterDataLookup.TryGetComponent(kinematicEntity, out StoredKinematicCharacterData characterData) &&
                                characterData.SimulateDynamicBody)
                            {
                                manifold.JacobianFlags |= JacobianFlags.Disabled;
                            }
                        }
                    }
                }
            }
        }
    }
}
