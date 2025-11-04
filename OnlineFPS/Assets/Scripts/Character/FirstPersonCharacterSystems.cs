using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using Unity.CharacterController;
using Unity.NetCode;
using UnityEngine;

namespace OnlineFPS
{
    [UpdateInGroup(typeof(PredictedSimulationSystemGroup), OrderFirst = true)]
    [UpdateBefore(typeof(PredictedFixedStepSimulationSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.ThinClientSimulation |
        WorldSystemFilterFlags.ServerSimulation)]
    [BurstCompile]
    public partial struct BuildCharacterPredictedRotationSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<LocalTransform, FirstPersonCharacterComponent>()
                .Build());
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            BuildCharacterPredictedRotationJob job = new BuildCharacterPredictedRotationJob();
            state.Dependency = job.Schedule(state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct BuildCharacterPredictedRotationJob : IJobEntity
        {
            void Execute(ref LocalTransform localTransform, in FirstPersonCharacterComponent characterComponent)
            {
                FirstPersonCharacterUtilities.ComputeRotationFromYAngleAndUp(characterComponent.CharacterYDegrees,
                    math.up(), out quaternion tmpRotation);
                localTransform.Rotation = tmpRotation;
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(TransformSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
    [BurstCompile]
    public partial struct BuildCharacterInterpolatedRotationSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<LocalTransform, FirstPersonCharacterComponent>()
                .Build());
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            BuildCharacterInterpolatedRotationJob job = new BuildCharacterInterpolatedRotationJob
            {
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            };
            state.Dependency = job.Schedule(state.Dependency);
        }

        [BurstCompile]
        [WithNone(typeof(GhostOwnerIsLocal))]
        public partial struct BuildCharacterInterpolatedRotationJob : IJobEntity
        {
            public ComponentLookup<LocalTransform> LocalTransformLookup;

            void Execute(Entity entity, in FirstPersonCharacterComponent characterComponent)
            {
                if (LocalTransformLookup.TryGetComponent(entity, out LocalTransform characterLocalTransform))
                {
                    FirstPersonCharacterUtilities.ComputeRotationFromYAngleAndUp(characterComponent.CharacterYDegrees,
                        math.up(), out quaternion tmpRotation);
                    characterLocalTransform.Rotation = tmpRotation;
                    LocalTransformLookup[entity] = characterLocalTransform;

                    if (LocalTransformLookup.TryGetComponent(characterComponent.ViewEntity,
                            out LocalTransform viewLocalTransform))
                    {
                        viewLocalTransform.Rotation =
                            FirstPersonCharacterUtilities.CalculateLocalViewRotation(
                                characterComponent.ViewPitchDegrees, 0f);
                        LocalTransformLookup[characterComponent.ViewEntity] = viewLocalTransform;
                    }
                }
            }
        }
    }

    [UpdateInGroup(typeof(KinematicCharacterPhysicsUpdateGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.ThinClientSimulation |
        WorldSystemFilterFlags.ServerSimulation)]
    [BurstCompile]
    public partial struct FirstPersonCharacterPhysicsUpdateSystem : ISystem
    {
        EntityQuery m_CharacterQuery;
        FirstPersonCharacterUpdateContext m_Context;
        KinematicCharacterUpdateContext m_BaseContext;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
                .WithAll<
                    FirstPersonCharacterComponent,
                    FirstPersonCharacterControl,
                    ActiveWeapon>()
                .Build(ref state);

            m_Context = new FirstPersonCharacterUpdateContext();
            m_Context.OnSystemCreate(ref state);
            m_BaseContext = new KinematicCharacterUpdateContext();
            m_BaseContext.OnSystemCreate(ref state);

            state.RequireForUpdate(m_CharacterQuery);
            state.RequireForUpdate<NetworkTime>();
            state.RequireForUpdate<PhysicsWorldSingleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.HasSingleton<NetworkTime>())
                return;

            m_Context.OnSystemUpdate(ref state);
            m_BaseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

            FirstPersonCharacterPhysicsUpdateJob job = new FirstPersonCharacterPhysicsUpdateJob
            {
                Context = m_Context,
                BaseContext = m_BaseContext,
            };
            state.Dependency = job.Schedule(state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        [WithDisabled(typeof(DelayedDespawn))]
        public partial struct FirstPersonCharacterPhysicsUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
        {
            public FirstPersonCharacterUpdateContext Context;
            public KinematicCharacterUpdateContext BaseContext;

            void Execute(
                Entity entity,
                RefRW<LocalTransform> localTransform,
                RefRW<KinematicCharacterProperties> characterProperties,
                RefRW<KinematicCharacterBody> characterBody,
                RefRW<PhysicsCollider> physicsCollider,
                RefRW<FirstPersonCharacterComponent> characterComponent,
                RefRW<FirstPersonCharacterControl> characterControl,
                RefRW<ActiveWeapon> activeWeapon,
                DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
                DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
                DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
                DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
            {
                var characterProcessor = new FirstPersonCharacterProcessor()
                {
                    CharacterDataAccess = new KinematicCharacterDataAccess(

                        entity,
                        localTransform,
                        characterProperties,
                        characterBody,
                        physicsCollider,
                        characterHitsBuffer,
                        statefulHitsBuffer,
                        deferredImpulsesBuffer,
                        velocityProjectionHits
                    ),
                    CharacterComponent = characterComponent,
                    CharacterControl = characterControl,
                    ActiveWeapon = activeWeapon
                };

                characterProcessor.PhysicsUpdate(ref Context, ref BaseContext);
            }

            public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                BaseContext.EnsureCreationOfTmpCollections();
                return true;
            }

            public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask, bool chunkWasExecuted) { }
        }
    }

    [UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
    [UpdateAfter(typeof(PredictedFixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(FirstPersonPlayerVariableStepControlSystem))]
    [UpdateAfter(typeof(BuildCharacterPredictedRotationSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation | WorldSystemFilterFlags.ThinClientSimulation |
        WorldSystemFilterFlags.ServerSimulation)]
    [BurstCompile]
    public partial struct FirstPersonCharacterVariableUpdateSystem : ISystem
    {
        EntityQuery m_CharacterQuery;
        FirstPersonCharacterUpdateContext m_Context;
        KinematicCharacterUpdateContext m_BaseContext;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
                .WithAll<
                    FirstPersonCharacterComponent,
                    FirstPersonCharacterControl,
                    ActiveWeapon>()
                .Build(ref state);

            m_Context = new FirstPersonCharacterUpdateContext();
            m_Context.OnSystemCreate(ref state);
            m_BaseContext = new KinematicCharacterUpdateContext();
            m_BaseContext.OnSystemCreate(ref state);

            state.RequireForUpdate(m_CharacterQuery);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            m_Context.OnSystemUpdate(ref state);
            m_BaseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

            FirstPersonCharacterVariableUpdateJob variableUpdateJob = new FirstPersonCharacterVariableUpdateJob
            {
                Context = m_Context,
                BaseContext = m_BaseContext,
            };
            state.Dependency = variableUpdateJob.Schedule(state.Dependency);

            FirstPersonCharacterViewJob viewJob = new FirstPersonCharacterViewJob
            {
                FirstPersonCharacterLookup = SystemAPI.GetComponentLookup<FirstPersonCharacterComponent>(true),
            };
            state.Dependency = viewJob.Schedule(state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        [WithDisabled(typeof(DelayedDespawn))]
        public partial struct FirstPersonCharacterVariableUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
        {
            public FirstPersonCharacterUpdateContext Context;
            public KinematicCharacterUpdateContext BaseContext;

            void Execute(
                Entity entity,
                RefRW<LocalTransform> localTransform,
                RefRW<KinematicCharacterProperties> characterProperties,
                RefRW<KinematicCharacterBody> characterBody,
                RefRW<PhysicsCollider> physicsCollider,
                RefRW<FirstPersonCharacterComponent> characterComponent,
                RefRW<FirstPersonCharacterControl> characterControl,
                RefRW<ActiveWeapon> activeWeapon,
                DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
                DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
                DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
                DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
            {
                var characterProcessor = new FirstPersonCharacterProcessor()
                {
                    CharacterDataAccess = new KinematicCharacterDataAccess(

                        entity,
                        localTransform,
                        characterProperties,
                        characterBody,
                        physicsCollider,
                        characterHitsBuffer,
                        statefulHitsBuffer,
                        deferredImpulsesBuffer,
                        velocityProjectionHits
                    ),
                    CharacterComponent = characterComponent,
                    CharacterControl = characterControl,
                    ActiveWeapon = activeWeapon
                };

                characterProcessor.VariableUpdate(ref Context, ref BaseContext);
            }

            public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                BaseContext.EnsureCreationOfTmpCollections();
                return true;
            }

            public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask, bool chunkWasExecuted) { }
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct FirstPersonCharacterViewJob : IJobEntity
        {
            [ReadOnly]
            public ComponentLookup<FirstPersonCharacterComponent> FirstPersonCharacterLookup;

            void Execute(ref LocalTransform localTransform, in FirstPersonCharacterView characterView)
            {
                if (FirstPersonCharacterLookup.TryGetComponent(characterView.CharacterEntity,
                        out FirstPersonCharacterComponent character))
                {
                    localTransform.Rotation = character.ViewLocalRotation;
                }
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(TransformSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
    [BurstCompile]
    public partial struct FirstPersonCharacterPresentationOnlySystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            FirstPersonCharacterViewRollJob viewJob = new FirstPersonCharacterViewRollJob
            {
                DeltaTime = SystemAPI.Time.DeltaTime,
                FirstPersonCharacterViewLookup = SystemAPI.GetComponentLookup<FirstPersonCharacterView>(true),
                LocalTransformLookup = SystemAPI.GetComponentLookup<LocalTransform>(false),
            };
            state.Dependency = viewJob.Schedule(state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        [WithDisabled(typeof(DelayedDespawn))]
        public partial struct FirstPersonCharacterViewRollJob : IJobEntity
        {
            public float DeltaTime;
            [ReadOnly]
            public ComponentLookup<FirstPersonCharacterView> FirstPersonCharacterViewLookup;
            public ComponentLookup<LocalTransform> LocalTransformLookup;

            void Execute(Entity entity, ref FirstPersonCharacterComponent characterComponent,
                in KinematicCharacterBody characterBody)
            {
                if (LocalTransformLookup.TryGetComponent(entity, out LocalTransform characterTransform) &&
                    LocalTransformLookup.TryGetComponent(characterComponent.ViewEntity,
                        out LocalTransform viewTransform) &&
                    FirstPersonCharacterViewLookup.TryGetComponent(characterComponent.ViewEntity,
                        out FirstPersonCharacterView characterView))
                {
                    // View roll angles
                    {
                        float3 characterRight = MathUtilities.GetRightFromRotation(characterTransform.Rotation);
                        float characterMaxSpeed = characterBody.IsGrounded
                            ? characterComponent.GroundMaxSpeed
                            : characterComponent.AirMaxSpeed;
                        float3 characterLateralVelocity =
                            math.projectsafe(characterBody.RelativeVelocity, characterRight);
                        float characterLateralVelocityRatio =
                            math.clamp(math.length(characterLateralVelocity) / characterMaxSpeed, 0f, 1f);
                        bool velocityIsRight = math.dot(characterBody.RelativeVelocity, characterRight) > 0f;
                        float targetTiltAngle = math.lerp(0f, characterComponent.ViewRollAmount,
                            characterLateralVelocityRatio);
                        targetTiltAngle = velocityIsRight ? -targetTiltAngle : targetTiltAngle;
                        characterComponent.ViewRollDegrees = math.lerp(characterComponent.ViewRollDegrees,
                            targetTiltAngle,
                            MathUtilities.GetSharpnessInterpolant(characterComponent.ViewRollSharpness, DeltaTime));
                    }

                    // Calculate view local rotation
                    characterComponent.ViewLocalRotation =
                        FirstPersonCharacterUtilities.CalculateLocalViewRotation(characterComponent.ViewPitchDegrees,
                            characterComponent.ViewRollDegrees);

                    // Set view local transform
                    viewTransform.Rotation = characterComponent.ViewLocalRotation;
                    LocalTransformLookup[characterComponent.ViewEntity] = viewTransform;
                }
            }
        }
    }
}
