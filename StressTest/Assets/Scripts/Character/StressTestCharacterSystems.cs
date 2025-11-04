using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Entities;
using Unity.Physics;
using Unity.Transforms;
using Unity.CharacterController;

[UpdateInGroup(typeof(KinematicCharacterPhysicsUpdateGroup))]
[BurstCompile]
public partial struct StressTestCharacterPhysicsUpdateSystem : ISystem
{
    EntityQuery m_CharacterQuery;
    StressTestCharacterUpdateContext m_Context;
    KinematicCharacterUpdateContext m_BaseContext;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<StressTestManagerSystem.Singleton>();
        m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
            .WithAll<
                StressTestCharacterComponent,
                StressTestCharacterControl>()
            .Build(ref state);

        m_Context = new StressTestCharacterUpdateContext();
        m_Context.OnSystemCreate(ref state);
        m_BaseContext = new KinematicCharacterUpdateContext();
        m_BaseContext.OnSystemCreate(ref state);

        state.RequireForUpdate(m_CharacterQuery);
        state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        if (!SystemAPI.HasSingleton<StressTestManagerSystem.Singleton>())
            return;

        bool multithreaded = SystemAPI.GetSingleton<StressTestManagerSystem.Singleton>().Multithreaded;

        m_Context.OnSystemUpdate(ref state);
        m_BaseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

        StressTestCharacterPhysicsUpdateJob job = new StressTestCharacterPhysicsUpdateJob
        {
            Context = m_Context,
            BaseContext = m_BaseContext,
        };
        if (multithreaded)
        {
            job.ScheduleParallel();
        }
        else
        {
            job.Schedule();
        }
    }

    [BurstCompile]
    public partial struct StressTestCharacterPhysicsUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public StressTestCharacterUpdateContext Context;
        public KinematicCharacterUpdateContext BaseContext;

        void Execute(
            Entity entity,
            RefRW<LocalTransform> localTransform,
            RefRW<KinematicCharacterProperties> characterProperties,
            RefRW<KinematicCharacterBody> characterBody,
            RefRW<PhysicsCollider> physicsCollider,
            RefRW<StressTestCharacterComponent> characterComponent,
            RefRW<StressTestCharacterControl> characterControl,
            DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
        {
            var characterProcessor = new StressTestCharacterProcessor()
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
                CharacterControl = characterControl
            };

            characterProcessor.PhysicsUpdate(ref Context, ref BaseContext);
        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            BaseContext.EnsureCreationOfTmpCollections();
            return true;
        }

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted) { }
    }
}

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(TransformSystemGroup))]
[BurstCompile]
public partial struct StressTestCharacterVariableUpdateSystem : ISystem
{
    EntityQuery m_CharacterQuery;
    StressTestCharacterUpdateContext m_Context;
    KinematicCharacterUpdateContext m_BaseContext;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
            .WithAll<
                StressTestCharacterComponent,
                StressTestCharacterControl>()
            .Build(ref state);

        m_Context = new StressTestCharacterUpdateContext();
        m_Context.OnSystemCreate(ref state);
        m_BaseContext = new KinematicCharacterUpdateContext();
        m_BaseContext.OnSystemCreate(ref state);

        state.RequireForUpdate(m_CharacterQuery);
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        if (!SystemAPI.HasSingleton<StressTestManagerSystem.Singleton>())
            return;

        bool multithreaded = SystemAPI.GetSingleton<StressTestManagerSystem.Singleton>().Multithreaded;

        m_Context.OnSystemUpdate(ref state);
        m_BaseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

        StressTestCharacterVariableUpdateJob job = new StressTestCharacterVariableUpdateJob
        {
            Context = m_Context,
            BaseContext = m_BaseContext,
        };
        if (multithreaded)
        {
            job.ScheduleParallel();
        }
        else
        {
            job.Schedule();
        }
    }

    [BurstCompile]
    public partial struct StressTestCharacterVariableUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public StressTestCharacterUpdateContext Context;
        public KinematicCharacterUpdateContext BaseContext;

        void Execute(
            Entity entity,
            RefRW<LocalTransform> localTransform,
            RefRW<KinematicCharacterProperties> characterProperties,
            RefRW<KinematicCharacterBody> characterBody,
            RefRW<PhysicsCollider> physicsCollider,
            RefRW<StressTestCharacterComponent> characterComponent,
            RefRW<StressTestCharacterControl> characterControl,
            DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
        {
            var characterProcessor = new StressTestCharacterProcessor()
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
                CharacterControl = characterControl
            };

            characterProcessor.VariableUpdate(ref Context, ref BaseContext);
        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            BaseContext.EnsureCreationOfTmpCollections();
            return true;
        }

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted) { }
    }
}
