using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Entities;
using Unity.Physics;
using Unity.Transforms;
using Unity.CharacterController;

[UpdateInGroup(typeof(KinematicCharacterPhysicsUpdateGroup))]
[BurstCompile]
public partial struct BasicCharacterPhysicsUpdateSystem : ISystem
{
    EntityQuery m_CharacterQuery;
    BasicCharacterUpdateContext m_Context;
    KinematicCharacterUpdateContext m_BaseContext;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
            .WithAll<BasicCharacterComponent, BasicCharacterControl>()
            .Build(ref state);

        m_Context = new BasicCharacterUpdateContext();
        m_Context.OnSystemCreate(ref state);
        m_BaseContext = new KinematicCharacterUpdateContext();
        m_BaseContext.OnSystemCreate(ref state);

        state.RequireForUpdate(m_CharacterQuery);
        state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        m_Context.OnSystemUpdate(ref state);
        m_BaseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());
        
        BasicCharacterPhysicsUpdateJob job = new BasicCharacterPhysicsUpdateJob
        {
            Context = m_Context,
            BaseContext = m_BaseContext,
        };
        job.ScheduleParallel();
    }

    [BurstCompile]
    public partial struct BasicCharacterPhysicsUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public BasicCharacterUpdateContext Context;
        public KinematicCharacterUpdateContext BaseContext;
    
        public void Execute(
            Entity entity,
            RefRW<LocalTransform> localTransform,
            RefRW<KinematicCharacterProperties> characterProperties,
            RefRW<KinematicCharacterBody> characterBody,
            RefRW<PhysicsCollider> physicsCollider,
            RefRW<BasicCharacterComponent> characterComponent,
            RefRW<BasicCharacterControl> characterControl,
            DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
        {
            var characterProcessor = new BasicCharacterProcessor()
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

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
        { }
    }
}

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(BasicPlayerVariableStepControlSystem))]
[UpdateBefore(typeof(TransformSystemGroup))]
[BurstCompile]
public partial struct BasicCharacterVariableUpdateSystem : ISystem
{
    EntityQuery m_CharacterQuery;
    BasicCharacterUpdateContext m_Context;
    KinematicCharacterUpdateContext m_BaseContext;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
            .WithAll<BasicCharacterComponent, BasicCharacterControl>()
            .Build(ref state);

        m_Context = new BasicCharacterUpdateContext();
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
        
        BasicCharacterVariableUpdateJob job = new BasicCharacterVariableUpdateJob
        {
            Context = m_Context,
            BaseContext = m_BaseContext,
        };
        job.ScheduleParallel();
    }

    [BurstCompile]
    public partial struct BasicCharacterVariableUpdateJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public BasicCharacterUpdateContext Context;
        public KinematicCharacterUpdateContext BaseContext;
    
        public void Execute(
            Entity entity,
            RefRW<LocalTransform> localTransform,
            RefRW<KinematicCharacterProperties> characterProperties,
            RefRW<KinematicCharacterBody> characterBody,
            RefRW<PhysicsCollider> physicsCollider,
            RefRW<BasicCharacterComponent> characterComponent,
            RefRW<BasicCharacterControl> characterControl,
            DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits)
        {
            var characterProcessor = new BasicCharacterProcessor()
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

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
        { }
    }
}
