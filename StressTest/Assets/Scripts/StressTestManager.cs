using System;
using System.Collections.Generic;
using Unity.Burst;
using UnityEngine;
using UnityEngine.UI;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Physics;
using Unity.CharacterController;

public class StressTestManager : MonoBehaviour
{
    [Header("References")]
    public Camera Camera;
    public Button SpawnButton;
    public InputField SpawnCountInputField;
    public Dropdown EnvironmentPrefabDropdown;
    public Toggle MultithreadedToggle;
    public Toggle PhysicsStepToggle;
    public Toggle RenderingToggle;
    public Toggle StepHandlingToggle;
    public Toggle SlopeChangesToggle;
    public Toggle ProjectVelocityOnInitialOverlapsToggle;
    public Toggle StatefulHitsToggle;
    public Toggle SimulateDynamicToggle;
    public Toggle SaveRestoreStateToggle;
    public Toggle EnhancedGroundPrecision;

    bool m_HasInitializedFromEntities;
    World m_World;
    EntityManager m_EntityManager;
    EntityQuery m_ManagerSingletonQuery;

    void Start()
    {
        m_World = World.DefaultGameObjectInjectionWorld;
        m_EntityManager = m_World.EntityManager;
        m_ManagerSingletonQuery = new EntityQueryBuilder(Allocator.Temp)
            .WithAll<
                StressTestManagerSystem.EnvironmentPrefabs>()
            .WithAllRW<
                StressTestManagerSystem.Singleton,
                StressTestManagerSystem.Event>()
            .Build(m_EntityManager);
        
        // No fixedUpdate, for performance measuring
        m_World.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>().RateManager = null;

        // Subscribe UI
        SpawnButton.onClick.AddListener(SpawnCharacters);
        EnvironmentPrefabDropdown.onValueChanged.AddListener(SwitchEnvironment);
        MultithreadedToggle.onValueChanged.AddListener(SetMultithreaded);
        PhysicsStepToggle.onValueChanged.AddListener(SetPhysicsStep);
        
        RenderingToggle.onValueChanged.AddListener(SetRendering);
        
        StepHandlingToggle.onValueChanged.AddListener(SetStepHandling);
        SlopeChangesToggle.onValueChanged.AddListener(SetSlopeChanges);
        ProjectVelocityOnInitialOverlapsToggle.onValueChanged.AddListener(SetProjectVelocityOnOverlaps);
        StatefulHitsToggle.onValueChanged.AddListener(SetStatefulHits);
        SimulateDynamicToggle.onValueChanged.AddListener(SetSimulateDynamicBody);
        SaveRestoreStateToggle.onValueChanged.AddListener(SetSaveRestoreState);
        EnhancedGroundPrecision.onValueChanged.AddListener(SetEnhancedGroundPrecision);
    }

    public bool TryGetManagerSingleton(out Entity entity)
    {
        if (m_ManagerSingletonQuery.HasSingleton<StressTestManagerSystem.Singleton>())
        {
            entity = m_ManagerSingletonQuery.GetSingletonEntity();
            return true;
        }

        entity = default;
        return false;
    }

    public ref StressTestManagerSystem.Singleton GetManagerSingletonRef()
    {
        return ref m_ManagerSingletonQuery.GetSingletonRW<StressTestManagerSystem.Singleton>().ValueRW;
    }

    public DynamicBuffer<StressTestManagerSystem.EnvironmentPrefabs> GetManagerSingletonEnvPrefabsBuffer()
    {
        return m_ManagerSingletonQuery.GetSingletonBuffer<StressTestManagerSystem.EnvironmentPrefabs>();
    }

    public DynamicBuffer<StressTestManagerSystem.Event> GetManagerSingletonEventsBuffer()
    {
        return m_ManagerSingletonQuery.GetSingletonBuffer<StressTestManagerSystem.Event>();
    }
    
    void Update()
    {
        if (!m_HasInitializedFromEntities)
        {
            if (TryGetManagerSingleton(out Entity entity))
            {
                DynamicBuffer<StressTestManagerSystem.EnvironmentPrefabs> environmentPrefabs = GetManagerSingletonEnvPrefabsBuffer();
                
                // Add environment prefabs to dropdown choices
                for (int i = 0; i < environmentPrefabs.Length; i++)
                {
                    EnvironmentPrefabDropdown.AddOptions(new List<Dropdown.OptionData>
                    {
                        new(m_EntityManager.GetComponentData<GameObjectName>(environmentPrefabs[i].Prefab).Value.Value),
                    });
                }

                // Initial setup
                SwitchEnvironment(EnvironmentPrefabDropdown.value);
                SetMultithreaded(MultithreadedToggle.isOn);
                SetRendering(RenderingToggle.isOn);

                ApplyCharacterSettings();

                m_HasInitializedFromEntities = true;
            }
        }
    }

    public void SpawnCharacters()
    {
        if (int.TryParse(SpawnCountInputField.text, out int spawnCount))
        {
            ref var singleton = ref GetManagerSingletonRef();
            singleton.CharacterCount = spawnCount;

            var eventsBuffer = GetManagerSingletonEventsBuffer();
            eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.SpawnCharacters });
        
            ApplyCharacterSettings();
        }
    }

    private void ApplyCharacterSettings()
    {
        SetStepHandling(StepHandlingToggle.isOn);
        SetSlopeChanges(SlopeChangesToggle.isOn);
        SetProjectVelocityOnOverlaps(ProjectVelocityOnInitialOverlapsToggle.isOn);
        SetStatefulHits(StatefulHitsToggle.isOn);
        SetSimulateDynamicBody(SimulateDynamicToggle.isOn);
        SetSaveRestoreState(SaveRestoreStateToggle.isOn);
    }

    public void SwitchEnvironment(int index)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.EnvironmentIndex = index;
        
        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.SpawnEnvironment });
    }

    public void SetMultithreaded(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.Multithreaded = active;
    }

    public void SetPhysicsStep(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.PhysicsStep = active;
        
        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplyPhysicsStep });
    }

    public void SetRendering(bool active)
    {
        Camera.enabled = active;
    }

    public void SetStepHandling(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.StepHandling = active;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplyStepHandling });
    }

    public void SetSlopeChanges(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.SlopeChanges = active;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplySlopeChanges });
    }

    public void SetProjectVelocityOnOverlaps(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.ProjectVelocityOnOverlaps = active;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplyProjectVelocityOnOverlaps });
    }

    public void SetStatefulHits(bool active)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.StatefulHits = active;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplyStatefulHits });
    }

    public unsafe void SetSimulateDynamicBody(bool value)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.SimulateDynamic = value;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplySimulateDynamic });
    }

    public void SetSaveRestoreState(bool value)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.SaveRestoreState = value;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplySaveRestoreState });
    }

    public void SetEnhancedGroundPrecision(bool value)
    {
        ref var singleton = ref GetManagerSingletonRef();
        singleton.EnhancedGroundPrecision = value;

        var eventsBuffer = GetManagerSingletonEventsBuffer();
        eventsBuffer.Add(new StressTestManagerSystem.Event { Type = StressTestManagerSystem.EventType.ApplyEnhancedGroundPrecision });
    }
}

[BurstCompile]
public partial struct StressTestManagerSystem : ISystem
{
    public enum EventType
    {
        SpawnCharacters,
        SpawnEnvironment,
        
        ApplyPhysicsStep,
        
        ApplyStepHandling,
        ApplySlopeChanges,
        ApplyProjectVelocityOnOverlaps,
        ApplyStatefulHits,
        ApplySimulateDynamic,
        ApplySaveRestoreState,
        ApplyEnhancedGroundPrecision,
    }
    
    public struct Singleton : IComponentData
    {
        public Entity CharacterPrefab;
        
        public bool SpawnCharacters;
        public int CharacterCount;
        public float CharacterSpacing;
    
        public bool SpawnEnvironment;
        public int EnvironmentIndex;

        public bool Multithreaded;
        public bool PhysicsStep;
    
        public bool StepHandling;
        public bool SlopeChanges;
        public bool ProjectVelocityOnOverlaps;
        public bool StatefulHits;
        public bool SimulateDynamic;
        public bool SaveRestoreState;
        public bool EnhancedGroundPrecision;
    }

    public struct EnvironmentPrefabs : IBufferElementData
    {
        public Entity Prefab;
    }

    public struct SpawnedCharacter : IBufferElementData
    {
        public Entity Entity;
    }

    public struct SpawnedEnvironment : IBufferElementData
    {
        public Entity Entity;
    }

    public struct Event : IBufferElementData
    {
        public EventType Type;
    }

    EntityQuery m_CharacterQuery;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<Singleton>();
        m_CharacterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
            .WithAll<
                StressTestCharacterComponent,
                StressTestCharacterControl>()
            .Build(ref state);
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        if (!SystemAPI.HasSingleton<Singleton>())
            return;

        Singleton singleton = SystemAPI.GetSingleton<Singleton>();
        NativeArray<Event> events = SystemAPI.GetSingletonBuffer<Event>().ToNativeArray(Allocator.Temp);

        for (int i = 0; i < events.Length; i++)
        {
            switch (events[i].Type)
            {
                case EventType.SpawnCharacters:
                    SpawnCharacters(ref state, singleton);
                    break;
                case EventType.SpawnEnvironment:
                    SpawnEnvironment(ref state, singleton);
                    break;
                case EventType.ApplyPhysicsStep:
                    ApplyPhysicsStep(ref state, singleton);
                    break;
                case EventType.ApplyStepHandling:
                    ApplyStepHandling(ref state, singleton);
                    break;
                case EventType.ApplySlopeChanges:
                    ApplySlopeChanges(ref state, singleton);
                    break;
                case EventType.ApplyProjectVelocityOnOverlaps:
                    ApplyProjectVelocityOnOverlaps(ref state, singleton);
                    break;
                case EventType.ApplyStatefulHits:
                    ApplyStatefulHits(ref state, singleton);
                    break;
                case EventType.ApplySimulateDynamic:
                    ApplySimulateDynamic(ref state, singleton);
                    break;
                case EventType.ApplySaveRestoreState:
                    ApplySaveRestoreState(ref state, singleton);
                    break;
                case EventType.ApplyEnhancedGroundPrecision:
                    ApplyEnhancedGroundPrecision(ref state, singleton);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
        events.Dispose();
        
        SystemAPI.GetSingletonBuffer<Event>().Clear();
    }

    [BurstCompile]
    void SpawnCharacters(ref SystemState state, Singleton singleton)
    {
        EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
        DynamicBuffer<Entity> spawnedCharacters = SystemAPI.GetSingletonBuffer<SpawnedCharacter>().Reinterpret<Entity>();
        
        // Clear spawned characters
        for (int i = 0; i < spawnedCharacters.Length; i++)
        {
            ecb.DestroyEntity(spawnedCharacters[i]);
        }
        spawnedCharacters.Clear();
        
        // Spawn new characters
        int spawnResolution = Mathf.CeilToInt(Mathf.Sqrt(singleton.CharacterCount));
        float totalWidth = (spawnResolution - 1) * singleton.CharacterSpacing;
        float3 spawnBottomCorner = (-math.right() * totalWidth * 0.5f) + (-math.forward() * totalWidth * 0.5f);
        int counter = 0;
        for (int x = 0; x < spawnResolution; x++)
        {
            for (int z = 0; z < spawnResolution; z++)
            {
                if (counter >= singleton.CharacterCount)
                {
                    break;
                }
                Entity spawnedCharacter = ecb.Instantiate(singleton.CharacterPrefab);
                ecb.AppendToBuffer(SystemAPI.GetSingletonEntity<Singleton>(),new SpawnedCharacter { Entity = spawnedCharacter });
                float3 spawnPos = spawnBottomCorner + (math.right() * x * singleton.CharacterSpacing) + (math.forward() * z * singleton.CharacterSpacing);
                ecb.SetComponent(spawnedCharacter, new LocalTransform { Position = spawnPos, Rotation = quaternion.identity, Scale = 1f});
                counter++;
            }
        }
        
        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }

    [BurstCompile]
    void SpawnEnvironment(ref SystemState state, Singleton singleton)
    {
        EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
        
        DynamicBuffer<Entity> spawnedEnvironments = SystemAPI.GetSingletonBuffer<SpawnedEnvironment>().Reinterpret<Entity>();
        DynamicBuffer<Entity> environmentPrefabs = SystemAPI.GetSingletonBuffer<EnvironmentPrefabs>().Reinterpret<Entity>();
        
        for (int i = 0; i < spawnedEnvironments.Length; i++)
        {
            ecb.DestroyEntity(spawnedEnvironments[i]);
        }
        
        ecb.AppendToBuffer(SystemAPI.GetSingletonEntity<Singleton>(),new SpawnedEnvironment { Entity = ecb.Instantiate(environmentPrefabs[singleton.EnvironmentIndex])});
        
        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }

    [BurstCompile]
    void ApplyPhysicsStep(ref SystemState state, Singleton singleton)
    {
        ref PhysicsStep physicsStep = ref SystemAPI.GetSingletonRW<PhysicsStep>().ValueRW;
        physicsStep.SimulationType = singleton.PhysicsStep ? SimulationType.UnityPhysics : SimulationType.NoPhysics;
        
        // TODO: havok
    }

    [BurstCompile]
    void ApplyStepHandling(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            StressTestCharacterComponent character = state.EntityManager.GetComponentData<StressTestCharacterComponent>(entities[i]);
            character.StepAndSlopeHandling.StepHandling = singleton.StepHandling;
            state.EntityManager.SetComponentData(entities[i], character);
        }
        entities.Dispose();
    }

    [BurstCompile]
    void ApplySlopeChanges(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            StressTestCharacterComponent character = state.EntityManager.GetComponentData<StressTestCharacterComponent>(entities[i]);
            character.StepAndSlopeHandling.PreventGroundingWhenMovingTowardsNoGrounding = singleton.SlopeChanges;
            character.StepAndSlopeHandling.HasMaxDownwardSlopeChangeAngle = singleton.SlopeChanges;
            state.EntityManager.SetComponentData(entities[i], character);
        }
        entities.Dispose();
    }

    [BurstCompile]
    void ApplyProjectVelocityOnOverlaps(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            KinematicCharacterProperties characterProperties = state.EntityManager.GetComponentData<KinematicCharacterProperties>(entities[i]);
            characterProperties.ProjectVelocityOnInitialOverlaps = singleton.ProjectVelocityOnOverlaps;
            state.EntityManager.SetComponentData(entities[i], characterProperties);
        }
        entities.Dispose();
    }

    [BurstCompile]
    void ApplyStatefulHits(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            StressTestCharacterComponent character = state.EntityManager.GetComponentData<StressTestCharacterComponent>(entities[i]);
            character.UseStatefulHits = singleton.StatefulHits;
            state.EntityManager.SetComponentData(entities[i], character);
        }
        entities.Dispose();
    }

    [BurstCompile]
    unsafe void ApplySimulateDynamic(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            KinematicCharacterProperties characterProperties = state.EntityManager.GetComponentData<KinematicCharacterProperties>(entities[i]);
            characterProperties.SimulateDynamicBody = singleton.SimulateDynamic;
            state.EntityManager.SetComponentData(entities[i], characterProperties);
            
            PhysicsCollider physicsCollider = state.EntityManager.GetComponentData<PhysicsCollider>(entities[i]);
            ConvexCollider* collider = (ConvexCollider*)physicsCollider.ColliderPtr;
            Unity.Physics.Material material = collider->Material;
            collider->Material = material;
            state.EntityManager.SetComponentData(entities[i], physicsCollider);
        }
        entities.Dispose();
    }

    [BurstCompile]
    void ApplySaveRestoreState(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            StressTestCharacterComponent character = state.EntityManager.GetComponentData<StressTestCharacterComponent>(entities[i]);
            character.UseSaveRestoreState = singleton.SaveRestoreState;
            state.EntityManager.SetComponentData(entities[i], character);
        }
        entities.Dispose();
    }

    [BurstCompile]
    void ApplyEnhancedGroundPrecision(ref SystemState state, Singleton singleton)
    {
        NativeArray<Entity> entities = m_CharacterQuery.ToEntityArray(Allocator.TempJob);
        for (int i = 0; i < entities.Length; i++)
        {
            KinematicCharacterProperties characterProperties = state.EntityManager.GetComponentData<KinematicCharacterProperties>(entities[i]);
            characterProperties.EnhancedGroundPrecision = singleton.SimulateDynamic;
            state.EntityManager.SetComponentData(entities[i], characterProperties);
        }
        entities.Dispose();
    }
}
