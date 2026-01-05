using System;

using Unity.Burst;
using Unity.Collections;
using Unity.Entities;

namespace Unity.CharacterController
{
    /// <summary>
    /// 现象：
    ///     角色 A 的逻辑需要读取角色 B 的速度（比如跟随 B 移动）。
    ///     但是 B 这一帧的速度可能还没算出来，也可能算了一半。
    ///     这会导致逻辑的不确定性（先跑 A 还是先跑 B，结果不一样）。
    /// 
    /// 解决：
    ///     双缓冲（Double Buffering）思想。
    ///     在每一帧物理开始之前，把所有角色的状态（速度、质量等）拷贝一份到 StoredKinematicCharacterData 组件里。
    ///     在这一帧的计算中，如果 A 要读 B，只许读 B 的 StoredData（上一帧的快照）。
    ///     这样无论 A 和 B 谁先跑，读到的数据永远是稳定的。
    /// A system that stores key character data in a component on the character entity, before the character update
    /// </summary>
    [UpdateInGroup(typeof(KinematicCharacterPhysicsUpdateGroup), OrderFirst = true)]
    [BurstCompile]
    public partial struct StoreKinematicCharacterBodyPropertiesSystem : ISystem
    {
        EntityQuery m_StoredCharacterQuery;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_StoredCharacterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<StoredKinematicCharacterData, KinematicCharacterProperties>()
                .Build(ref state);

            state.RequireForUpdate(m_StoredCharacterQuery);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            StoreKinematicCharacterBodyPropertiesJob job = new StoreKinematicCharacterBodyPropertiesJob();
            job.ScheduleParallel();
        }

        /// <summary>
        /// Job that copies character data to another component on the same entity, to capture a snapshot of them before modifications.
        /// This exists to allow deterministic parallel character updates
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct StoreKinematicCharacterBodyPropertiesJob : IJobEntity
        {
            void Execute(ref StoredKinematicCharacterData storedData, in KinematicCharacterProperties characterProperties, in KinematicCharacterBody characterBody)
            {
                storedData.SetFrom(in characterProperties, in characterBody);
            }
        }
    }
}
