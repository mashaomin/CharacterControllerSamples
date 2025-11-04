using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace OnlineFPS
{
    [BurstCompile]
    public partial struct ConstantRotationSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            foreach (var (localTransform, constantRotation) in
                     SystemAPI.Query<RefRW<LocalTransform>, ConstantRotation>())
            {
                localTransform.ValueRW.Rotation =
                    math.mul(quaternion.Euler(constantRotation.RotationSpeed * SystemAPI.Time.DeltaTime),
                        localTransform.ValueRO.Rotation);
            }
        }
    }
}