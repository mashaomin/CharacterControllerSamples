using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[Serializable]
public struct CustomGravity : IComponentData
{
    public float GravityMultiplier;                             // 重力倍率。允许特定物体（如角色）受到的重力比其他物体更强或更弱。

    [HideInInspector] public float3 Gravity;                    // 当前帧最终计算出的重力向量 (方向 * 大小)
    [HideInInspector] public bool TouchedByNonGlobalGravity;
    [HideInInspector] public Entity CurrentZoneEntity;
    [HideInInspector] public Entity LastZoneEntity;
}