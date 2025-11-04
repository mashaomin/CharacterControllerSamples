using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[Serializable]
public struct PrefabThrower : IComponentData
{
    public Entity PrefabEntity;
    public float3 InitialEulerAngles;
    public float ThrowForce;
}