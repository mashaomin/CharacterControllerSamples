using System;
using Unity.Entities;
using UnityEngine;

[Serializable]
public struct BouncySurface : IComponentData
{
    public float BounceEnergyMultiplier;
}