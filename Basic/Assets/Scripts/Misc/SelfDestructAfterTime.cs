using System;
using Unity.Entities;
using UnityEngine;

[Serializable]
public struct SelfDestructAfterTime : IComponentData
{
    public float LifeTime;
    public float TimeSinceAlive;
}