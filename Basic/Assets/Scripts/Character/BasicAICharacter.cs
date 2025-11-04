using System;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
public struct BasicAICharacter : IComponentData
{
    public float MovementPeriod;
    public float3 MovementDirection;
}
