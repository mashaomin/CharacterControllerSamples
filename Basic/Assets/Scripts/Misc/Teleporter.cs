using System;
using Unity.Entities;

[Serializable]
public struct Teleporter : IComponentData
{
    public Entity DestinationEntity;
}