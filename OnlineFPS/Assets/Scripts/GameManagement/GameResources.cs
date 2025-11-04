using System;
using Unity.Entities;
using Unity.NetCode;
using UnityEngine;

namespace OnlineFPS
{
    [Serializable]
    public struct GameResources : IComponentData
    {
        public ClientServerTickRate ClientServerTickRate;
        public float JoinTimeout;
        public uint DespawnTicks;
        public uint PolledEventsTicks;
        public float RespawnTime;

        public Entity PlayerGhost;
        public Entity CharacterGhost;
        public Entity SpectatorPrefab;

        public bool ForceOnlyFirstWeapon;
    }

    public struct GameResourcesWeapon : IBufferElementData
    {
        public Entity WeaponPrefab;
    }
}