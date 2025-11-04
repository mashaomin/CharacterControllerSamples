using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace OnlineFPS
{
    [Serializable]
    public struct ConstantRotation : IComponentData
    {
        public float3 RotationSpeed;
    }
}
