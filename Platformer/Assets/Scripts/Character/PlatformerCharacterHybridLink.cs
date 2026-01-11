using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

// 配置
[Serializable]
public class PlatformerCharacterHybridData : IComponentData
{
    public GameObject MeshPrefab;
}

// 运行时
[Serializable]
public class PlatformerCharacterHybridLink : ICleanupComponentData
{
    public GameObject Object;
    public Animator Animator;
}