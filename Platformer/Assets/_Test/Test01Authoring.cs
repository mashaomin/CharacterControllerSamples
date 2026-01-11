using System.Collections;
using System.Collections.Generic;
using System.Data;

using Unity.Entities;

using UnityEngine;

namespace KCCTest
{
    public class Test01Authoring : MonoBehaviour
    {
        class Baker : Baker<Test01Authoring>
        {
            public override void Bake(Test01Authoring authoring)
            {
                Entity entity = GetEntity(TransformUsageFlags.None);
                AddComponent(entity, new Test01Component());
            }
        }
    }
}