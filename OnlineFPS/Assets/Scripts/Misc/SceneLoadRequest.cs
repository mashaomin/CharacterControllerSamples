using System;
using Unity.Entities;
using UnityEngine;

namespace OnlineFPS
{
    [Serializable]
    public struct SceneLoadRequest : IComponentData
    {
        public bool IsLoaded;
    }

    [Serializable]
    public struct SceneIdentifier : IBufferElementData
    {
        public Entity SceneEntity;

        public SceneIdentifier(Entity sceneEntity)
        {
            SceneEntity = sceneEntity;
        }
    }
}
