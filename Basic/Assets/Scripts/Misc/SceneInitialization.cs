using Unity.Entities;
using UnityEngine;

[System.Serializable]
public struct SceneInitialization : IComponentData
{
    public Entity CharacterSpawnPointEntity;
    public Entity CharacterPrefabEntity;
    public Entity CameraPrefabEntity;
    public Entity PlayerPrefabEntity;
}
