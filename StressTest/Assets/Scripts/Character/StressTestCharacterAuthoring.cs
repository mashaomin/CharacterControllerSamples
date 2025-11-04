using Unity.Entities;
using UnityEngine;
using Unity.CharacterController;

[DisallowMultipleComponent]
public class StressTestCharacterAuthoring : MonoBehaviour
{
    public AuthoringKinematicCharacterProperties CharacterProperties = AuthoringKinematicCharacterProperties.GetDefault();
    public StressTestCharacterComponent Character = StressTestCharacterComponent.GetDefault();

    public class Baker : Baker<StressTestCharacterAuthoring>
    {
        public override void Bake(StressTestCharacterAuthoring authoring)
        {
            KinematicCharacterUtilities.BakeCharacter(this, authoring, authoring.CharacterProperties);

            Entity selfEntity = GetEntity(TransformUsageFlags.None);
            
            AddComponent(selfEntity, authoring.Character);
            AddComponent(selfEntity, new StressTestCharacterControl());
        }
    }
}
