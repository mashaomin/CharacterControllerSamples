using Unity.Entities;
using UnityEngine;

namespace OnlineFPS
{
    public class ConstantRotationAuthoring : MonoBehaviour
    {
        public ConstantRotation ConstantRotation;

        public class Baker : Baker<ConstantRotationAuthoring>
        {
            public override void Bake(ConstantRotationAuthoring authoring)
            {
                Entity entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, authoring.ConstantRotation);
            }
        }
    }
}