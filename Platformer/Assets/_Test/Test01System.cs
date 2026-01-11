using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Stateful;
using Unity.Physics.Systems;
using Unity.Transforms;
using Unity.CharacterController;
using Unity.Physics.Extensions;

namespace KCCTest
{
    public partial struct Test01System : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        { }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每10秒切换一次状态
            double time = SystemAPI.Time.ElapsedTime;
            // 0-10s: None (false), 10-20s: Collide (true)
            bool enableCollision = (time % 20.0d) >= 10.0d;
            CollisionResponsePolicy targetPolicy = enableCollision ? CollisionResponsePolicy.Collide : CollisionResponsePolicy.None;

            Test01Job job = new Test01Job
            {
                TargetPolicy = targetPolicy
            };
            var d = job.Schedule(state.Dependency);
            state.Dependency = d;


            /*// 遍历需要修改的实体
            foreach (var (colliderRef, entity) in SystemAPI.Query<RefRW<PhysicsCollider>>()
                     .WithAll<Test01Component>() // 假设你有标记组件
                     .WithEntityAccess())
            {
                // 1. 获取当前的 Collider 引用
                ref PhysicsCollider collider = ref colliderRef.ValueRW;

                // 2. 必须创建一个新的 BlobAsset 副本 (Make Unique)
                // 注意：这会分配内存，尽量不要每帧都做，最好只在初始化时做一次
                if (collider.Value.IsCreated)
                {
                    // 使用 Unity.Physics 提供的工具或自定义扩展来克隆并修改
                    // 这里通常需要先克隆 Blob，然后修改新 Blob 的属性
                    // 由于 Unity Physics 没有内置简单的一行 MakeUnique，
                    // 常见的做法是预制好两种 Collider (正常/无碰撞)，然后切换 PhysicsCollider 组件的引用。

                    // 或者，如果只是为了禁用碰撞，可以考虑禁用 PhysicsBody 或修改 CollisionFilter
                }
            }*/
        }

        [BurstCompile]
        public partial struct Test01Job : IJobEntity
        {
            public CollisionResponsePolicy TargetPolicy;

            public void Execute(Entity e, Test01Component testComponent, ref PhysicsCollider physicsCollider)
            {
                if (physicsCollider.Value.IsCreated)
                {
                    // 注意：这会修改共享的 BlobAsset，所有使用此 Collider 的实体都会受影响
                    physicsCollider.Value.Value.SetCollisionResponse(TargetPolicy);
                }
            }
        }

    }
}