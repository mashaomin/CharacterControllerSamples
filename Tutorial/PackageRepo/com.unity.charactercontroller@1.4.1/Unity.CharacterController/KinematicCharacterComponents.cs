using System;
using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.CharacterController
{
    /// <summary>
    /// Defines the authoring data for kinematic character properties
    /// </summary>
    [Serializable]
    public struct AuthoringKinematicCharacterProperties
    {
        /// <summary>
        /// Nothing
        /// 给角色刚体附加的物理标签，用于过滤/识别碰撞。 
        ///     示例：把角色设为 Player 标签，便于只被某些触发器检测。
        /// </summary>
        [Header("基础特性")]
        public CustomPhysicsBodyTags CustomPhysicsBodyTags;
        /// <summary>
        /// true
        /// 在物理步之间插值位置，视觉更平滑。示例：低帧率下奔跑不抖动
        /// </summary>
        public bool InterpolatePosition;
        /// <summary>
        /// false
        /// 在物理步之间插值旋转。标准角色默认禁用，因为它们在常规帧更新里自己转向。
        ///     示例：如果自定义角色在 FixedUpdate 驱动旋转且想平滑显示，可开启。
        /// </summary>
        public bool InterpolateRotation;

        /// <summary>
        /// 是否检测并评估接地状态（坡度、法线等）。若关掉，角色不会自动判定落地/滑坡
        /// </summary>
        [Header("接地")]
        public bool EvaluateGrounding;
        /// <summary>
        /// true
        /// 在地面下方一定距离内自动贴地，避免从台阶/斜坡微微“浮空”。
        /// </summary>
        public bool SnapToGround;
        /// <summary>
        /// 0.5f
        /// 贴地射线的最大距离。示例：楼梯台阶较高时可稍微增大。
        /// </summary>
        public float GroundSnappingDistance;
        /// <summary>
        /// false
        /// 对距离为 0 的命中做精确重算，减少靠墙/斜面时的抖动；有额外开销，遇到抖动再开。
        /// </summary>
        public bool EnhancedGroundPrecision;
        /// <summary>
        /// 45度
        /// 可视为已接地的最大坡度。示例：设为 45°，更陡的坡会被判定为未接地并滑落。
        /// </summary>
        public float MaxGroundedSlopeAngle;

        /// <summary>
        /// true
        /// 基于速度的碰撞体 cast（防穿透）。几乎总是保持开启
        /// </summary>
        [Header("碰撞")]
        public bool DetectMovementCollisions;
        /// <summary>
        /// true
        /// 处理初始或过程中的重叠（推出去）
        /// </summary>
        public bool DecollideFromOverlaps;
        /// <summary>
        /// false
        /// 在移动前对初始重叠再做一次速度投影，降低旋转导致的穿透风险；有性能代价，只有遇到穿透再开
        /// </summary>
        public bool ProjectVelocityOnInitialOverlaps;
        /// <summary>
        /// 8次
        /// 本帧最多连续 cast 次数（防止卡死）。提高可减少高速/狭窄环境漏检但更耗时
        /// </summary>
        public byte MaxContinuousCollisionsIterations;
        /// <summary>
        /// 2次
        /// 本帧最多重叠分离迭代次数。
        /// </summary>
        public byte MaxOverlapDecollisionIterations;
        /// <summary>
        /// true
        /// 超过迭代上限时丢弃本帧剩余位移，避免继续卡墙
        /// </summary>
        public bool DiscardMovementWhenExceedMaxIterations;
        /// <summary>
        /// true
        /// 超过迭代上限时清零速度，防止下帧继续穿透。
        /// </summary>
        public bool KillVelocityWhenExceedMaxIterations;
        /// <summary>
        /// false
        /// 被父物体（平台）携带时也做 cast 检测阻挡，而不是直接平移。平台可能推你撞墙时可开启。
        public bool DetectObstructionsForParentBodyMovement;

        /// <summary>
        /// true
        /// 允许与动态刚体互推/被推；需将碰撞响应设为 None 或 Raise Trigger Events 才能被推
        /// </summary>
        [Header("（动力学交互）")]
        public bool SimulateDynamicBody;
        /// <summary>
        /// 与动态刚体交互的质量。示例：设大一点（如 50）让角色更像“重坦克”，不易被推走。
        /// </summary>
        public float Mass;

        public static AuthoringKinematicCharacterProperties GetDefault()
        {
            AuthoringKinematicCharacterProperties c = new AuthoringKinematicCharacterProperties
            {
                // Body Properties
                CustomPhysicsBodyTags = CustomPhysicsBodyTags.Nothing,
                InterpolatePosition = true,
                InterpolateRotation = false,

                // Grounding
                EvaluateGrounding = true,
                SnapToGround = true,
                GroundSnappingDistance = 0.5f,
                EnhancedGroundPrecision = false,
                MaxGroundedSlopeAngle = 60f,

                // Collisions
                DetectMovementCollisions = true,
                DecollideFromOverlaps = true,
                ProjectVelocityOnInitialOverlaps = false,
                MaxContinuousCollisionsIterations = 8,
                MaxOverlapDecollisionIterations = 2,
                DiscardMovementWhenExceedMaxIterations = true,
                KillVelocityWhenExceedMaxIterations = true,
                DetectObstructionsForParentBodyMovement = false,

                // Dynamics
                SimulateDynamicBody = true,
                Mass = 1f,
            };
            return c;
        }
    }

    /// <summary>
    /// Component holding general properties for a kinematic character
    /// </summary>
    [Serializable]
    public struct KinematicCharacterProperties : IComponentData
    {
        /// <summary>
        /// Enables detecting ground and evaluating grounding for each hit
        /// </summary>
        public bool EvaluateGrounding;
        /// <summary>
        /// Enables snapping to the ground surface below the character
        /// </summary>
        public bool SnapToGround;
        /// <summary>
        /// Distance to snap to ground, if SnapToGround is enabled
        /// </summary>
        public float GroundSnappingDistance;
        /// <summary>
        /// Computes a more precise distance to ground hits when the original query returned a distance of 0f due to imprecisions. Helps reduce jitter in certain situations, but can have an additional performance cost. It is recommended that you only enable this if you notice jitter problems when moving against angled walls
        /// </summary>
        public bool EnhancedGroundPrecision;
        /// <summary>
        /// The max slope angle that the character can be considered grounded on
        /// </summary>
        public float MaxGroundedSlopeDotProduct;

        /// <summary>
        /// Enables detecting and solving movement collisions with a collider cast, based on character's velocity
        /// </summary>
        public bool DetectMovementCollisions;
        /// <summary>
        /// Enables detecting and solving overlaps
        /// </summary>
        public bool DecollideFromOverlaps;
        /// <summary>
        /// Enables doing an extra physics check to project velocity on initial overlaps before the character moves. This can help with tunneling issues when you rotate your character in a way that could change the detected collisions (which doesn't happen if your character has an upright capsule shape and only rotates around up axis, for example), but it has a performance cost.
        /// </summary>
        public bool ProjectVelocityOnInitialOverlaps;
        /// <summary>
        /// The maximum amount of times per frame that the character should try to cast its collider for detecting hits
        /// </summary>
        public byte MaxContinuousCollisionsIterations;
        /// <summary>
        /// The maximum amount of times per frame that the character should try to decollide itself from overlaps
        /// </summary>
        public byte MaxOverlapDecollisionIterations;
        /// <summary>
        /// Whether we should reset the remaining move distance to zero when the character exceeds the maximum collision iterations
        /// </summary>
        public bool DiscardMovementWhenExceedMaxIterations;
        /// <summary>
        /// Whether we should reset the velocity to zero when the character exceeds the maximum collision iterations
        /// </summary>
        public bool KillVelocityWhenExceedMaxIterations;
        /// <summary>
        /// Enables doing a collider cast to detect obstructions when being moved by a parent body, instead of simply moving the character transform along
        /// </summary>
        public bool DetectObstructionsForParentBodyMovement;

        /// <summary>
        /// Enables physics interactions (push and be pushed) with other dynamic bodies. Note that in order to be pushed properly, the character's collision response has to be either \"None\" or \"Raise Trigger Events\"
        /// </summary>
        public bool SimulateDynamicBody;
        /// <summary>
        /// The mass used to simulate dynamic body interactions
        /// </summary>
        public float Mass;

        /// <summary>
        /// Constructs the runtime properties component based on authoring data
        /// </summary>
        /// <param name="forAuthoring"> The authoring character properties to build these properties from </param>
        public KinematicCharacterProperties(AuthoringKinematicCharacterProperties forAuthoring)
        {
            EvaluateGrounding = forAuthoring.EvaluateGrounding;
            SnapToGround = forAuthoring.SnapToGround;
            GroundSnappingDistance = forAuthoring.GroundSnappingDistance;
            EnhancedGroundPrecision = forAuthoring.EnhancedGroundPrecision;
            MaxGroundedSlopeDotProduct = MathUtilities.AngleRadiansToDotRatio(math.radians(forAuthoring.MaxGroundedSlopeAngle));

            DetectMovementCollisions = forAuthoring.DetectMovementCollisions;
            DecollideFromOverlaps = forAuthoring.DecollideFromOverlaps;
            ProjectVelocityOnInitialOverlaps = forAuthoring.ProjectVelocityOnInitialOverlaps;
            MaxContinuousCollisionsIterations = forAuthoring.MaxContinuousCollisionsIterations;
            MaxOverlapDecollisionIterations = forAuthoring.MaxOverlapDecollisionIterations;
            DiscardMovementWhenExceedMaxIterations = forAuthoring.DiscardMovementWhenExceedMaxIterations;
            KillVelocityWhenExceedMaxIterations = forAuthoring.KillVelocityWhenExceedMaxIterations;
            DetectObstructionsForParentBodyMovement = forAuthoring.DetectObstructionsForParentBodyMovement;

            SimulateDynamicBody = forAuthoring.SimulateDynamicBody;
            Mass = forAuthoring.Mass;
        }

        /// <summary>
        /// Whether or not dynamic rigidbody collisions should be enabled, with the current character properties
        /// </summary>
        /// <returns> If the character should ignore dynamic bodies based on these properties </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ShouldIgnoreDynamicBodies()
        {
            return !SimulateDynamicBody;
        }
    }

    /// <summary>
    /// A component holding the transient data (data that gets modified during the character update) of the character
    /// </summary>
    [Serializable]
    public struct KinematicCharacterBody : IComponentData, IEnableableComponent
    {
        /// <summary>
        /// Whether the character is currently grounded or not
        /// </summary>
        public bool IsGrounded;
        /// <summary>
        /// The character's velocity relatively to its assigned parent's velocity (if any)
        /// </summary>
        public float3 RelativeVelocity;
        /// <summary>
        /// The character's parent entity
        /// </summary>
        public Entity ParentEntity;
        /// <summary>
        /// The character's anchor point to its parent, expressed in the parent's local space
        /// </summary>
        public float3 ParentLocalAnchorPoint;

        // The following data is fully reset at the beginning of the character update, or recalculated during the update.
        // This means it typically doesn't need any network sync unless you access that data before the character update.

        /// <summary>
        /// The character's grounding up direction
        /// </summary>
        public float3 GroundingUp;
        /// <summary>
        /// The character's detected ground hit
        /// </summary>
        public BasicHit GroundHit;
        /// <summary>
        /// The calculated velocity of the character's parent
        /// </summary>
        public float3 ParentVelocity;
        /// <summary>
        /// The previous parent entity
        /// </summary>
        public Entity PreviousParentEntity;
        /// <summary>
        /// The rotation resulting from the parent's movement over the latest update
        /// </summary>
        public quaternion RotationFromParent;
        /// <summary>
        /// The last known delta time of the character update
        /// </summary>
        public float LastPhysicsUpdateDeltaTime;
        /// <summary>
        /// Whether or not the character was considered grounded at the beginning of the update, before ground is detected
        /// </summary>
        public bool WasGroundedBeforeCharacterUpdate;

        /// <summary>
        /// Returns a sensible default for this component
        /// </summary>
        /// <returns> The default KinematicCharacterBody </returns>
        public static KinematicCharacterBody GetDefault()
        {
            return new KinematicCharacterBody
            {
                IsGrounded = default,
                RelativeVelocity = default,
                ParentEntity = default,
                ParentLocalAnchorPoint = default,

                GroundingUp = math.up(),
                GroundHit = default,
                ParentVelocity = default,
                PreviousParentEntity = default,
                RotationFromParent = quaternion.identity,
                LastPhysicsUpdateDeltaTime = 0f,
                WasGroundedBeforeCharacterUpdate = default,
            };
        }

        /// <summary>
        /// Whether or not the character has become grounded on this frame
        /// </summary>
        /// <returns> If the character has become grounded </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeGrounded()
        {
            return !WasGroundedBeforeCharacterUpdate && IsGrounded;
        }

        /// <summary>
        /// Whether or not the character has become ungrounded on this frame
        /// </summary>
        /// <returns> If the character has become ungrounded </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeUngrounded()
        {
            return WasGroundedBeforeCharacterUpdate && !IsGrounded;
        }
    }

    /// <summary>
    /// Stores all the data that a character might need to access on OTHER characters during its update.
    /// This component exists only in order to allow deterministic parallel execution of the character update.
    /// </summary>
    [System.Serializable]
    public struct StoredKinematicCharacterData : IComponentData
    {
        /// <summary>
        /// Enables physics interactions (push and be pushed) with other dynamic bodies. Note that in order to be pushed properly, the character's collision response has to be either \"None\" or \"Raise Trigger Events\"
        /// 角色能够 推动动态刚体，或被动态刚体推动
        /// SimulateDynamicBody=true
        ///     角色会对自身以及其他刚体施加力
        ///     以此来 模拟真实动态刚体（dynamic rigidbody）的行为
        ///     通过Mass来模拟碰撞的质量对比
        /// </summary>
        public bool SimulateDynamicBody;
        /// <summary>
        /// The mass used to simulate dynamic body interactions
        /// </summary>
        public float Mass;
        /// <summary>
        /// The character's velocity relatively to its assigned parent's velocity (if any)
        /// </summary>
        public float3 RelativeVelocity;
        /// <summary>
        /// The calculated velocity of the character's parent
        /// </summary>
        public float3 ParentVelocity;

        /// <summary>
        /// Automatically sets the data in this component based on a character body and character properties component
        /// </summary>
        /// <param name="characterProperties"> A character properties component to get data from </param>
        /// <param name="characterBody"> A character body component to get data from </param>
        public void SetFrom(in KinematicCharacterProperties characterProperties, in KinematicCharacterBody characterBody)
        {
            SimulateDynamicBody = characterProperties.SimulateDynamicBody;
            Mass = characterProperties.Mass;
            RelativeVelocity = characterBody.RelativeVelocity;
            ParentVelocity = characterBody.ParentVelocity;
        }
    }

    /// <summary>
    /// Stores an impulse to apply to another body later in the frame
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicCharacterDeferredImpulse : IBufferElementData
    {
        /// <summary>
        /// Entity on which to apply the impulse
        /// </summary>
        public Entity OnEntity;
        /// <summary>
        /// The impulse's change in linear velocity
        /// </summary>
        public float3 LinearVelocityChange;
        /// <summary>
        /// The impulse's change in angular velocity
        /// </summary>
        public float3 AngularVelocityChange;
        /// <summary>
        /// The impulse's change in position
        /// </summary>
        public float3 Displacement;
    }

    /// <summary>
    /// Data representing a detected character hit
    /// 本帧内检测到的所有命中
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicCharacterHit : IBufferElementData
    {
        /// <summary>
        /// Hit entity
        /// </summary>
        public Entity Entity;
        /// <summary>
        /// Hit rigidbody index
        /// </summary>
        public int RigidBodyIndex;
        /// <summary>
        /// Hit collider key
        /// </summary>
        public ColliderKey ColliderKey;
        /// <summary>
        /// Hit point
        /// </summary>
        public float3 Position;
        /// <summary>
        /// Hit normal
        /// </summary>
        public float3 Normal;
        /// <summary>
        /// The hit physics material
        /// </summary>
        public Physics.Material Material;
        /// <summary>
        /// Whether or not the character was grounded when the hit was detected
        /// </summary>
        public bool WasCharacterGroundedOnHitEnter;
        /// <summary>
        /// Whether or not the character would consider itself grounded on this hit
        /// </summary>
        public bool IsGroundedOnHit;
        /// <summary>
        /// The character's velocity before velocity projection on this hit
        /// </summary>
        public float3 CharacterVelocityBeforeHit;
        /// <summary>
        /// The character's velocity after velocity projection on this hit
        /// </summary>
        public float3 CharacterVelocityAfterHit;
    }

    /// <summary>
    /// Holds the data for hits that participate in velocity projection
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicVelocityProjectionHit : IBufferElementData
    {
        /// <summary>
        /// Hit entity
        /// </summary>
        public Entity Entity;
        /// <summary>
        /// Hit rigidbody index
        /// </summary>
        public int RigidBodyIndex;
        /// <summary>
        /// Hit collider key
        /// </summary>
        public ColliderKey ColliderKey;
        /// <summary>
        /// Hit point
        /// </summary>
        public float3 Position;
        /// <summary>
        /// Hit normal
        /// </summary>
        public float3 Normal;
        /// <summary>
        /// Hit material
        /// </summary>
        public Physics.Material Material;
        /// <summary>
        /// Whether or not the character would consider itself grounded on this hit
        /// </summary>
        public bool IsGroundedOnHit;

        /// <summary>
        /// Constructs the velocity projection hit based on a character hit
        /// </summary>
        /// <param name="hit"> A character hit </param>
        public KinematicVelocityProjectionHit(KinematicCharacterHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
            IsGroundedOnHit = hit.IsGroundedOnHit;
        }

        /// <summary>
        /// Constructs the velocity projection hit based on a basic hit and grounding status
        /// </summary>
        /// <param name="hit"> A basic hit </param>
        /// <param name="isGroundedOnHit"> Whether or not the character is grounded on this hit </param>
        public KinematicVelocityProjectionHit(BasicHit hit, bool isGroundedOnHit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
            IsGroundedOnHit = isGroundedOnHit;
        }

        /// <summary>
        /// Constructs the velocity projection hit based on a basic hit and grounding status
        /// </summary>
        /// <param name="normal"> The hit normal </param>
        /// <param name="position"> The hit point </param>
        /// <param name="isGroundedOnHit"> Whether or not the character is grounded on this hit </param>
        public KinematicVelocityProjectionHit(float3 normal, float3 position, bool isGroundedOnHit)
        {
            Entity = Entity.Null;
            RigidBodyIndex = -1;
            ColliderKey = default;
            Position = position;
            Normal = normal;
            Material = default;
            IsGroundedOnHit = isGroundedOnHit;
        }
    }

    /// <summary>
    /// A stateful version of character hits, which includes the hit state (enter, exit, stay)
    /// 用于存储 带有状态信息（Enter / Exit / Stay） 的命中结果
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct StatefulKinematicCharacterHit : IBufferElementData
    {
        /// <summary>
        /// State of the hit (enter/exit/stay)
        /// </summary>
        public CharacterHitState State;
        /// <summary>
        /// The character hit
        /// </summary>
        public KinematicCharacterHit Hit;

        /// <summary>
        /// Constructs a stateful character hit from a character hit
        /// </summary>
        /// <param name="characterHit"> The character hit </param>
        public StatefulKinematicCharacterHit(KinematicCharacterHit characterHit)
        {
            State = default;
            Hit = characterHit;
        }
    }
}
