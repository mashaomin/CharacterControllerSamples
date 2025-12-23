using System;
using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;
using Unity.Physics;
using UnityEngine;

public struct ThirdPersonCharacterUpdateContext
{
    // Here, you may add additional global data for your character updates, such as ComponentLookups, Singletons, NativeCollections, etc...
    // The data you add here will be accessible in your character updates and all of your character "callbacks".

    public void OnSystemCreate(ref SystemState state)
    {
        // Get lookups
    }

    public void OnSystemUpdate(ref SystemState state)
    {
        // Update lookups
    }
}

/// <summary>
/// 这是纯逻辑层，实现了 IKinematicCharacterProcessor 接口。它不直接由 ECS 运行，而是被 System 调用
/// 封装了角色如何移动、如何处理碰撞、如何响应输入的具体算法
///     1. PhysicsUpdate：固定步长的物理更新。
///         处理核心移动逻辑：初始化 -> 父对象跟随 -> 接地检测 -> 速度控制（HandleVelocityControl） 
///         -> 碰撞去穿透 -> 动量保持。这是角色“物理上”怎么动的代码。
///     2. VariableUpdate：变步长更新（通常是每帧渲染前）
///         处理平滑插值，比如角色旋转（为了视觉平滑，旋转通常在这里做，而不是在固定物理
///     3. HandleVelocityControl：具体的业务逻辑，判断在地面怎么走、在空中怎么跳、怎么加速。
///     4. Callbacks（如 IsGroundedOnHit, OnMovementHit）：定制物理交互细节（如：什么样的坡算地面、撞墙后是否滑行）
/// 
/// 接口：IKinematicCharacterProcessor
///     Up：头朝哪？
///     CanCollide：要不要撞？
///     IsGrounded：能不能站？
///     OnMovementHit：撞了怎么走？
///     ProjectVelocity：被夹住怎么滑？
///     OverrideMass：推不推得动？
/// </summary>
public struct ThirdPersonCharacterProcessor : IKinematicCharacterProcessor<ThirdPersonCharacterUpdateContext>
{
    public KinematicCharacterDataAccess CharacterDataAccess;
    public RefRW<ThirdPersonCharacterComponent> CharacterComponent;
    public RefRW<ThirdPersonCharacterControl> CharacterControl;

    public void PhysicsUpdate(ref ThirdPersonCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref ThirdPersonCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;

        // First phase of default character update
        // 在更新开始时，清空并初始化角色的核心数据和缓冲区。
        KinematicCharacterUtilities.Update_Initialize(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.DeferredImpulsesBuffer,
            CharacterDataAccess.VelocityProjectionHits,
            baseContext.Time.DeltaTime);

        // 如果角色被分配了 ParentEntity，则根据该父实体的运动来移动角色。
        KinematicCharacterUtilities.Update_ParentMovement(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            ref characterPosition,
            characterBody.WasGroundedBeforeCharacterUpdate);

        // 检测角色是否处于接地状态。
        KinematicCharacterUtilities.Update_Grounding(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterEntity,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            CharacterDataAccess.VelocityProjectionHits,
            CharacterDataAccess.CharacterHitsBuffer,
            ref characterPosition);

        // Update desired character velocity after grounding was detected, but before doing additional processing that depends on velocity
        // 根据传入方法的 BasicStepAndSlopeHandlingParameters 定义，取消角色的接地状态。例如：当角色正朝着悬崖边缘前进时，取消其接地状态。
        HandleVelocityControl(ref context, ref baseContext);

        // Second phase of default character update
        // 根据传入方法的 BasicStepAndSlopeHandlingParameters 定义，取消角色的接地状态。例如：当角色正朝着悬崖边缘前进时，取消其接地状态。
        KinematicCharacterUtilities.Update_PreventGroundingFromFutureSlopeChange(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            in characterComponent.StepAndSlopeHandling);

        // 如果当前地面实体是动态实体（dynamic），则对其施加一个持续的力。
        KinematicCharacterUtilities.Update_GroundPushing(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            CharacterDataAccess.DeferredImpulsesBuffer,
            characterComponent.Gravity);

        // 根据角色速度移动角色，并解决碰撞（去穿透）。
        KinematicCharacterUtilities.Update_MovementAndDecollisions(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            CharacterDataAccess.VelocityProjectionHits,
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.DeferredImpulsesBuffer,
            ref characterPosition);

        // 检测有效的移动平台实体，并将其分配为角色的 ParentEntity。更多信息请参阅 Parenting（父子关系）文档
        KinematicCharacterUtilities.Update_MovingPlatformDetection(
            ref baseContext,
            ref characterBody);

        // 当角色与父实体分离时，保留角色的速度动量
        KinematicCharacterUtilities.Update_ParentMomentum(
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.LocalTransform.ValueRO.Position);

        // 将具有 Enter、Exit 或 Stay 状态的角色碰撞结果写入角色实体上的 StatefulKinematicCharacterHit 缓冲区
        KinematicCharacterUtilities.Update_ProcessStatefulCharacterHits(
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.StatefulHitsBuffer);
    }

    void HandleVelocityControl(ref ThirdPersonCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref ThirdPersonCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref ThirdPersonCharacterControl characterControl = ref CharacterControl.ValueRW;

        // Rotate move input and velocity to take into account parent rotation
        if (characterBody.ParentEntity != Entity.Null)
        {
            characterControl.MoveVector = math.rotate(characterBody.RotationFromParent, characterControl.MoveVector);
            characterBody.RelativeVelocity = math.rotate(characterBody.RotationFromParent, characterBody.RelativeVelocity);
        }

        if (characterBody.IsGrounded)
        {
            // Move on ground
            float3 targetVelocity = characterControl.MoveVector * characterComponent.GroundMaxSpeed;
            CharacterControlUtilities.StandardGroundMove_Interpolated(ref characterBody.RelativeVelocity, targetVelocity, characterComponent.GroundedMovementSharpness, deltaTime, characterBody.GroundingUp, characterBody.GroundHit.Normal);

            // Jump
            if (characterControl.Jump)
            {
                CharacterControlUtilities.StandardJump(ref characterBody, characterBody.GroundingUp * characterComponent.JumpSpeed, true, characterBody.GroundingUp);
            }
        }
        else
        {
            // Move in air
            float3 airAcceleration = characterControl.MoveVector * characterComponent.AirAcceleration;
            if (math.lengthsq(airAcceleration) > 0f)
            {
                float3 tmpVelocity = characterBody.RelativeVelocity;
                CharacterControlUtilities.StandardAirMove(ref characterBody.RelativeVelocity, airAcceleration, characterComponent.AirMaxSpeed, characterBody.GroundingUp, deltaTime, false);

                // Cancel air acceleration from input if we would hit a non-grounded surface (prevents air-climbing slopes at high air accelerations)
                if (characterComponent.PreventAirAccelerationAgainstUngroundedHits
                    && KinematicCharacterUtilities.MovementWouldHitNonGroundedObstruction(
                        in this,
                        ref context,
                        ref baseContext,
                        CharacterDataAccess.CharacterProperties.ValueRO,
                        CharacterDataAccess.LocalTransform.ValueRO,
                        CharacterDataAccess.CharacterEntity,
                        CharacterDataAccess.PhysicsCollider.ValueRO,
                        characterBody.RelativeVelocity * deltaTime,
                        out ColliderCastHit hit))
                {
                    characterBody.RelativeVelocity = tmpVelocity;
                }
            }

            // Gravity
            CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, characterComponent.Gravity, deltaTime);

            // Drag
            CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, characterComponent.AirDrag);
        }
    }

    public void VariableUpdate(ref ThirdPersonCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref ThirdPersonCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref ThirdPersonCharacterControl characterControl = ref CharacterControl.ValueRW;
        ref quaternion characterRotation = ref CharacterDataAccess.LocalTransform.ValueRW.Rotation;

        // Add rotation from parent body to the character rotation
        // (this is for allowing a rotating moving platform to rotate your character as well, and handle interpolation properly)
        KinematicCharacterUtilities.AddVariableRateRotationFromFixedRateRotation(ref characterRotation, characterBody.RotationFromParent, baseContext.Time.DeltaTime, characterBody.LastPhysicsUpdateDeltaTime);

        // Rotate towards move direction
        if (math.lengthsq(characterControl.MoveVector) > 0f)
        {
            CharacterControlUtilities.SlerpRotationTowardsDirectionAroundUp(ref characterRotation, baseContext.Time.DeltaTime, math.normalizesafe(characterControl.MoveVector), MathUtilities.GetUpFromRotation(characterRotation), characterComponent.RotationSharpness);
        }
    }

    #region Character Processor Callbacks

    public void UpdateGroundingUp(ref ThirdPersonCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;

        KinematicCharacterUtilities.Default_UpdateGroundingUp(
            ref characterBody,
            CharacterDataAccess.LocalTransform.ValueRO.Rotation);
    }

    public bool CanCollideWithHit(
        ref ThirdPersonCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit)
    {
        return PhysicsUtilities.IsCollidable(hit.Material);
    }

    public bool IsGroundedOnHit(
        ref ThirdPersonCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit,
        int groundingEvaluationType)
    {
        ThirdPersonCharacterComponent characterComponent = CharacterComponent.ValueRO;

        return KinematicCharacterUtilities.Default_IsGroundedOnHit(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            CharacterDataAccess.CharacterBody.ValueRO,
            CharacterDataAccess.CharacterProperties.ValueRO,
            in hit,
            in characterComponent.StepAndSlopeHandling,
            groundingEvaluationType);
    }

    public void OnMovementHit(
        ref ThirdPersonCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref KinematicCharacterHit hit,
        ref float3 remainingMovementDirection,
        ref float remainingMovementLength,
        float3 originalVelocityDirection,
        float hitDistance)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        ThirdPersonCharacterComponent characterComponent = CharacterComponent.ValueRO;

        KinematicCharacterUtilities.Default_OnMovementHit(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterEntity,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            ref characterPosition,
            CharacterDataAccess.VelocityProjectionHits,
            ref hit,
            ref remainingMovementDirection,
            ref remainingMovementLength,
            originalVelocityDirection,
            hitDistance,
            characterComponent.StepAndSlopeHandling.StepHandling,
            characterComponent.StepAndSlopeHandling.MaxStepHeight,
            characterComponent.StepAndSlopeHandling.CharacterWidthForStepGroundingCheck);
    }

    public void OverrideDynamicHitMasses(
        ref ThirdPersonCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref PhysicsMass characterMass,
        ref PhysicsMass otherMass,
        BasicHit hit)
    {
        // Custom mass overrides
    }

    public void ProjectVelocityOnHits(
        ref ThirdPersonCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref float3 velocity,
        ref bool characterIsGrounded,
        ref BasicHit characterGroundHit,
        in DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits,
        float3 originalVelocityDirection)
    {
        ThirdPersonCharacterComponent characterComponent = CharacterComponent.ValueRO;

        KinematicCharacterUtilities.Default_ProjectVelocityOnHits(
            ref velocity,
            ref characterIsGrounded,
            ref characterGroundHit,
            in velocityProjectionHits,
            originalVelocityDirection,
            characterComponent.StepAndSlopeHandling.ConstrainVelocityToGroundPlane,
            in CharacterDataAccess.CharacterBody.ValueRO);
    }

    #endregion
}
