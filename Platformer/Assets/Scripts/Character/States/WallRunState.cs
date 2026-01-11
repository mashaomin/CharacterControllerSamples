using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;
using Unity.Physics;

/// <summary>
/// 飞檐走壁 模式
/// </summary>
public struct WallRunState : IPlatformerCharacterState
{
    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor) { }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;

        processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, true);

        // Detect if still moving against ungrounded surface
        if (KinematicCharacterUtilities.MovementWouldHitNonGroundedObstruction(
                in processor,
                ref context,
                ref baseContext,
                processor.CharacterDataAccess.CharacterProperties.ValueRO,
                processor.CharacterDataAccess.LocalTransform.ValueRO,
                processor.CharacterDataAccess.CharacterEntity,
                processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                -character.LastKnownWallNormal * character.WallRunDetectionDistance,
                out ColliderCastHit detectedHit))
        {
            // 摸到了！确认墙还在，更新墙的法线
            character.HasDetectedMoveAgainstWall = true;
            character.LastKnownWallNormal = detectedHit.SurfaceNormal;
        }
        else
        {
            // 没摸到，墙断了或者离开了
            character.LastKnownWallNormal = default;
        }

        if (character.HasDetectedMoveAgainstWall)
        {
            // 1.计算墙面切线(Constrained Move Direction)
            float3 constrainedMoveDirection = math.normalizesafe(math.cross(character.LastKnownWallNormal, characterBody.GroundingUp));
            
            // 2. 处理玩家输入
            // 先把玩家的输入投影到水平面上
            float3 moveVectorOnPlane = math.normalizesafe(MathUtilities.ProjectOnPlane(characterControl.MoveVector, characterBody.GroundingUp)) * math.length(characterControl.MoveVector);
            float3 acceleration = moveVectorOnPlane * character.WallRunAcceleration;

            // 3. 强制输入约束 (Key Logic!)
            // 将玩家的原始加速度，强制投影到墙面切线上。
            // 这意味着：如果你对着墙跑，或者背着墙跑，最后都会变成“沿着墙跑”。
            acceleration = math.projectsafe(acceleration, constrainedMoveDirection);

            // 4. 应用移动
            CharacterControlUtilities.StandardAirMove(ref characterBody.RelativeVelocity, acceleration, character.WallRunMaxSpeed, characterBody.GroundingUp, deltaTime, false);

            // Jumping
            if (character.HasDetectedMoveAgainstWall && characterControl.JumpPressed)
            {
                float3 jumpDirection = math.normalizesafe(math.lerp(characterBody.GroundingUp, character.LastKnownWallNormal, character.WallRunJumpRatioFromCharacterUp));
                CharacterControlUtilities.StandardJump(ref characterBody, jumpDirection * character.WallRunJumpSpeed, true, jumpDirection);
            }

            if (characterControl.JumpHeld && character.HeldJumpTimeCounter < character.MaxHeldJumpTime)
            {
                characterBody.RelativeVelocity += characterBody.GroundingUp * character.JumpHeldAcceleration * deltaTime;
            }
        }

        // 1. 重力折扣
        // WallRunGravityFactor 通常是 0.1f ~ 0.5f。
        // 让你在墙上感觉“身轻如燕”。
        CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, (customGravity.Gravity * character.WallRunGravityFactor), deltaTime);

        // Drag
        CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, character.WallRunDrag);

        processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, false, true, true, true, true);

        DetectTransitions(ref context, ref baseContext, in processor);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;

        // Orientation
        if (character.HasDetectedMoveAgainstWall)
        {
            float3 rotationDirection = math.normalizesafe(math.cross(character.LastKnownWallNormal, characterBody.GroundingUp));
            if (math.dot(rotationDirection, characterBody.RelativeVelocity) < 0f)
            {
                rotationDirection *= -1f;
            }

            CharacterControlUtilities.SlerpRotationTowardsDirectionAroundUp(ref characterRotation, deltaTime, rotationDirection, characterBody.GroundingUp, character.GroundedRotationSharpness);
        }

        CharacterControlUtilities.SlerpCharacterUpTowardsDirection(ref characterRotation, deltaTime, math.normalizesafe(-customGravity.Gravity), character.UpOrientationAdaptationSharpness);
    }

    public void GetCameraParameters(in PlatformerCharacterComponent character, out Entity cameraTarget, out bool calculateUpFromGravity)
    {
        cameraTarget = character.DefaultCameraTargetEntity;
        calculateUpFromGravity = true;
    }

    public void GetMoveVectorFromPlayerInput(in PlatformerPlayerInputs inputs, quaternion cameraRotation, out float3 moveVector)
    {
        PlatformerCharacterProcessor.GetCommonMoveVectorFromPlayerInput(in inputs, cameraRotation, out moveVector);
    }

    public bool DetectTransitions(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref PlatformerCharacterStateMachine stateMachine = ref processor.StateMachine.ValueRW;

        if (characterControl.RollHeld)
        {
            stateMachine.TransitionToState(CharacterState.Rolling, ref context, ref baseContext, in processor);
            return true;
        }

        if (characterControl.DashPressed)
        {
            stateMachine.TransitionToState(CharacterState.Dashing, ref context, ref baseContext, in processor);
            return true;
        }

        if (characterBody.IsGrounded)
        {
            stateMachine.TransitionToState(CharacterState.GroundMove, ref context, ref baseContext, in processor);
            return true;
        }

        if (!character.HasDetectedMoveAgainstWall)
        {
            stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in processor);
            return true;
        }

        return processor.DetectGlobalTransitions(ref context, ref baseContext);
    }
}
