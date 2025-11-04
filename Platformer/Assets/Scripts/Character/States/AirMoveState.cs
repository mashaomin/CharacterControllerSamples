using Unity.Mathematics;
using Unity.Physics;
using Unity.Entities;
using Unity.CharacterController;

public struct AirMoveState : IPlatformerCharacterState
{
    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        
        processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    { }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        float elapsedTime = (float)baseContext.Time.ElapsedTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;
        
        processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, true);

        // Move
        float3 airAcceleration = characterControl.MoveVector * character.AirAcceleration;
        if (math.lengthsq(airAcceleration) > 0f)
        {
            float3 tmpVelocity = characterBody.RelativeVelocity;
            CharacterControlUtilities.StandardAirMove(ref characterBody.RelativeVelocity, airAcceleration, character.AirMaxSpeed, characterBody.GroundingUp, deltaTime, false);

            // Cancel air acceleration from input if we would hit a non-grounded surface (prevents air-climbing slopes at high air accelerations)
            if (KinematicCharacterUtilities.MovementWouldHitNonGroundedObstruction(
                    in processor, 
                    ref context, 
                    ref baseContext, 
                    processor.CharacterDataAccess.CharacterProperties.ValueRO,
                    processor.CharacterDataAccess.LocalTransform.ValueRO,
                    processor.CharacterDataAccess.CharacterEntity,
                    processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                    characterBody.RelativeVelocity * deltaTime, 
                    out ColliderCastHit hit))
            {
                characterBody.RelativeVelocity = tmpVelocity;
                
                character.HasDetectedMoveAgainstWall = true;
                character.LastKnownWallNormal = hit.SurfaceNormal;
            }
        }
        
        // Jumping
        {
            if (characterControl.JumpPressed)
            {
                // Allow jumping shortly after getting degrounded
                if (character.AllowJumpAfterBecameUngrounded && elapsedTime < character.LastTimeWasGrounded + character.JumpAfterUngroundedGraceTime)
                {
                    CharacterControlUtilities.StandardJump(ref characterBody, characterBody.GroundingUp * character.GroundJumpSpeed, true, characterBody.GroundingUp);
                    character.HeldJumpTimeCounter = 0f;
                }
                // Air jumps
                else if (character.CurrentUngroundedJumps < character.MaxUngroundedJumps)
                {
                    CharacterControlUtilities.StandardJump(ref characterBody, characterBody.GroundingUp * character.AirJumpSpeed, true, characterBody.GroundingUp);
                    character.CurrentUngroundedJumps++;
                }
                // Remember that we wanted to jump before we became grounded
                else
                {
                    character.JumpPressedBeforeBecameGrounded = true;
                }

                character.AllowJumpAfterBecameUngrounded = false;
            }
            
            // Additional jump power when holding jump
            if (character.AllowHeldJumpInAir && characterControl.JumpHeld && character.HeldJumpTimeCounter < character.MaxHeldJumpTime)
            {
                characterBody.RelativeVelocity += characterBody.GroundingUp * character.JumpHeldAcceleration * deltaTime;
            }
        }
        
        // Gravity
        CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, customGravity.Gravity, deltaTime);

        // Drag
        CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, character.AirDrag);

        processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, true, true, true, true, true);

        DetectTransitions(ref context, ref baseContext, in processor);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;
        
        if (math.lengthsq(characterControl.MoveVector) > 0f)
        {
            CharacterControlUtilities.SlerpRotationTowardsDirectionAroundUp(ref characterRotation, deltaTime, math.normalizesafe(characterControl.MoveVector), MathUtilities.GetUpFromRotation(characterRotation), character.AirRotationSharpness);
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
        
        if (characterControl.RopePressed && RopeSwingState.DetectRopePoints(in baseContext.PhysicsWorld, in processor, out float3 detectedRopeAnchorPoint))
        {
            stateMachine.RopeSwingState.AnchorPoint = detectedRopeAnchorPoint;
            stateMachine.TransitionToState(CharacterState.RopeSwing, ref context, ref baseContext, in processor);
            return true;
        }

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

        if (characterControl.SprintHeld && character.HasDetectedMoveAgainstWall)
        {
            stateMachine.TransitionToState(CharacterState.WallRun, ref context, ref baseContext, in processor);
            return true;
        }

        if (LedgeGrabState.CanGrabLedge(ref context, ref baseContext, in processor, out Entity ledgeEntity, out ColliderCastHit ledgeSurfaceHit))
        {
            stateMachine.TransitionToState(CharacterState.LedgeGrab, ref context, ref baseContext, in processor);
            KinematicCharacterUtilities.SetOrUpdateParentBody(ref baseContext, ref characterBody, ledgeEntity, ledgeSurfaceHit.Position); 
            return true;
        }

        if (characterControl.ClimbPressed)
        {
            if (ClimbingState.CanStartClimbing(ref context, ref baseContext, in processor))
            {
                stateMachine.TransitionToState(CharacterState.Climbing, ref context, ref baseContext, in processor);
                return true;
            }
        }

        return processor.DetectGlobalTransitions(ref context, ref baseContext);
    }
}