using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;
using Unity.Physics;

public struct LedgeGrabState : IPlatformerCharacterState
{
    bool m_DetectedMustExitLedge;
    float3 m_ForwardHitNormal;

    const float k_CollisionOffset = 0.02f;
    
    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        
        processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
        
        characterProperties.EvaluateGrounding = false;
        characterProperties.DetectMovementCollisions = false;
        characterProperties.DecollideFromOverlaps = false;

        characterBody.RelativeVelocity = float3.zero;
        characterBody.IsGrounded = false;
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        
        if (nextState != CharacterState.LedgeStandingUp)
        {
            characterProperties.EvaluateGrounding = true;
            characterProperties.DetectMovementCollisions = true;
            characterProperties.DecollideFromOverlaps = true;

            KinematicCharacterUtilities.SetOrUpdateParentBody(ref baseContext, ref characterBody, default, default); 
        }

        characterBody.RelativeVelocity = float3.zero;
    }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref float3 characterPosition = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Position;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        
        processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, false);

        m_DetectedMustExitLedge = false;
        characterBody.RelativeVelocity = float3.zero;

        LedgeDetection(
            ref context,
            ref baseContext,
            in processor,
            characterPosition,
            characterRotation,
            out bool ledgeIsValid,
            out ColliderCastHit surfaceHit,
            out ColliderCastHit forwardHit,
            out float3 characterTranslationAtLedgeSurface,
            out bool wouldBeGroundedOnLedgeSurfaceHit,
            out float forwardHitDistance,
            out bool isObstructedAtSurface,
            out bool isObstructedAtCurrentPosition,
            out float upOffsetToPlaceLedgeDetectionPointAtLedgeLevel);

        if (ledgeIsValid && !isObstructedAtSurface)
        {
            m_ForwardHitNormal = forwardHit.SurfaceNormal;

            // Stick to wall
            float3 characterForward = MathUtilities.GetForwardFromRotation(characterRotation);
            characterPosition += characterForward * (forwardHitDistance - k_CollisionOffset);

            // Adjust to ledge height
            characterPosition += characterBody.GroundingUp * (upOffsetToPlaceLedgeDetectionPointAtLedgeLevel - k_CollisionOffset);

            if (math.lengthsq(characterControl.MoveVector) > 0f)
            {
                // Move input
                float3 ledgeDirection = math.normalizesafe(math.cross(surfaceHit.SurfaceNormal, forwardHit.SurfaceNormal));
                float3 moveInputOnLedgeDirection = math.projectsafe(characterControl.MoveVector, ledgeDirection);

                // Check for move obstructions
                float3 targetTranslationAfterMove = characterPosition + (moveInputOnLedgeDirection * character.LedgeMoveSpeed * deltaTime);
                LedgeDetection(
                    ref context,
                    ref baseContext,
                    in processor,
                    targetTranslationAfterMove,
                    characterRotation,
                    out bool afterMoveLedgeIsValid,
                    out ColliderCastHit afterMoveSurfaceHit,
                    out ColliderCastHit afterMoveForwardHit,
                    out float3 afterMoveCharacterTranslationAtLedgeSurface,
                    out bool afterMoveWouldBeGroundedOnLedgeSurfaceHit,
                    out float afterMoveForwardHitDistance,
                    out bool afterMoveIsObstructedAtSurface,
                    out bool afterMoveIsObstructedAtCurrentPosition,
                    out float afterMoveUpOffsetToPlaceLedgeDetectionPointAtLedgeLevel);

                if (afterMoveLedgeIsValid && !afterMoveIsObstructedAtSurface)
                {
                    characterBody.RelativeVelocity = moveInputOnLedgeDirection * character.LedgeMoveSpeed;
            
                    // Apply velocity to position
                    characterPosition += characterBody.RelativeVelocity * baseContext.Time.DeltaTime;
                }
            }
            
            KinematicCharacterUtilities.SetOrUpdateParentBody(ref baseContext, ref characterBody, forwardHit.Entity, forwardHit.Position); 
        }
        else
        {
            m_DetectedMustExitLedge = true;
        }

        // Detect letting go of ledge
        if (characterControl.CrouchPressed || characterControl.DashPressed)
        {
            character.LedgeGrabBlockCounter = 0.3f;
        }

        processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, false, false, false, false, true);

        DetectTransitions(ref context, ref baseContext, in processor);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;

        // Adjust rotation to face current ledge wall
        quaternion targetRotation = quaternion.LookRotationSafe(math.normalizesafe(MathUtilities.ProjectOnPlane(-m_ForwardHitNormal, characterBody.GroundingUp)), characterBody.GroundingUp);
        characterRotation = math.slerp(characterRotation, targetRotation, MathUtilities.GetSharpnessInterpolant(character.LedgeRotationSharpness, deltaTime));
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
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref float3 characterPosition = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Position;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        ref PlatformerCharacterStateMachine stateMachine = ref processor.StateMachine.ValueRW;
        
        if (IsLedgeGrabBlocked(in character) || m_DetectedMustExitLedge)
        {
            stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in processor);
            return true;
        }

        if (characterControl.JumpPressed)
        {
            LedgeDetection(
                ref context,
                ref baseContext,
                in processor,
                characterPosition,
                characterRotation,
                out bool ledgeIsValid,
                out ColliderCastHit surfaceHit,
                out ColliderCastHit forwardHit,
                out float3 characterTranslationAtLedgeSurface,
                out bool wouldBeGroundedOnLedgeSurfaceHit,
                out float forwardHitDistance,
                out bool isObstructedAtSurface,
                out bool isObstructedAtCurrentPosition,
                out float upOffsetToPlaceLedgeDetectionPointAtLedgeLevel);

            if (ledgeIsValid && !isObstructedAtSurface && wouldBeGroundedOnLedgeSurfaceHit)
            {
                stateMachine.LedgeStandingUpState.StandingPoint = surfaceHit.Position;
                stateMachine.TransitionToState(CharacterState.LedgeStandingUp, ref context, ref baseContext, in processor);
                return true;
            }
        }

        return processor.DetectGlobalTransitions(ref context, ref baseContext);
    }

    public static bool IsLedgeGrabBlocked(in PlatformerCharacterComponent character)
    {
        return character.LedgeGrabBlockCounter > 0f;
    }

    public static bool CanGrabLedge(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor, out Entity ledgeEntity, out ColliderCastHit ledgeSurfaceHit)
    {
        ledgeEntity = Entity.Null;
        ledgeSurfaceHit = default;
        
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref float3 characterPosition = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Position;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;

        if (IsLedgeGrabBlocked(in character))
        {
            return false;
        }

        LedgeDetection(
            ref context,
            ref baseContext,
            in processor,
            characterPosition,
            characterRotation,
            out bool ledgeIsValid,
            out ledgeSurfaceHit,
            out ColliderCastHit forwardHit,
            out float3 characterTranslationAtLedgeSurface,
            out bool wouldBeGroundedOnLedgeSurfaceHit,
            out float forwardHitDistance,
            out bool isObstructedAtSurface,
            out bool isObstructedAtCurrentPosition,
            out float upOffsetToPlaceLedgeDetectionPointAtLedgeLevel);

        // Prevent detecting valid grab if going up
        if (math.dot(characterBody.RelativeVelocity, ledgeSurfaceHit.SurfaceNormal) > 0f)
        {
            ledgeIsValid = false;
        }

        if (ledgeIsValid)
        {
            ledgeEntity = ledgeSurfaceHit.Entity;
        }

        return ledgeIsValid && !isObstructedAtSurface;
    }

    public static void LedgeDetection(
        ref PlatformerCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        in PlatformerCharacterProcessor processor,
        float3 atCharacterTranslation,
        quaternion atCharacterRotation,
        out bool ledgeIsValid,
        out ColliderCastHit surfaceHit,
        out ColliderCastHit forwardHit,
        out float3 characterTranslationAtLedgeSurface,
        out bool wouldBeGroundedOnLedgeSurfaceHit,
        out float forwardHitDistance,
        out bool isObstructedAtSurface,
        out bool isObstructedAtCurrentPosition,
        out float upOffsetToPlaceLedgeDetectionPointAtLedgeLevel)
    {
        const float ledgeProbingToleranceOffset = 0.04f;

        ledgeIsValid = false;
        surfaceHit = default;
        forwardHit = default;
        characterTranslationAtLedgeSurface = default;
        wouldBeGroundedOnLedgeSurfaceHit = false;
        forwardHitDistance = -1f;
        isObstructedAtSurface = false;
        isObstructedAtCurrentPosition = false;
        upOffsetToPlaceLedgeDetectionPointAtLedgeLevel = -1f;
        
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        float characterScale = processor.CharacterDataAccess.LocalTransform.ValueRO.Scale;

        float3 currentCharacterForward = MathUtilities.GetForwardFromRotation(atCharacterRotation);
        float3 currentCharacterRight = MathUtilities.GetRightFromRotation(atCharacterRotation);
        RigidTransform currentCharacterRigidTransform = math.RigidTransform(atCharacterRotation, atCharacterTranslation);
        float3 worldSpaceLedgeDetectionPoint = math.transform(currentCharacterRigidTransform, character.LocalLedgeDetectionPoint);
        float forwardDepthOfLedgeDetectionPoint = math.length(math.projectsafe(worldSpaceLedgeDetectionPoint - atCharacterTranslation, currentCharacterForward));

        // Forward detection against the ledge wall
        bool forwardHitDetected = false;
        if (KinematicCharacterUtilities.CastColliderClosestCollisions(
                in processor,
                ref context,
                ref baseContext,
                processor.CharacterDataAccess.CharacterEntity,
                processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                atCharacterTranslation,
                atCharacterRotation,
                characterScale,
                currentCharacterForward,
                forwardDepthOfLedgeDetectionPoint,
                false,
                characterProperties.ShouldIgnoreDynamicBodies(),
                out forwardHit,
                out forwardHitDistance))
        {
            forwardHitDetected = true;

            if (KinematicCharacterUtilities.CalculateDistanceClosestCollisions(
                    in processor,
                    ref context,
                    ref baseContext,
                    processor.CharacterDataAccess.CharacterEntity,
                    processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                    atCharacterTranslation,
                    atCharacterRotation,
                    characterScale,
                    0f,
                    characterProperties.ShouldIgnoreDynamicBodies(),
                    out DistanceHit closestOverlapHit))
            {
                if (closestOverlapHit.Distance <= 0f)
                {
                    isObstructedAtCurrentPosition = true;
                }
            }
        }

        // Cancel rest of detection if no forward hit detected
        if (!forwardHitDetected)
        {
            return;
        }

        // Cancel rest of detection if currently obstructed
        if (isObstructedAtCurrentPosition)
        {
            return;
        }

        // Raycast downward at detectionPoint to find a surface hit
        bool surfaceRaycastHitDetected = false;
        float3 startPointOfSurfaceDetectionRaycast = worldSpaceLedgeDetectionPoint + (characterBody.GroundingUp * character.LedgeSurfaceProbingHeight);
        float surfaceRaycastLength = character.LedgeSurfaceProbingHeight + ledgeProbingToleranceOffset;
        if (KinematicCharacterUtilities.RaycastClosestCollisions(
                in processor,
                ref context,
                ref baseContext,
                processor.CharacterDataAccess.CharacterEntity,
                startPointOfSurfaceDetectionRaycast,
                -characterBody.GroundingUp,
                surfaceRaycastLength,
                characterProperties.ShouldIgnoreDynamicBodies(),
                processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                out RaycastHit surfaceRaycastHit,
                out float surfaceRaycastHitDistance))
        {
            if (surfaceRaycastHit.Fraction > 0f)
            {
                surfaceRaycastHitDetected = true;
            }
        }

        // If no ray hit found, do more raycast tests on the sides
        if (!surfaceRaycastHitDetected)
        {
            float3 rightStartPointOfSurfaceDetectionRaycast = startPointOfSurfaceDetectionRaycast + (currentCharacterRight * character.LedgeSideProbingLength);
            if (KinematicCharacterUtilities.RaycastClosestCollisions(
                    in processor,
                    ref context,
                    ref baseContext,
                    processor.CharacterDataAccess.CharacterEntity,
                    rightStartPointOfSurfaceDetectionRaycast,
                    -characterBody.GroundingUp,
                    surfaceRaycastLength,
                    characterProperties.ShouldIgnoreDynamicBodies(),
                    processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                    out surfaceRaycastHit,
                    out surfaceRaycastHitDistance))
            {
                if (surfaceRaycastHit.Fraction > 0f)
                {
                    surfaceRaycastHitDetected = true;
                }
            }
        }
        if (!surfaceRaycastHitDetected)
        {
            float3 leftStartPointOfSurfaceDetectionRaycast = startPointOfSurfaceDetectionRaycast - (currentCharacterRight * character.LedgeSideProbingLength);
            if (KinematicCharacterUtilities.RaycastClosestCollisions(
                    in processor,
                    ref context,
                    ref baseContext,
                    processor.CharacterDataAccess.CharacterEntity,
                    leftStartPointOfSurfaceDetectionRaycast,
                    -characterBody.GroundingUp,
                    surfaceRaycastLength,
                    characterProperties.ShouldIgnoreDynamicBodies(),
                    processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                    out surfaceRaycastHit,
                    out surfaceRaycastHitDistance))
            {
                if (surfaceRaycastHit.Fraction > 0f)
                {
                    surfaceRaycastHitDetected = true;
                }
            }
        }

        // Cancel rest of detection if no surface raycast hit detected
        if (!surfaceRaycastHitDetected)
        {
            return;
        }

        // Cancel rest of detection if surface hit is dynamic
        if (PhysicsUtilities.IsBodyDynamic(baseContext.PhysicsWorld, surfaceRaycastHit.RigidBodyIndex))
        {
            return;
        }

        ledgeIsValid = true;

        upOffsetToPlaceLedgeDetectionPointAtLedgeLevel = surfaceRaycastLength - surfaceRaycastHitDistance;

        // Note: this assumes that our transform pivot is at the base of our capsule collider
        float3 startPointOfSurfaceObstructionDetectionCast = surfaceRaycastHit.Position + (characterBody.GroundingUp * character.LedgeSurfaceObstructionProbingHeight);

        // Check obstructions at surface hit point
        if (KinematicCharacterUtilities.CastColliderClosestCollisions(
                in processor,
                ref context,
                ref baseContext,
                processor.CharacterDataAccess.CharacterEntity,
                processor.CharacterDataAccess.PhysicsCollider.ValueRO,
                startPointOfSurfaceObstructionDetectionCast,
                atCharacterRotation,
                characterScale,
                -characterBody.GroundingUp,
                character.LedgeSurfaceObstructionProbingHeight + ledgeProbingToleranceOffset,
                false,
                characterProperties.ShouldIgnoreDynamicBodies(),
                
                out surfaceHit,
                out float closestSurfaceObstructionHitDistance))
        {
            if (surfaceHit.Fraction <= 0f)
            {
                isObstructedAtSurface = true;
            }
        }

        // Cancel rest of detection if obstruction at surface
        if (isObstructedAtSurface)
        {
            return;
        }

        // Cancel rest of detection if found no surface hit
        if (surfaceHit.Entity == Entity.Null)
        {
            return;
        }

        characterTranslationAtLedgeSurface = startPointOfSurfaceObstructionDetectionCast + (-characterBody.GroundingUp * closestSurfaceObstructionHitDistance);

        wouldBeGroundedOnLedgeSurfaceHit = processor.IsGroundedOnHit(ref context, ref baseContext, new BasicHit(surfaceHit), 0);
    }
}