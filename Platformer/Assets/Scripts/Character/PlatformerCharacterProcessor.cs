using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.CharacterController;
using UnityEngine;
using CapsuleCollider = Unity.Physics.CapsuleCollider;

public struct PlatformerCharacterUpdateContext
{
    public int ChunkIndex;
    public EntityCommandBuffer.ParallelWriter EndFrameECB;
    [ReadOnly]
    public ComponentLookup<CharacterFrictionModifier> CharacterFrictionModifierLookup;
    [ReadOnly]
    public BufferLookup<LinkedEntityGroup> LinkedEntityGroupLookup;

    public void SetChunkIndex(int chunkIndex)
    {
        ChunkIndex = chunkIndex;
    }

    public void OnSystemCreate(ref SystemState state)
    {
        CharacterFrictionModifierLookup = state.GetComponentLookup<CharacterFrictionModifier>(true);
        LinkedEntityGroupLookup = state.GetBufferLookup<LinkedEntityGroup>(true);
    }

    public void OnSystemUpdate(ref SystemState state, EntityCommandBuffer endFrameECB)
    {
        EndFrameECB = endFrameECB.AsParallelWriter();
        CharacterFrictionModifierLookup.Update(ref state);
        LinkedEntityGroupLookup.Update(ref state);
    }
}

public struct PlatformerCharacterProcessor : IKinematicCharacterProcessor<PlatformerCharacterUpdateContext>
{
    public KinematicCharacterDataAccess CharacterDataAccess;
    public RefRW<PlatformerCharacterComponent> Character;
    public RefRW<PlatformerCharacterControl> CharacterControl;
    public RefRW<PlatformerCharacterStateMachine> StateMachine;
    public RefRW<CustomGravity> CustomGravity;

    public void PhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref CharacterControl.ValueRW;
        ref PlatformerCharacterStateMachine stateMachine = ref StateMachine.ValueRW;

        // Common pre-update logic across states
        {
            // Handle initial state transition
            if (stateMachine.CurrentState == CharacterState.Uninitialized)
            {
                stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in this);
            }

            if (characterControl.JumpHeld)
            {
                character.HeldJumpTimeCounter += baseContext.Time.DeltaTime;
            }
            else
            {
                character.HeldJumpTimeCounter = 0f;
                character.AllowHeldJumpInAir = false;
            }
            if (characterControl.JumpPressed)
            {
                character.LastTimeJumpPressed = (float)baseContext.Time.ElapsedTime;
            }
            
            character.HasDetectedMoveAgainstWall = false;
            if (characterBody.IsGrounded)
            {
                character.LastTimeWasGrounded = (float)baseContext.Time.ElapsedTime;
                
                character.CurrentUngroundedJumps = 0;
                character.AllowJumpAfterBecameUngrounded = true;
                character.AllowHeldJumpInAir = true;
            }
            if (character.LedgeGrabBlockCounter > 0f)
            {
                character.LedgeGrabBlockCounter -= baseContext.Time.DeltaTime;
            }
        }
        
        stateMachine.OnStatePhysicsUpdate(stateMachine.CurrentState, ref context, ref baseContext, in this);
        
        // Common post-update logic across states
        {
            character.JumpPressedBeforeBecameGrounded = false;
        }
    }

    public void VariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterStateMachine stateMachine = ref StateMachine.ValueRW;
        ref quaternion characterRotation = ref CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        
        KinematicCharacterUtilities.AddVariableRateRotationFromFixedRateRotation(ref characterRotation, characterBody.RotationFromParent, baseContext.Time.DeltaTime, characterBody.LastPhysicsUpdateDeltaTime);
        stateMachine.OnStateVariableUpdate(stateMachine.CurrentState, ref context, ref baseContext, in this);
    }

    public bool DetectGlobalTransitions(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref PlatformerCharacterStateMachine stateMachine = ref StateMachine.ValueRW;
        ref PlatformerCharacterControl characterControl = ref CharacterControl.ValueRW;
        
        if (stateMachine.CurrentState != CharacterState.Swimming && stateMachine.CurrentState != CharacterState.FlyingNoCollisions)
        {
            if (SwimmingState.DetectWaterZones(ref context, ref baseContext, in this, out float3 tmpDirection, out float tmpDistance))
            {
                if (tmpDistance < 0f)
                {
                    stateMachine.TransitionToState(CharacterState.Swimming, ref context, ref baseContext, in this);
                    return true;
                }
            }
        }

        if (characterControl.FlyNoCollisionsPressed)
        {
            if (stateMachine.CurrentState == CharacterState.FlyingNoCollisions)
            {
                stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in this);
                return true;
            }
            else
            {
                stateMachine.TransitionToState(CharacterState.FlyingNoCollisions, ref context, ref baseContext, in this);
                return true;
            }
        }

        return false;
    }

    public void HandlePhysicsUpdatePhase1(
        ref PlatformerCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        bool allowParentHandling,
        bool allowGroundingDetection)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        
        KinematicCharacterUtilities.Update_Initialize(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.DeferredImpulsesBuffer,
            CharacterDataAccess.VelocityProjectionHits,
            baseContext.Time.DeltaTime);

        if (allowParentHandling)
        {
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
        }
        if (allowGroundingDetection)
        {
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
        }
    }

    public void HandlePhysicsUpdatePhase2(
        ref PlatformerCharacterUpdateContext context, 
        ref KinematicCharacterUpdateContext baseContext,
        bool allowPreventGroundingFromFutureSlopeChange,
        bool allowGroundingPushing,
        bool allowMovementAndDecollisions,
        bool allowMovingPlatformDetection,
        bool allowParentHandling)
    {
        ref PlatformerCharacterComponent character = ref Character.ValueRW;
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        CustomGravity customGravity = CustomGravity.ValueRO;

        if (allowPreventGroundingFromFutureSlopeChange)
        {
            KinematicCharacterUtilities.Update_PreventGroundingFromFutureSlopeChange(
                in this,
                ref context,
                ref baseContext,
                CharacterDataAccess.CharacterEntity,
                ref characterBody,
                CharacterDataAccess.CharacterProperties.ValueRO,
                CharacterDataAccess.PhysicsCollider.ValueRO,
                in character.StepAndSlopeHandling);
        }
        if (allowGroundingPushing)
        {
            KinematicCharacterUtilities.Update_GroundPushing(
                in this,
                ref context,
                ref baseContext,
                ref characterBody,
                CharacterDataAccess.CharacterProperties.ValueRO,
                CharacterDataAccess.LocalTransform.ValueRO,
                CharacterDataAccess.DeferredImpulsesBuffer,
                customGravity.Gravity);
        }
        if (allowMovementAndDecollisions)
        {
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
        }
        if (allowMovingPlatformDetection)
        {
            KinematicCharacterUtilities.Update_MovingPlatformDetection(
                ref baseContext,
                ref characterBody);
        }
        if (allowParentHandling)
        {
            KinematicCharacterUtilities.Update_ParentMomentum(
                ref baseContext,
                ref characterBody,
                CharacterDataAccess.LocalTransform.ValueRO.Position);
        }
        KinematicCharacterUtilities.Update_ProcessStatefulCharacterHits(
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.StatefulHitsBuffer);
    }

    public unsafe void SetCapsuleGeometry(CapsuleGeometry capsuleGeometry)
    {
        ref PhysicsCollider physicsCollider = ref CharacterDataAccess.PhysicsCollider.ValueRW;
        
        CapsuleCollider* capsuleCollider = (CapsuleCollider*)physicsCollider.ColliderPtr;
        capsuleCollider->Geometry = capsuleGeometry;
    }

    public float3 GetGeometryCenter(CapsuleGeometryDefinition geometry)
    {
        float3 characterPosition = CharacterDataAccess.LocalTransform.ValueRW.Position;
        quaternion characterRotation = CharacterDataAccess.LocalTransform.ValueRW.Rotation;

        RigidTransform characterTransform = new RigidTransform(characterRotation, characterPosition);
        float3 geometryCenter = math.transform(characterTransform, geometry.Center);
        return geometryCenter;
    }

    public unsafe bool CanStandUp(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref PhysicsCollider physicsCollider = ref CharacterDataAccess.PhysicsCollider.ValueRW;
        ref PlatformerCharacterComponent character = ref Character.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        ref quaternion characterRotation = ref CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        float characterScale = CharacterDataAccess.LocalTransform.ValueRO.Scale;
        ref KinematicCharacterProperties characterProperties = ref CharacterDataAccess.CharacterProperties.ValueRW;
        
        // Overlap test with standing geometry to see if we have space to stand
        CapsuleCollider* capsuleCollider = ((CapsuleCollider*)physicsCollider.ColliderPtr);

        CapsuleGeometry initialGeometry = capsuleCollider->Geometry;
        capsuleCollider->Geometry = character.StandingGeometry.ToCapsuleGeometry();

        bool isObstructed = KinematicCharacterUtilities.CalculateDistanceClosestCollisions(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            characterPosition,
            characterRotation,
            characterScale,
            0f,
            characterProperties.ShouldIgnoreDynamicBodies(),
            out DistanceHit hit);

        capsuleCollider->Geometry = initialGeometry;

        return !isObstructed;
    }

    public static bool CanBeAffectedByWindZone(CharacterState currentCharacterState)
    {
        if (currentCharacterState == CharacterState.GroundMove ||
            currentCharacterState == CharacterState.AirMove ||
            currentCharacterState == CharacterState.Crouched ||
            currentCharacterState == CharacterState.Rolling)
        {
            return true;
        }

        return false;
    }

    public static CapsuleGeometry CreateCharacterCapsuleGeometry(float radius, float height, bool centered)
    {
        height = math.max(height, radius * 2f);
        float halfHeight = height * 0.5f;

        return new CapsuleGeometry
        {
            Radius = radius,
            Vertex0 = centered ? (-math.up() * (halfHeight - radius)) : (math.up() * radius),
            Vertex1 = centered ? (math.up() * (halfHeight - radius)) : (math.up() * (height - radius)),
        };
    }

    public static void GetCommonMoveVectorFromPlayerInput(in PlatformerPlayerInputs inputs, quaternion cameraRotation, out float3 moveVector)
    {
        moveVector = (math.mul(cameraRotation, math.right()) * inputs.Move.x) + (math.mul(cameraRotation, math.forward()) * inputs.Move.y);
    }
    
    #region Character Processor Callbacks
    public void UpdateGroundingUp(
        ref PlatformerCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        
        KinematicCharacterUtilities.Default_UpdateGroundingUp(
            ref characterBody,
            CharacterDataAccess.LocalTransform.ValueRO.Rotation);
    }
    
    public bool CanCollideWithHit(
        ref PlatformerCharacterUpdateContext context, 
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit)
    {
        return PhysicsUtilities.IsCollidable(hit.Material);
    }

    public bool IsGroundedOnHit(
        ref PlatformerCharacterUpdateContext context, 
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit, 
        int groundingEvaluationType)
    {
        PlatformerCharacterComponent characterComponent = Character.ValueRO;
        
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
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext baseContext,
            ref KinematicCharacterHit hit,
            ref float3 remainingMovementDirection,
            ref float remainingMovementLength,
            float3 originalVelocityDirection,
            float hitDistance)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        PlatformerCharacterComponent characterComponent = Character.ValueRO;
        
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
        ref PlatformerCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref PhysicsMass characterMass,
        ref PhysicsMass otherMass,
        BasicHit hit)
    {
    }

    public void ProjectVelocityOnHits(
        ref PlatformerCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref float3 velocity,
        ref bool characterIsGrounded,
        ref BasicHit characterGroundHit,
        in DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits,
        float3 originalVelocityDirection)
    {
        PlatformerCharacterComponent characterComponent = Character.ValueRO;
        
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

