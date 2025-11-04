using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.CharacterController;
using UnityEngine;

public struct BasicCharacterUpdateContext
{
    [ReadOnly]
    public ComponentLookup<BouncySurface> BouncySurfaceLookup;

    public void OnSystemCreate(ref SystemState state)
    {
        BouncySurfaceLookup = state.GetComponentLookup<BouncySurface>(true);
    }

    public void OnSystemUpdate(ref SystemState state)
    {
        BouncySurfaceLookup.Update(ref state);
    }
}

public struct BasicCharacterProcessor : IKinematicCharacterProcessor<BasicCharacterUpdateContext>
{
    public KinematicCharacterDataAccess CharacterDataAccess;
    public RefRW<BasicCharacterComponent> CharacterComponent;
    public RefRW<BasicCharacterControl> CharacterControl;

    public void PhysicsUpdate(ref BasicCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref BasicCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        
        // First phase of default character update
        KinematicCharacterUtilities.Update_Initialize(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.DeferredImpulsesBuffer,
            CharacterDataAccess.VelocityProjectionHits,
            baseContext.Time.DeltaTime);

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
        HandleVelocityControl(ref context, ref baseContext);

        // Second phase of default character update
        KinematicCharacterUtilities.Update_PreventGroundingFromFutureSlopeChange(
            in this,
            ref context,
            ref baseContext,
            CharacterDataAccess.CharacterEntity,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.PhysicsCollider.ValueRO,
            in characterComponent.StepAndSlopeHandling);

        KinematicCharacterUtilities.Update_GroundPushing(
            in this,
            ref context,
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.CharacterProperties.ValueRO,
            CharacterDataAccess.LocalTransform.ValueRO,
            CharacterDataAccess.DeferredImpulsesBuffer,
            characterComponent.Gravity);

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

        KinematicCharacterUtilities.Update_MovingPlatformDetection(
            ref baseContext,
            ref characterBody);

        KinematicCharacterUtilities.Update_ParentMomentum(
            ref baseContext,
            ref characterBody,
            CharacterDataAccess.LocalTransform.ValueRO.Position);

        KinematicCharacterUtilities.Update_ProcessStatefulCharacterHits(
            CharacterDataAccess.CharacterHitsBuffer,
            CharacterDataAccess.StatefulHitsBuffer);
    }

    void HandleVelocityControl(ref BasicCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref BasicCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref BasicCharacterControl characterControl = ref CharacterControl.ValueRW;

        // Rotate move input and velocity to take into account parent rotation
        if(characterBody.ParentEntity != Entity.Null)
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

            characterComponent.CurrentJumpsInAir = 0;
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
            
            // Jump in air
            if (characterControl.Jump && characterComponent.CurrentJumpsInAir < characterComponent.MaxJumpsInAir)
            {
                CharacterControlUtilities.StandardJump(ref characterBody, characterBody.GroundingUp * characterComponent.JumpSpeed, true, characterBody.GroundingUp);
                characterComponent.CurrentJumpsInAir++;
            }
            
            // Gravity
            CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, characterComponent.Gravity, deltaTime);

            // Drag
            CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, characterComponent.AirDrag);
        }
    }

    public void VariableUpdate(ref BasicCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref BasicCharacterComponent characterComponent = ref CharacterComponent.ValueRW;
        ref BasicCharacterControl characterControl = ref CharacterControl.ValueRW;
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
    public void UpdateGroundingUp(
        ref BasicCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        
        KinematicCharacterUtilities.Default_UpdateGroundingUp(
            ref characterBody,
            CharacterDataAccess.LocalTransform.ValueRO.Rotation);
    }
    
    public bool CanCollideWithHit(
        ref BasicCharacterUpdateContext context, 
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit)
    {
        if (!PhysicsUtilities.IsCollidable(hit.Material))
        {
            return false;
        }

        BasicCharacterComponent characterComponent = CharacterComponent.ValueRO;

        if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.IgnoreCollisionsTag))
        {
            return false;
        }

        return true;
    }

    public bool IsGroundedOnHit(
        ref BasicCharacterUpdateContext context, 
        ref KinematicCharacterUpdateContext baseContext,
        in BasicHit hit, 
        int groundingEvaluationType)
    {
        BasicCharacterComponent characterComponent = CharacterComponent.ValueRO;
        
        // Ignore grounding
        if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.IgnoreGroundingTag))
        {
            return false;
        }

        // Ignore step handling
        if (characterComponent.StepAndSlopeHandling.StepHandling && PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.IgnoreStepHandlingTag))
        {
            characterComponent.StepAndSlopeHandling.StepHandling = false;
        }
        
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
            ref BasicCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext baseContext,
            ref KinematicCharacterHit hit,
            ref float3 remainingMovementDirection,
            ref float remainingMovementLength,
            float3 originalVelocityDirection,
            float hitDistance)
    {
        ref KinematicCharacterBody characterBody = ref CharacterDataAccess.CharacterBody.ValueRW;
        ref float3 characterPosition = ref CharacterDataAccess.LocalTransform.ValueRW.Position;
        BasicCharacterComponent characterComponent = CharacterComponent.ValueRO;

        // Ignore step handling
        if (characterComponent.StepAndSlopeHandling.StepHandling && PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.IgnoreStepHandlingTag))
        {
            characterComponent.StepAndSlopeHandling.StepHandling = false;
        }
        
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
        ref BasicCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref PhysicsMass characterMass,
        ref PhysicsMass otherMass,
        BasicHit hit)
    {
        BasicCharacterComponent characterComponent = CharacterComponent.ValueRO;
        
        if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.ZeroMassAgainstCharacterTag))
        {
            characterMass.InverseMass = 0f;
            characterMass.InverseInertia = new float3(0f);
            otherMass.InverseMass = 1f;
            otherMass.InverseInertia = new float3(1f);
        }
        if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterComponent.InfiniteMassAgainstCharacterTag))
        {
            characterMass.InverseMass = 1f;
            characterMass.InverseInertia = new float3(1f);
            otherMass.InverseMass = 0f;
            otherMass.InverseInertia = new float3(0f);
        }
    }

    public void ProjectVelocityOnHits(
        ref BasicCharacterUpdateContext context,
        ref KinematicCharacterUpdateContext baseContext,
        ref float3 velocity,
        ref bool characterIsGrounded,
        ref BasicHit characterGroundHit,
        in DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits,
        float3 originalVelocityDirection)
    {
        BasicCharacterComponent characterComponent = CharacterComponent.ValueRO;
        
        var latestHit = velocityProjectionHits[velocityProjectionHits.Length - 1];
        if (context.BouncySurfaceLookup.HasComponent(latestHit.Entity))
        {
            BouncySurface bouncySurface = context.BouncySurfaceLookup[latestHit.Entity];
            velocity = math.reflect(velocity, latestHit.Normal);
            velocity *= bouncySurface.BounceEnergyMultiplier;
        }
        else
        {
            KinematicCharacterUtilities.Default_ProjectVelocityOnHits(
                ref velocity,
                ref characterIsGrounded,
                ref characterGroundHit,
                in velocityProjectionHits,
                originalVelocityDirection,
                characterComponent.StepAndSlopeHandling.ConstrainVelocityToGroundPlane,
                in CharacterDataAccess.CharacterBody.ValueRO);
        }
    }
    #endregion
}
