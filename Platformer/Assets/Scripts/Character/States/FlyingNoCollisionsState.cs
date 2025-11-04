using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;

public struct FlyingNoCollisionsState : IPlatformerCharacterState
{
    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PhysicsCollider characterCollider = ref processor.CharacterDataAccess.PhysicsCollider.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        
        processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
        
        KinematicCharacterUtilities.SetCollisionDetectionActive(false, ref characterProperties, ref characterCollider);
        characterBody.IsGrounded = false;
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PhysicsCollider characterCollider = ref processor.CharacterDataAccess.PhysicsCollider.ValueRW;
        
        KinematicCharacterUtilities.SetCollisionDetectionActive(true, ref characterProperties, ref characterCollider);
    }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref float3 characterPosition = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Position;
        
        KinematicCharacterUtilities.Update_Initialize(
            in processor, 
            ref context, 
            ref baseContext, 
            ref characterBody, 
            processor.CharacterDataAccess.CharacterHitsBuffer,
            processor.CharacterDataAccess.DeferredImpulsesBuffer,
            processor.CharacterDataAccess.VelocityProjectionHits,
            deltaTime);
        
        // Movement
        float3 targetVelocity = characterControl.MoveVector * character.FlyingMaxSpeed;
        CharacterControlUtilities.InterpolateVelocityTowardsTarget(ref characterBody.RelativeVelocity, targetVelocity, deltaTime, character.FlyingMovementSharpness);
        characterPosition += characterBody.RelativeVelocity * deltaTime;

        processor.DetectGlobalTransitions(ref context, ref baseContext);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        
        characterRotation = quaternion.identity;
    }

    public void GetCameraParameters(in PlatformerCharacterComponent character, out Entity cameraTarget, out bool calculateUpFromGravity)
    {
        cameraTarget = character.DefaultCameraTargetEntity;
        calculateUpFromGravity = false;
    }

    public void GetMoveVectorFromPlayerInput(in PlatformerPlayerInputs inputs, quaternion cameraRotation, out float3 moveVector)
    {
        PlatformerCharacterProcessor.GetCommonMoveVectorFromPlayerInput(in inputs, cameraRotation, out moveVector);
        float verticalInput = (inputs.JumpHeld ? 1f : 0f) + (inputs.RollHeld ? -1f : 0f);
        moveVector = MathUtilities.ClampToMaxLength(moveVector + (math.mul(cameraRotation, math.up()) * verticalInput), 1f);
    }
}