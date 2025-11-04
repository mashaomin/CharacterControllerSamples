using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Rendering;
using Unity.Transforms;

public struct RollingState : IPlatformerCharacterState
{
    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        
        processor.SetCapsuleGeometry(character.RollingGeometry.ToCapsuleGeometry());
        characterProperties.EvaluateGrounding = false;
        characterBody.IsGrounded = false;

        PlatformerUtilities.SetEntityHierarchyEnabledParallel(true, character.RollballMeshEntity, context.EndFrameECB, context.ChunkIndex, context.LinkedEntityGroupLookup);
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;

        characterProperties.EvaluateGrounding = true;

        PlatformerUtilities.SetEntityHierarchyEnabledParallel(false, character.RollballMeshEntity, context.EndFrameECB, context.ChunkIndex, context.LinkedEntityGroupLookup);
    }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;

        processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, false);

        CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, characterControl.MoveVector * character.RollingAcceleration, deltaTime);
        CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, customGravity.Gravity, deltaTime);
        
        processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, false, false, true, false, true);

        DetectTransitions(ref context, ref baseContext, in processor);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        float deltaTime = baseContext.Time.DeltaTime;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
        CustomGravity customGravity = processor.CustomGravity.ValueRO;

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
        ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
        ref PlatformerCharacterStateMachine stateMachine = ref processor.StateMachine.ValueRW;
        
        if (!characterControl.RollHeld && processor.CanStandUp(ref context, ref baseContext))
        {
            stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in processor);
            return true;
        }

        return processor.DetectGlobalTransitions(ref context, ref baseContext);
    }
}