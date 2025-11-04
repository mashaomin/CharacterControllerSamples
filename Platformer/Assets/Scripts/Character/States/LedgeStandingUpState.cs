using Unity.Entities;
using Unity.CharacterController;
using Unity.Mathematics;

public struct LedgeStandingUpState : IPlatformerCharacterState
{
    public float3 StandingPoint;
    
    private bool ShouldExitState;

    public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
        
        processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
        
        characterBody.RelativeVelocity = default;
        characterBody.IsGrounded = false;

        characterProperties.EvaluateGrounding = false;
        characterProperties.DetectMovementCollisions = false;
        characterProperties.DecollideFromOverlaps = false;

        ShouldExitState = false;
    }

    public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
        ref KinematicCharacterProperties characterProperties = ref processor.CharacterDataAccess.CharacterProperties.ValueRW;
        
        characterProperties.EvaluateGrounding = true;
        characterProperties.DetectMovementCollisions = true;
        characterProperties.DecollideFromOverlaps = true;

        KinematicCharacterUtilities.SetOrUpdateParentBody(ref baseContext, ref characterBody, default, default); 
    }

    public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {
        ref float3 characterPosition = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Position;
        
        processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, false);

        // TODO: root motion standing up

        characterPosition = StandingPoint;
        ShouldExitState = true;
        
        processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, false, false, false, false, true);

        DetectTransitions(ref context, ref baseContext, in processor);
    }

    public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
    {

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
        ref PlatformerCharacterStateMachine stateMachine = ref processor.StateMachine.ValueRW;
        
        if (ShouldExitState)
        {
            if (characterBody.IsGrounded)
            {
                stateMachine.TransitionToState(CharacterState.GroundMove, ref context, ref baseContext, in processor);
                return true;
            }
            else
            {
                stateMachine.TransitionToState(CharacterState.AirMove, ref context, ref baseContext, in processor);
                return true;
            }
        }

        return processor.DetectGlobalTransitions(ref context, ref baseContext);
    }
}