using System.Collections;
using System.Collections.Generic;

using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

using UnityEngine;

namespace BBG
{
    public struct EntityAirMoveState : IPlatformerCharacterState
    {

        #region IPlatformerCharacterState
        public void GetCameraParameters(in PlatformerCharacterComponent character, out Entity cameraTarget, out bool calculateUpFromGravity)
        {
            cameraTarget = character.DefaultCameraTargetEntity;
            calculateUpFromGravity = true;
        }

        public void GetMoveVectorFromPlayerInput(in PlatformerPlayerInputs inputs, quaternion cameraRotation, out float3 moveVector)
        {
            PlatformerCharacterProcessor.GetCommonMoveVectorFromPlayerInput(in inputs, cameraRotation, out moveVector);
        }

        public void OnStateEnter(CharacterState previousState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
        {
            ref PlatformerCharacterComponent character= ref processor.Character.ValueRW;
            // 状态切换:
            //      站立(GroundMove / AirMove): 使用 StandingGeometry（高胶囊体）。
            //      蹲下(Crouched): 使用 CrouchingGeometry（矮胶囊体，比如高度变成 1米），这样角色才能钻过低矮的通道。
            //      翻滚(Rolling): 使用 RollingGeometry（通常更圆的形状）。
            // 防止卡住: 比如从蹲下变成站立前，代码通常会先检测头顶空间是否足够（CanStandUp），如果足够，再调用 SetCapsuleGeometry 变回站立高度。
            processor.SetCapsuleGeometry(character.StandingGeometry.ToCapsuleGeometry());
        }

        public void OnStateExit(CharacterState nextState, ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
        {
            
        }

        // 频率: 固定（例如每 0.02秒一次）
        // 用途: 处理物理运动、碰撞检测、速度计算。
        // 物理引擎需要稳定的时间间隔来保证模拟的稳定性
        public void OnStatePhysicsUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
        {
            float deltaTime = baseContext.Time.DeltaTime;
            ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
            ref PlatformerCharacterComponent character=ref processor.Character.ValueRW;
            ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
            ref quaternion characterRotation = ref processor.CharacterDataAccess.LocalTransform.ValueRW.Rotation;
            CustomGravity customGravity = processor.CustomGravity.ValueRW;

            // 在每一帧计算任何具体的移动逻辑（如跑步、跳跃、游泳）之前，必须先调用它。它负责准备工作，确保角色处于一个“最新且干净”的状态
            // 做的是第一阶段的工作
            // 主要做三部分内容
            // Phase 1(准备):
            //      “我在哪？我脚下有地吗？我的电梯动了吗？”
            // 
            // State Logic(你的代码):
            //      “基于 Phase 1 的结果，我想跑、跳、还是飞？我要施加多少速度？”
            //      例如：在 GroundMoveState 中，你根据 Phase 1 算出的 IsGrounded 来决定是否允许跳跃。
            //
            // Phase 2(执行):
            //      “好的，你决定了速度。现在我去处理所有的物理碰撞、穿透修正、推力计算，并最终移动角色。”
            //      调用 HandlePhysicsUpdatePhase2。
            processor.HandlePhysicsUpdatePhase1(ref context, ref baseContext, true, true);

            // 通过移动方向*移动系数=移动的加速度
            float3 airAcceleration = characterControl.MoveVector * character.AirAcceleration;
            if (math.lengthsq(airAcceleration) > 0f)
            {
                // 2. 调用工具函数
                // ref characterBody.RelativeVelocity: 直接修改角色的物理速度
                // characterBody.GroundingUp: 告诉函数哪边是“上”（重力反方向），以免改错轴
                CharacterControlUtilities.StandardAirMove(
                    ref characterBody.RelativeVelocity,
                    airAcceleration,
                    character.AirMaxSpeed,
                    characterBody.GroundingUp,
                    deltaTime,
                    false // forceNoMaxSpeedExcess
                );
            }

            // StandardAirMove：（主动力）只负责玩家输入的水平推力（按 W/ A / S / D 的效果）。
            // AccelerateVelocity：(恒定环境力) 负责环境重力（无论你按不按键，地球都在拉你）。
            // ApplyDragToVelocity：(被动阻力) 负责空气阻力（防止速度无限增加，模拟真实物理）。
            // 为什么要分成三步 力的解耦，同时这三步也有要顺序
            // 物理引擎中处理运动的标准管线
            // Gravity
            CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, customGravity.Gravity, deltaTime);

            // Drag
            CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, character.AirDrag);

            processor.HandlePhysicsUpdatePhase2(ref context, ref baseContext, true, true, true, true, true);
            DetectTransitions(ref context, ref baseContext, in processor);
        }

        // 频率: 随帧率变化（例如 60fps, 144fps）。
        // 用途: 处理视觉表现、动画参数、平滑旋转、相机跟随。
        public void OnStateVariableUpdate(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
        {
            // 如果视觉逻辑（如转身）放在 FixedUpdate 里，当帧率高于物理频率时，你会看到角色动作一卡一卡的（Jitter）；反之则会浪费性能。
            // 它主要负责 “看起来怎么样”，而不是 “实际上在哪里”。
        }

        #endregion

        public bool DetectTransitions(ref PlatformerCharacterUpdateContext context, ref KinematicCharacterUpdateContext baseContext, in PlatformerCharacterProcessor processor)
        {
            // 8. 全局转换 (Global Transitions)
            //    检测一些任何状态下都可能触发的转换，比如游泳（进水）、飞行模式等
            //    这通常包含 "State Agnostic" 的逻辑

            ref KinematicCharacterBody characterBody = ref processor.CharacterDataAccess.CharacterBody.ValueRW;
            ref PlatformerCharacterComponent character = ref processor.Character.ValueRW;
            ref PlatformerCharacterControl characterControl = ref processor.CharacterControl.ValueRW;
            ref PlatformerCharacterStateMachine stateMachine = ref processor.StateMachine.ValueRW;

            // 判断发射钩爪
            if (characterControl.RopePressed && RopeSwingState.DetectRopePoints(in baseContext.PhysicsWorld, in processor, out float3 detectedRopeAnchorPoint))
            {
                stateMachine.RopeSwingState.AnchorPoint = detectedRopeAnchorPoint;
                stateMachine.TransitionToState(CharacterState.RopeSwing, ref context, ref baseContext, in processor);
                return true;
            }

            // 判断翻滚
            if (characterControl.RollHeld)
            {
                stateMachine.TransitionToState(CharacterState.Rolling, ref context, ref baseContext, in processor);
                return true;
            }

            // 判断冲刺
            if (characterControl.DashPressed)
            {
                stateMachine.TransitionToState(CharacterState.Dashing, ref context, ref baseContext, in processor);
                return true;
            }

            // 判断是否从空中落地
            if (characterBody.IsGrounded)
            {
                stateMachine.TransitionToState(CharacterState.GroundMove, ref context, ref baseContext, in processor);
                return true;
            }

            // 加速跑
            if (characterControl.SprintHeld && character.HasDetectedMoveAgainstWall)
            {
                stateMachine.TransitionToState(CharacterState.WallRun, ref context, ref baseContext, in processor);
                return true;
            }

            // 
            if (LedgeGrabState.CanGrabLedge(ref context, ref baseContext, in processor, out Entity ledgeEntity, out ColliderCastHit ledgeSurfaceHit))
            {
                stateMachine.TransitionToState(CharacterState.LedgeGrab, ref context, ref baseContext, in processor);
                KinematicCharacterUtilities.SetOrUpdateParentBody(ref baseContext, ref characterBody, ledgeEntity, ledgeSurfaceHit.Position);
                return true;
            }

            // 吸附到墙上攀爬
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
}