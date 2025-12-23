using System;

using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

namespace Unity.CharacterController
{
    /// <summary>
    /// 接口：IKinematicCharacterProcessor
    ///     Up：头朝哪？
    ///     CanCollide：要不要撞？
    ///     IsGrounded：能不能站？
    ///     OnMovementHit：撞了怎么走？
    ///     ProjectVelocity：被夹住怎么滑？
    ///     OverrideMass：推不推得动？
    /// Interface implemented by structs meant to be passed as parameter to various character update steps in 
    /// order to customize internal character update logic.
    /// </summary>
    /// <typeparam name="C"> The type of the character "context" struct created by the user </typeparam>
    public interface IKinematicCharacterProcessor<C> where C : unmanaged
    {
        /// <summary>
        /// 决定角色当前的“头顶朝哪”
        /// 普通角色：直接用 math.up() 或 transform 的 up，表示永远头朝天。
        /// 你需要在这里设置 characterBody.GroundingUp
        /// Requests that the grounding up direction should be updated.
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        void UpdateGroundingUp(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext);

        /// <summary>
        /// 物理层已经检测到了碰撞，这里决定逻辑上要不要穿过去
        /// 场景:
        ///     穿透队友：如果 hit 是队友实体，返回 false，直接穿过去
        ///     忽略特定物体：比如地上的小草是物理实体，但你想直接走过去。
        /// 返回值：true = 发生物理碰撞（挡住）；false = 忽略碰撞（穿透）。
        /// Determines if a hit can be collided with or not.
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <returns> Return true if the hit can be collided with, return false if not. </returns>
        bool CanCollideWithHit(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext,
            in BasicHit hit);

        /// <summary>
        /// 判定脚下的这个碰撞点，能不能算作“地面”。
        /// 场景：
        ///     坡度限制：这是最常用的。如果 hit.Normal 和 Up 的夹角超过 60 度，返回 false，角色就会判定为未接地并滑下来。
        ///     特殊表面：比如踩在“油面”或“弹力网”上，强制判定为不可站立。
        ///  参数 groundingEvaluationType：告诉你当前是在做哪种检查（是 Step 判定还是普通的 Ground 判定），
        ///  通常传给 KinematicCharacterUtilities.Default_IsGroundedOnHit 即可
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <param name="groundingEvaluationType"> An identifier meant to indicate what type of grounding evaluation is being done at the moment of calling this. </param>
        /// <returns> Returns true if the character is grounded on the specified hit </returns>
        bool IsGroundedOnHit(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext,
            in BasicHit hit,
            int groundingEvaluationType);

        /// <summary>
        /// 角色在移动过程中撞到了东西，决定怎么处理剩余的位移。
        /// 场景：
        ///     普通滑行：撞墙后，把剩余的速度沿着墙面投射，让角色顺着墙滑行（默认行为）。
        ///     反弹：撞到弹球板，把剩余位移反向反射。
        ///     攀爬/吸附：撞墙后直接停住并开始攀爬逻辑。
        ///     上台阶（Step Handling）：在这里检测这是否是个小台阶，如果是，把角色抬上去。
        /// 重要参数：remainingMovementDirection 和 remainingMovementLength。你可以修改它们来改变角色撞墙后的走向
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <param name="remainingMovementDirection"> The direction of the movement vector that remains to be processed </param>
        /// <param name="remainingMovementLength"> The magnitude of the movement vector that remains to be processed </param>
        /// <param name="originalVelocityDirection"> The original direction of the movement vector before any movement projection happened </param>
        /// <param name="hitDistance"> The distance of the detected hit </param>
        void OnMovementHit(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext,
            ref KinematicCharacterHit hit,
            ref float3 remainingMovementDirection,
            ref float remainingMovementLength,
            float3 originalVelocityDirection,
            float hitDistance);

        /// <summary>
        /// Requests that the character velocity be projected on the hits detected so far in the character update.
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        /// <param name="velocity"> The character velocity that needs to be projected </param>
        /// <param name="characterIsGrounded"> Whether the character is grounded or not </param>
        /// <param name="characterGroundHit"> The current effective ground hit of the character </param>
        /// <param name="velocityProjectionHits"> The hits that have been detected so far during the character update </param>
        /// <param name="originalVelocityDirection"> The original velocity direction of the character at the beginning of the character update, before any projection has happened </param>
        void ProjectVelocityOnHits(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext,
            ref float3 velocity,
            ref bool characterIsGrounded,
            ref BasicHit characterGroundHit,
            in DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits,
            float3 originalVelocityDirection);

        /// <summary>
        /// Provides an opportunity to modify the physics masses used to solve impulses between characters and detected hit bodies.
        /// </summary>
        /// <param name="context"> The user context struct holding global data meant to be accessed during the character update </param>
        /// <param name="baseContext"> The built-in context struct holding global data meant to be accessed during the character update </param>
        /// <param name="characterMass"> The mass of the character </param>
        /// <param name="otherMass"> The mass of the other body that we've detected a hit with </param>
        /// <param name="hit"> The evaluated hit with the dynamic body </param>
        void OverrideDynamicHitMasses(
            ref C context,
            ref KinematicCharacterUpdateContext baseContext,
            ref PhysicsMass characterMass,
            ref PhysicsMass otherMass,
            BasicHit hit);
    }
}
