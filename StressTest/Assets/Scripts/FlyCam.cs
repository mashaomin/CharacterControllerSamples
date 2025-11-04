using UnityEngine;
using UnityEngine.InputSystem;

public class FlyCam : MonoBehaviour
{
    public float MaxMoveSpeed = 10f;
    public float MoveSharpness = 10f;
    public float SprintSpeedBoost = 5f;

    public float RotationSpeed = 10f;
    public float RotationSharpness = 999999f;

    float m_PitchAngle = 0f;
    Vector3 m_PlanarForward = Vector3.forward;
    Vector3 m_CurrentMoveVelocity = default;
    Vector2 m_PreviousMousePos = default;

    void Start()
    {
        m_PlanarForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
        m_PitchAngle = Vector3.SignedAngle(m_PlanarForward, transform.forward, transform.right);
    }

    void Update()
    {
        if (Mouse.current.rightButton.wasPressedThisFrame)
        {
            m_PreviousMousePos = Mouse.current.position.ReadValue();
        }

        if (Mouse.current.rightButton.isPressed)
        {
            // Rotation Input
            Vector3 mouseDelta = (Mouse.current.position.ReadValue() - m_PreviousMousePos);
            m_PreviousMousePos = Mouse.current.position.ReadValue();

            // Yaw
            float yawAngleChange = mouseDelta.x * RotationSpeed * Time.deltaTime;
            Quaternion yawRotation = Quaternion.Euler(Vector3.up * yawAngleChange);
            m_PlanarForward = yawRotation * m_PlanarForward;

            // Pitch
            m_PitchAngle += -mouseDelta.y * RotationSpeed * Time.deltaTime;
            m_PitchAngle = Mathf.Clamp(m_PitchAngle, -89f, 89f);
            Quaternion pitchRotation = Quaternion.Euler(Vector3.right * m_PitchAngle);

            // Final rotation
            Quaternion targetRotation = Quaternion.LookRotation(m_PlanarForward, Vector3.up) * pitchRotation;
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, RotationSharpness * Time.deltaTime);

            // Move Input
            Vector3 forwardInput = transform.forward * ((Keyboard.current.wKey.isPressed ? 1f : 0f) + (Keyboard.current.sKey.isPressed ? -1f : 0f));
            Vector3 rightInput = transform.right * ((Keyboard.current.dKey.isPressed ? 1f : 0f) + (Keyboard.current.aKey.isPressed ? -1f : 0f));
            Vector3 upInput = transform.up * ((Keyboard.current.eKey.isPressed ? 1f : 0f) + (Keyboard.current.qKey.isPressed ? -1f : 0f));
            Vector3 directionalInput = Vector3.ClampMagnitude(forwardInput + rightInput + upInput, 1f);

            // Move
            float finalMaxSpeed = MaxMoveSpeed;
            if (Keyboard.current.leftShiftKey.isPressed)
            {
                finalMaxSpeed *= SprintSpeedBoost;
            }

            m_CurrentMoveVelocity = Vector3.Lerp(m_CurrentMoveVelocity, directionalInput * finalMaxSpeed, Mathf.Clamp01(MoveSharpness * Time.deltaTime));
            transform.position += m_CurrentMoveVelocity * Time.deltaTime;
        }
    }
}