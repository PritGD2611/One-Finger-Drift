using UnityEngine;

public class CarCameraFollow : MonoBehaviour
{
    public Transform target;
    public float height = 6f;
    public float distance = 10f;
    public float rotationSmooth = 5f;
    public float positionSmooth = 8f;

    Vector3 currentVelocity;

    void LateUpdate()
    {
        if (!target) return;

        // Smooth rotation follows car
        Quaternion targetRotation = Quaternion.Euler(
            0f,
            target.eulerAngles.y,
            0f
        );

        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            targetRotation,
            rotationSmooth * Time.deltaTime
        );

        // Position stays behind car with SmoothDamp for less jitter
        Vector3 targetPosition =
            target.position
            - target.forward * distance
            + Vector3.up * height;

        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref currentVelocity,
            1f / positionSmooth
        );

        transform.LookAt(target.position + target.forward * 5f);
    }
}
