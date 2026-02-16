using UnityEngine;

public class CarController_Old : MonoBehaviour
{
    public float speed = 10f;
    public float turnSpeed = 90f;
    public float driftFactor = 0.88f;

    Rigidbody rb;
    float turn;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        turn = 0f;

        if (Input.touchCount > 0)
        {
            if (Input.GetTouch(0).position.x < Screen.width / 2)
                turn = -1f;
            else
                turn = 1f;
        }

        rb.linearVelocity = transform.forward * speed;
        transform.Rotate(Vector3.up, turn * turnSpeed * Time.fixedDeltaTime);

        Vector3 f = transform.forward * Vector3.Dot(rb.linearVelocity, transform.forward);
        Vector3 s = transform.right * Vector3.Dot(rb.linearVelocity, transform.right);
        rb.linearVelocity = f + s * driftFactor;
    }
}
