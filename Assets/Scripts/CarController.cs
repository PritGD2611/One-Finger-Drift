using UnityEngine;
using UnityEngine.SceneManagement;

public class CarController : MonoBehaviour
{
    [Header("Speed")]
    public float baseSpeed = 40f;

    [Header("Turning")]
    public float turnStrength = 65f;

    [Header("Physics")]
    public float grip = 0.6f;
    public float gravityMultiplier = 8f;

    [Header("Ground Check")]
    public LayerMask groundLayer;
    public float groundCheckDistance = 1.5f;

    Rigidbody rb;
    float turnInput;

    // Current hold state
    bool isHolding;
    float holdDirection;
    bool isGrounded = true;

    public float CurrentSpeed => baseSpeed;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void FixedUpdate()
    {
        CheckGround();
        ReadInput();
        Turn();
        ApplyMovement();
        ApplyExtraGravity();
    }

    void Update()
    {
        DetectHold();
    }

    void DetectHold()
    {
        isHolding = false;
        holdDirection = 0f;

        if (Input.touchCount > 0)
        {
            isHolding = true;
            holdDirection = Input.GetTouch(0).position.x < Screen.width / 2f ? -1f : 1f;
        }
#if UNITY_EDITOR
        else if (Input.GetMouseButton(0))
        {
            isHolding = true;
            holdDirection = Input.mousePosition.x < Screen.width / 2f ? -1f : 1f;
        }
#endif
    }

    void ReadInput()
    {
        turnInput = isHolding ? holdDirection : 0f;
    }

    void Turn()
    {
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, turnInput * turnStrength * Time.fixedDeltaTime, 0f));
    }

    void ApplyMovement()
    {
        Vector3 forwardVelocity = transform.forward * baseSpeed;

        Vector3 sideVel = transform.right * Vector3.Dot(rb.linearVelocity, transform.right);

        rb.linearVelocity = new Vector3(
            forwardVelocity.x + sideVel.x * grip,
            rb.linearVelocity.y,
            forwardVelocity.z + sideVel.z * grip
        );
    }

    void ApplyExtraGravity()
    {
        if (gravityMultiplier > 1f)
        {
            Vector3 extraGravity = Physics.gravity * (gravityMultiplier - 1f);
            rb.AddForce(extraGravity, ForceMode.Acceleration);
        }
    }

    void CheckGround()
    {
        bool wasGrounded = isGrounded;
        isGrounded = Physics.Raycast(transform.position, Vector3.down, groundCheckDistance, groundLayer);

        if (isGrounded && !wasGrounded)
        {
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;
        }
        else if (!isGrounded && wasGrounded)
        {
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            rb.useGravity = true;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("FinishLine"))
        {
            SceneManager.LoadScene("Levels");
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("FinishLine"))
        {
            SceneManager.LoadScene("Levels");
        }
    }
}
