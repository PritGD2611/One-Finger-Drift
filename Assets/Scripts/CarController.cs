using UnityEngine;
using UnityEngine.SceneManagement;

public class CarController : MonoBehaviour
{
    [Header("Speed")]
    public float baseSpeed = 40f;

    [Header("Turning")]
    public float turnStrength = 65f;
    public float normalTurnMultiplier = 1f;
    public float driftTurnMultiplier = 4f;

    [Header("Drift Physics")]
    public float grip = 0.6f;            // normal grip (high = stable)
    public float driftGrip = 0.7f;       // drift grip (low = more slide)
    public float driftSlideForce = 3f;   // sideways slide force during drift
    public float gravityMultiplier = 8f; // extra gravity for faster falling

    [Header("Ground Check")]
    public LayerMask groundLayer;
    public float groundCheckDistance = 1.5f;

    [Header("Drift Mode")]
    public float driftSpeedMultiplierMax = 1.5f;   // max speed multiplier (1.5 = 50% faster)
    public float driftBoostRate = 0.05f;           // multiplier increase per second
    public float boostPulseAmount = 0.1f;          // extra multiplier every pulse
    public float boostPulseInterval = 2f;          // seconds between boost pulses
    public float doubleTapTime = 0.3f;             // max time between taps for double tap
    public float driftDurationLimit = 3f;          // max seconds drift mode can be active

    Rigidbody rb;
    float turnInput;
    bool driftMode;
    float currentSpeedMultiplier = 1f;
    float boostPulseTimer;
    float driftTimer;

    // Double tap detection
    float lastTapTime;
    bool lastTapWasLeft;
    int consecutiveTaps;
    public int tapsToActivateDrift = 3;  // number of taps on same side to activate drift

    // Current hold state
    bool isHolding;
    float holdDirection; // -1 = left, 1 = right, 0 = not holding
    bool isGrounded = true;

    public bool IsDriftMode => driftMode;
    public float CurrentSpeed => baseSpeed * currentSpeedMultiplier;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Freeze rotation on X and Z to prevent flipping, freeze Y position to stay on ground
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;

        // Prevent clipping through ground
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void FixedUpdate()
    {
        CheckGround();
        ReadInput();
        UpdateDriftBoost();
        Turn();
        ApplyMovement();
        ApplyExtraGravity();
    }

    void Update()
    {
        DetectMultiTap();
        DetectHold();
    }

    void DetectMultiTap()
    {
        bool tapped = false;
        bool tappedLeft = false;

        // Detect tap start and which side
        if (Input.touchCount > 0 && Input.GetTouch(0).phase == TouchPhase.Began)
        {
            tapped = true;
            tappedLeft = Input.GetTouch(0).position.x < Screen.width / 2f;
        }
#if UNITY_EDITOR
        else if (Input.GetMouseButtonDown(0))
        {
            tapped = true;
            tappedLeft = Input.mousePosition.x < Screen.width / 2f;
        }
#endif

        if (tapped)
        {
            float timeSinceLastTap = Time.time - lastTapTime;

            // Check if tap is on the same side and within time window
            if (timeSinceLastTap <= doubleTapTime && tappedLeft == lastTapWasLeft)
            {
                consecutiveTaps++;

                // Quadruple tap detected (4 taps on same side)
                if (consecutiveTaps >= tapsToActivateDrift)
                {
                    // Toggle drift mode
                    driftMode = !driftMode;

                    // Reset multiplier when exiting drift mode
                    if (!driftMode)
                    {
                        currentSpeedMultiplier = 1f;
                        boostPulseTimer = 0f;
                    }

                    consecutiveTaps = 0;
                }
            }
            else
            {
                // Reset tap count - different side or too slow
                consecutiveTaps = 1;
            }

            lastTapTime = Time.time;
            lastTapWasLeft = tappedLeft;
        }
    }

    void DetectHold()
    {
        isHolding = false;
        holdDirection = 0f;

        // Check if screen is being held
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
        turnInput = 0f;

        if (driftMode && isHolding)
        {
            // In drift mode while holding - turn in the hold direction
            turnInput = holdDirection;
        }
        else if (!driftMode && isHolding)
        {
            // Normal mode - direct input for turning
            turnInput = holdDirection;
        }
        // If not holding, turnInput stays 0 (go straight)
    }

    void UpdateDriftBoost()
    {
        if (driftMode)
        {
            // Track drift duration
            driftTimer += Time.fixedDeltaTime;

            // Auto-exit drift mode when time limit reached
            if (driftTimer >= driftDurationLimit)
            {
                driftMode = false;
                currentSpeedMultiplier = 1f;
                boostPulseTimer = 0f;
                driftTimer = 0f;
                return;
            }

            // Continuous speed multiplier increase while in drift mode
            currentSpeedMultiplier += driftBoostRate * Time.fixedDeltaTime;

            // Boost pulse every X seconds for noticeable speed jumps
            boostPulseTimer += Time.fixedDeltaTime;
            if (boostPulseTimer >= boostPulseInterval)
            {
                boostPulseTimer = 0f;
                currentSpeedMultiplier += boostPulseAmount;
            }

            // Cap at max multiplier
            currentSpeedMultiplier = Mathf.Min(currentSpeedMultiplier, driftSpeedMultiplierMax);
        }
        else
        {
            // Reset to normal speed when not in drift mode
            currentSpeedMultiplier = 1f;
            boostPulseTimer = 0f;
            driftTimer = 0f;
        }
    }

    void Turn()
    {
        float turnPower = turnStrength;

        // More turn power in drift mode
        if (driftMode && isHolding)
            turnPower *= driftTurnMultiplier;
        else
            turnPower *= normalTurnMultiplier;

        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, turnInput * turnPower * Time.fixedDeltaTime, 0f));
    }

    void ApplyMovement()
    {
        // Speed = base * multiplier
        float targetSpeed = baseSpeed * currentSpeedMultiplier;

        // Calculate forward velocity
        Vector3 forwardVelocity = transform.forward * targetSpeed;

        // Apply drift/grip - only apply drift physics when in drift mode AND holding
        bool isDrifting = driftMode && isHolding;
        float currentGrip = isDrifting ? driftGrip : grip;
        Vector3 sideVel = transform.right * Vector3.Dot(rb.linearVelocity, transform.right);

        // Calculate base velocity
        Vector3 newVelocity = new Vector3(
            forwardVelocity.x + sideVel.x * currentGrip,
            rb.linearVelocity.y,
            forwardVelocity.z + sideVel.z * currentGrip
        );

        // Add sideways slide force in drift mode when holding (GTA style)
        // Slide in CROSS direction: hold left = slide right, hold right = slide left
        if (isDrifting)
        {
            // Cross direction: opposite of hold direction
            float slideDirection = -holdDirection;
            Vector3 slideForce = transform.right * slideDirection * driftSlideForce;
            newVelocity += new Vector3(slideForce.x, 0f, slideForce.z);
        }

        rb.linearVelocity = newVelocity;
    }

    void ApplyExtraGravity()
    {
        // Apply extra gravity force for faster falling
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
            // Just landed - freeze Y position
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;
        }
        else if (!isGrounded && wasGrounded)
        {
            // Just left the ground - unfreeze Y so car falls
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
