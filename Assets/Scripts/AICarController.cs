using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public class AICarController : MonoBehaviour
{
    [Header("Speed (Match Player)")]
    public float baseSpeed = 10f;
    public float maxSpeed = 20f;

    [Header("Turning")]
    public float turnStrength = 45f;
    public float driftTurnMultiplier = 1.2f;

    [Header("Drift Physics")]
    public float grip = 0.85f;
    public float driftGrip = 0.6f;
    public float driftThreshold = 0.3f;

    [Header("Drift Boost")]
    public float driftBoostRate = 3f;
    public float driftBoostMax = 8f;
    public float boostDecayRate = 5f;

    [Header("Navigation")]
    public Transform target;
    public float pathUpdateInterval = 0.5f;
    public float finishLineDistance = 3f;

    [Header("Recovery Settings")]
    public float offPathThreshold = 2f;
    public float recoverySteerStrength = 1.5f;
    public float stuckSpeedThreshold = 2f;
    public float stuckTimeThreshold = 1f;

    [Header("Fall & Respawn")]
    public Transform playerTransform;
    public float respawnDistanceBehindPlayer = 120f;
    public string obstacleTag = "Obstacle";

    [Header("AI Mistake Settings")]
    [Range(0f, 1f)]
    public float perfectTurnChance = 0.6f;
    public float mistakeCheckInterval = 1f;
    public float mistakeDelayMin = 0.2f;
    public float mistakeDelayMax = 0.5f;
    public float overDriftMultiplier = 1.5f;
    public float underSteerMultiplier = 0.5f;
    [Range(0f, 1f)]
    public float mistakeSpeedMultiplier = 0.5f;

    Rigidbody rb;
    NavMeshAgent agent;
    bool drifting;
    float pathUpdateTimer;
    float mistakeCheckTimer;
    float currentDriftBoost;

    // Mistake state
    bool isMakingMistake;
    MistakeType currentMistake;
    float mistakeTimer;
    float mistakeDuration;

    // Recovery state
    bool isRecovering;
    float stuckTimer;
    Vector3 lastPosition;

    // Finish state
    bool hasFinished;

    public bool HasFinished => hasFinished;

    enum MistakeType
    {
        None,
        DelayedTurn,
        OverDrift,
        UnderSteer
    }

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Prevent clipping through ground
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        agent = GetComponent<NavMeshAgent>();

        // Use agent only for pathfinding, not movement
        agent.updatePosition = false;
        agent.updateRotation = false;
    }

    void FixedUpdate()
    {
        if (target == null) return;

        // Stop if finished
        if (hasFinished) return;

        // Check if reached finish line
        CheckFinishLine();
        if (hasFinished) return;

        UpdatePath();
        CheckRecovery();

        // Skip mistakes when recovering
        if (!isRecovering)
            CheckForMistake();

        float turnInput = CalculateTurnInput();
        drifting = Mathf.Abs(turnInput) > driftThreshold;

        UpdateDriftBoost();
        Turn(turnInput);
        ApplyMovement();

        // Sync agent position with actual position
        agent.nextPosition = transform.position;
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag(obstacleTag))
        {
            Debug.Log("AI hit obstacle: " + collision.gameObject.name);
            RespawnBehindPlayer();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(obstacleTag))
        {
            Debug.Log("AI triggered obstacle: " + other.gameObject.name);
            RespawnBehindPlayer();
        }
    }

    void RespawnBehindPlayer()
    {
        if (playerTransform == null)
        {
            Debug.LogError("AICarController: Player Transform not assigned! Cannot respawn.");
            return;
        }

        Debug.Log("AI Respawning behind player...");

        // Calculate position behind player
        Vector3 respawnPosition = playerTransform.position - playerTransform.forward * respawnDistanceBehindPlayer;

        // Find nearest point on NavMesh
        NavMeshHit hit;
        if (NavMesh.SamplePosition(respawnPosition, out hit, 50f, NavMesh.AllAreas))
        {
            respawnPosition = hit.position;
        }

        // Respawn AI
        transform.position = respawnPosition;
        transform.rotation = playerTransform.rotation;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Reset states
        isRecovering = false;
        isMakingMistake = false;
        stuckTimer = 0f;
        currentDriftBoost = 0f;

        // Sync agent
        agent.nextPosition = transform.position;
        if (target != null)
            agent.SetDestination(target.position);
    }

    void CheckFinishLine()
    {
        if (target == null) return;

        float distanceToTarget = Vector3.Distance(transform.position, target.position);

        if (distanceToTarget <= finishLineDistance)
        {
            hasFinished = true;

            // Stop the car completely
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // Clear path
            if (agent.hasPath)
                agent.ResetPath();
        }
    }

    void CheckRecovery()
    {
        if (!agent.hasPath) return;

        // Check if stuck (low speed for too long)
        float currentSpeed = rb.linearVelocity.magnitude;
        if (currentSpeed < stuckSpeedThreshold)
        {
            stuckTimer += Time.fixedDeltaTime;
            if (stuckTimer >= stuckTimeThreshold)
            {
                isRecovering = true;
            }
        }
        else
        {
            stuckTimer = 0f;
        }

        // Check distance from ideal path
        NavMeshHit hit;
        if (NavMesh.SamplePosition(transform.position, out hit, 10f, NavMesh.AllAreas))
        {
            // Find closest point on path
            float distanceFromPath = GetDistanceFromPath();

            if (distanceFromPath > offPathThreshold)
            {
                isRecovering = true;
            }
            else if (distanceFromPath < offPathThreshold * 0.5f && stuckTimer == 0f)
            {
                // Back to center, stop recovering
                isRecovering = false;
            }
        }

        lastPosition = transform.position;
    }

    float GetDistanceFromPath()
    {
        if (!agent.hasPath || agent.path.corners.Length < 2) return 0f;

        Vector3[] corners = agent.path.corners;
        float minDistance = float.MaxValue;

        // Find closest distance to any path segment
        for (int i = 0; i < corners.Length - 1; i++)
        {
            Vector3 closest = ClosestPointOnLineSegment(corners[i], corners[i + 1], transform.position);
            float dist = Vector3.Distance(transform.position, closest);
            if (dist < minDistance)
                minDistance = dist;
        }

        return minDistance;
    }

    Vector3 ClosestPointOnLineSegment(Vector3 a, Vector3 b, Vector3 point)
    {
        Vector3 ab = b - a;
        float t = Mathf.Clamp01(Vector3.Dot(point - a, ab) / Vector3.Dot(ab, ab));
        return a + t * ab;
    }

    void UpdatePath()
    {
        pathUpdateTimer += Time.fixedDeltaTime;

        if (pathUpdateTimer >= pathUpdateInterval)
        {
            pathUpdateTimer = 0f;
            agent.SetDestination(target.position);
        }
    }

    void CheckForMistake()
    {
        if (isMakingMistake) return;

        mistakeCheckTimer += Time.fixedDeltaTime;

        if (mistakeCheckTimer >= mistakeCheckInterval)
        {
            mistakeCheckTimer = 0f;

            // Check if we need to turn significantly
            if (agent.hasPath && agent.path.corners.Length > 1)
            {
                Vector3 toNextCorner = (agent.steeringTarget - transform.position).normalized;
                float turnAngle = Vector3.SignedAngle(transform.forward, toNextCorner, Vector3.up);

                // Only consider mistakes on actual turns
                if (Mathf.Abs(turnAngle) > 20f)
                {
                    DecideMistake();
                }
            }
        }
    }

    void DecideMistake()
    {
        float roll = Random.value;

        if (roll > perfectTurnChance)
        {
            isMakingMistake = true;
            mistakeTimer = 0f;

            int mistakeRoll = Random.Range(0, 3);
            switch (mistakeRoll)
            {
                case 0:
                    currentMistake = MistakeType.DelayedTurn;
                    mistakeDuration = Random.Range(mistakeDelayMin, mistakeDelayMax);
                    break;
                case 1:
                    currentMistake = MistakeType.OverDrift;
                    mistakeDuration = Random.Range(0.3f, 0.6f);
                    break;
                case 2:
                    currentMistake = MistakeType.UnderSteer;
                    mistakeDuration = Random.Range(0.3f, 0.6f);
                    break;
            }
        }
    }

    void UpdateDriftBoost()
    {
        if (drifting)
        {
            currentDriftBoost += driftBoostRate * Time.fixedDeltaTime;
            currentDriftBoost = Mathf.Min(currentDriftBoost, driftBoostMax);
        }
        else
        {
            currentDriftBoost -= boostDecayRate * Time.fixedDeltaTime;
            currentDriftBoost = Mathf.Max(currentDriftBoost, 0f);
        }
    }

    float CalculateTurnInput()
    {
        if (!agent.hasPath) return 0f;

        // Get direction to next point on path
        Vector3 toTarget = (agent.steeringTarget - transform.position).normalized;
        float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
        float turnInput = Mathf.Clamp(angle / 45f, -1f, 1f);

        // Recovery mode: stronger steering to get back on track
        if (isRecovering)
        {
            turnInput *= recoverySteerStrength;
            turnInput = Mathf.Clamp(turnInput, -1f, 1f);
            // Cancel any mistakes during recovery
            isMakingMistake = false;
            return turnInput;
        }

        // Apply mistake modifiers
        if (isMakingMistake)
        {
            mistakeTimer += Time.fixedDeltaTime;

            switch (currentMistake)
            {
                case MistakeType.DelayedTurn:
                    if (mistakeTimer < mistakeDuration)
                        turnInput = 0f;
                    else
                        isMakingMistake = false;
                    break;

                case MistakeType.OverDrift:
                    if (mistakeTimer < mistakeDuration)
                        turnInput *= overDriftMultiplier;
                    else
                        isMakingMistake = false;
                    break;

                case MistakeType.UnderSteer:
                    if (mistakeTimer < mistakeDuration)
                        turnInput *= underSteerMultiplier;
                    else
                        isMakingMistake = false;
                    break;
            }
        }

        return turnInput;
    }

    void Turn(float turnInput)
    {
        float turnPower = turnStrength;

        if (drifting)
            turnPower *= driftTurnMultiplier;

        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, turnInput * turnPower * Time.fixedDeltaTime, 0f));
    }

    void ApplyMovement()
    {
        float targetSpeed = baseSpeed + currentDriftBoost;
        targetSpeed = Mathf.Min(targetSpeed, maxSpeed);

        // Reduce speed when making a mistake (half speed)
        if (isMakingMistake)
        {
            targetSpeed *= mistakeSpeedMultiplier;
        }

        Vector3 forwardVelocity = transform.forward * targetSpeed;

        float currentGrip = drifting ? driftGrip : grip;
        Vector3 sideVel = transform.right * Vector3.Dot(rb.linearVelocity, transform.right);

        rb.linearVelocity = new Vector3(
            forwardVelocity.x + sideVel.x * currentGrip,
            rb.linearVelocity.y,
            forwardVelocity.z + sideVel.z * currentGrip
        );
    }

    void OnDrawGizmos()
    {
        if (agent == null || !agent.hasPath) return;

        // Draw path
        if (isRecovering)
            Gizmos.color = Color.yellow;
        else if (isMakingMistake)
            Gizmos.color = Color.red;
        else
            Gizmos.color = Color.cyan;

        Vector3[] corners = agent.path.corners;
        for (int i = 0; i < corners.Length - 1; i++)
        {
            Gizmos.DrawLine(corners[i], corners[i + 1]);
        }

        // Draw steering target
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(agent.steeringTarget, 1f);
    }
}
