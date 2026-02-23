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
    public int pathSmoothingSegments = 5;
    public float lookAheadDistance = 5f;
    public float turnAngleThreshold = 15f;
    public float edgeMargin = 1.5f;

    [Header("Recovery Settings")]
    public float offPathThreshold = 2f;
    public float recoverySteerStrength = 1.5f;
    public float stuckSpeedThreshold = 2f;
    public float stuckTimeThreshold = 1f;

    [Header("Track Boundary")]
    public float trackEdgeCheckDistance = 3f;
    public float edgeCorrectionStrength = 2f;
    public float centerTrackStrength = 0.5f;

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

    // Track boundary state
    bool nearTrackEdge;
    Vector3 trackCenterDirection;
    Vector3 centerOfTrackDirection;

    // Finish state
    bool hasFinished;

    // Smoothed path
    Vector3[] smoothedPath;
    int currentPathIndex;
    Vector3 currentSteeringTarget;

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
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;

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
        CheckTrackBoundary();

        // Skip mistakes when recovering or near edge
        if (!isRecovering && !nearTrackEdge)
            CheckForMistake();

        float turnInput = CalculateTurnInput();
        drifting = Mathf.Abs(turnInput) > driftThreshold;

        UpdateDriftBoost();
        Turn(turnInput);
        ApplyMovement();

        // Sync agent position with actual position
        agent.nextPosition = transform.position;
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
            rb.isKinematic = true;

            // Clear path
            if (agent.hasPath)
                agent.ResetPath();

            Debug.Log("AI Car Finished!");
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

    void CheckTrackBoundary()
    {
        nearTrackEdge = false;
        trackCenterDirection = Vector3.zero;
        centerOfTrackDirection = Vector3.zero;

        // Check if we're near the edge of the NavMesh
        NavMeshHit hitLeft, hitRight;

        // Check forward-left
        Vector3 checkDirLeft = (transform.forward + transform.right * -0.5f).normalized;
        bool leftEdge = NavMesh.Raycast(transform.position, transform.position + checkDirLeft * trackEdgeCheckDistance, out hitLeft, NavMesh.AllAreas);

        // Check forward-right  
        Vector3 checkDirRight = (transform.forward + transform.right * 0.5f).normalized;
        bool rightEdge = NavMesh.Raycast(transform.position, transform.position + checkDirRight * trackEdgeCheckDistance, out hitRight, NavMesh.AllAreas);

        // Check direct left for edge distance
        NavMeshHit directHitLeft, directHitRight;
        bool directLeftEdge = NavMesh.Raycast(transform.position, transform.position + transform.right * -trackEdgeCheckDistance * 2f, out directHitLeft, NavMesh.AllAreas);
        bool directRightEdge = NavMesh.Raycast(transform.position, transform.position + transform.right * trackEdgeCheckDistance * 2f, out directHitRight, NavMesh.AllAreas);

        // Calculate center of track by comparing distances to left and right edges
        float leftDist = directLeftEdge ? Vector3.Distance(transform.position, directHitLeft.position) : trackEdgeCheckDistance * 2f;
        float rightDist = directRightEdge ? Vector3.Distance(transform.position, directHitRight.position) : trackEdgeCheckDistance * 2f;

        // Steer toward center: if left is closer, steer right (and vice versa)
        float centerOffset = rightDist - leftDist;
        if (Mathf.Abs(centerOffset) > 0.5f)
        {
            centerOfTrackDirection = transform.right * Mathf.Sign(centerOffset);
        }

        // If raycast hits (returns true), there's an edge in that direction
        if (leftEdge || directLeftEdge && leftDist < trackEdgeCheckDistance)
        {
            // Edge on left, steer right
            nearTrackEdge = true;
            trackCenterDirection += transform.right;
        }

        if (rightEdge || directRightEdge && rightDist < trackEdgeCheckDistance)
        {
            // Edge on right, steer left
            nearTrackEdge = true;
            trackCenterDirection -= transform.right;
        }

        if (nearTrackEdge)
        {
            trackCenterDirection.Normalize();
            // Cancel any ongoing mistakes when near edge
            isMakingMistake = false;
        }
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

            // Generate smoothed path when we get a new path
            if (agent.hasPath && agent.path.corners.Length >= 2)
            {
                GenerateSmoothedPath();
            }
        }

        // Update current steering target along smoothed path
        UpdateSteeringTarget();
    }

    void GenerateSmoothedPath()
    {
        Vector3[] corners = agent.path.corners;

        if (corners.Length < 2)
        {
            smoothedPath = corners;
            return;
        }

        // Simple path - just center each corner point
        smoothedPath = new Vector3[corners.Length];

        for (int i = 0; i < corners.Length; i++)
        {
            Vector3 dir;
            if (i < corners.Length - 1)
                dir = (corners[i + 1] - corners[i]).normalized;
            else
                dir = (corners[i] - corners[i - 1]).normalized;

            smoothedPath[i] = GetCenteredPoint(corners[i], dir);
        }

        currentPathIndex = 0;
    }

    Vector3 GetCenteredPoint(Vector3 point, Vector3 forwardDir)
    {
        // First, make sure we're starting from a valid NavMesh position
        NavMeshHit startHit;
        if (!NavMesh.SamplePosition(point, out startHit, 5f, NavMesh.AllAreas))
        {
            return point;
        }
        point = startHit.position;

        // Calculate perpendicular direction to the path (left/right)
        Vector3 perpendicular = Vector3.Cross(Vector3.up, forwardDir).normalized;
        if (perpendicular == Vector3.zero)
            perpendicular = Vector3.right;

        float maxDist = 25f;

        // Cast ray to find LEFT edge
        Vector3 leftEdgePos = point;
        NavMeshHit leftHit;
        if (NavMesh.Raycast(point, point - perpendicular * maxDist, out leftHit, NavMesh.AllAreas))
        {
            leftEdgePos = leftHit.position;
        }
        else
        {
            leftEdgePos = point - perpendicular * maxDist;
        }

        // Cast ray to find RIGHT edge
        Vector3 rightEdgePos = point;
        NavMeshHit rightHit;
        if (NavMesh.Raycast(point, point + perpendicular * maxDist, out rightHit, NavMesh.AllAreas))
        {
            rightEdgePos = rightHit.position;
        }
        else
        {
            rightEdgePos = point + perpendicular * maxDist;
        }

        // Calculate the CENTER between the two edges
        Vector3 center = (leftEdgePos + rightEdgePos) * 0.5f;

        // Apply edge margin - move edges inward before calculating center
        float trackWidth = Vector3.Distance(leftEdgePos, rightEdgePos);
        if (trackWidth > edgeMargin * 2f)
        {
            // Shrink the usable track width by the margin on each side
            Vector3 leftInner = leftEdgePos + perpendicular * edgeMargin;
            Vector3 rightInner = rightEdgePos - perpendicular * edgeMargin;
            center = (leftInner + rightInner) * 0.5f;
        }

        // Make sure the center point is on the NavMesh
        NavMeshHit centerHit;
        if (NavMesh.SamplePosition(center, out centerHit, 3f, NavMesh.AllAreas))
        {
            return centerHit.position;
        }

        return point;
    }

    Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Catmull-Rom spline formula
        float t2 = t * t;
        float t3 = t2 * t;

        return 0.5f * (
            2f * p1 +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }

    void UpdateSteeringTarget()
    {
        if (smoothedPath == null || smoothedPath.Length == 0)
        {
            currentSteeringTarget = agent.steeringTarget;
            return;
        }

        // Find the closest point on the smoothed path
        float minDist = float.MaxValue;
        int closestIndex = currentPathIndex;

        // Only search forward from current index (don't go backwards)
        int searchEnd = Mathf.Min(currentPathIndex + pathSmoothingSegments * 2, smoothedPath.Length);
        for (int i = currentPathIndex; i < searchEnd; i++)
        {
            float dist = Vector3.Distance(transform.position, smoothedPath[i]);
            if (dist < minDist)
            {
                minDist = dist;
                closestIndex = i;
            }
        }

        currentPathIndex = closestIndex;

        // Look ahead on the path for smoother steering
        float distanceAccumulated = 0f;
        int lookAheadIndex = currentPathIndex;

        for (int i = currentPathIndex; i < smoothedPath.Length - 1; i++)
        {
            distanceAccumulated += Vector3.Distance(smoothedPath[i], smoothedPath[i + 1]);
            lookAheadIndex = i + 1;

            if (distanceAccumulated >= lookAheadDistance)
                break;
        }

        currentSteeringTarget = smoothedPath[lookAheadIndex];
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

        // Use smoothed path target instead of raw NavMesh steering target
        Vector3 steerTarget = (smoothedPath != null && smoothedPath.Length > 0) 
            ? currentSteeringTarget 
            : agent.steeringTarget;

        // Get direction to next point on path
        Vector3 toTarget = (steerTarget - transform.position).normalized;
        float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);
        float turnInput = Mathf.Clamp(angle / 45f, -1f, 1f);

        // Apply center-track bias for smoother turns
        if (centerOfTrackDirection != Vector3.zero && !nearTrackEdge)
        {
            float centerAngle = Vector3.SignedAngle(transform.forward, centerOfTrackDirection, Vector3.up);
            float centerBias = Mathf.Clamp(centerAngle / 90f, -1f, 1f) * centerTrackStrength;
            turnInput += centerBias;
            turnInput = Mathf.Clamp(turnInput, -1f, 1f);
        }

        // Near track edge: apply strong correction to stay on track
        if (nearTrackEdge)
        {
            float edgeAngle = Vector3.SignedAngle(transform.forward, trackCenterDirection, Vector3.up);
            float edgeCorrection = Mathf.Clamp(edgeAngle / 30f, -1f, 1f) * edgeCorrectionStrength;
            turnInput += edgeCorrection;
            turnInput = Mathf.Clamp(turnInput, -1f, 1f);
            return turnInput;
        }

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
        if (!agent.hasPath) return;

        // Use smoothed path target instead of raw NavMesh steering target
        Vector3 steerTarget = (smoothedPath != null && smoothedPath.Length > 0) 
            ? currentSteeringTarget 
            : agent.steeringTarget;

        // Get the direction the car should face (toward the steering target)
        Vector3 targetDirection = (steerTarget - transform.position).normalized;
        targetDirection.y = 0f; // Keep rotation on horizontal plane

        if (targetDirection == Vector3.zero) return;

        // Calculate the target rotation to face the path
        Quaternion targetRotation = Quaternion.LookRotation(targetDirection, Vector3.up);

        // Smoothly rotate toward the target direction 
        float rotationSpeed = turnStrength;
        if (drifting)
            rotationSpeed *= driftTurnMultiplier;

        rb.MoveRotation(Quaternion.RotateTowards(rb.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime));
    }

    void ApplyMovement()
    {
        float targetSpeed = baseSpeed + currentDriftBoost;
        targetSpeed = Mathf.Min(targetSpeed, maxSpeed);

        // Slow down near track edge to avoid flying off
        if (nearTrackEdge)
        {
            targetSpeed *= 0.7f;
        }

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

        // Draw smoothed path only
        if (smoothedPath != null && smoothedPath.Length > 1)
        {
            Gizmos.color = Color.yellow;

            for (int i = 0; i < smoothedPath.Length - 1; i++)
            {
                Gizmos.DrawLine(smoothedPath[i], smoothedPath[i + 1]);
            }
        }
    }
}
