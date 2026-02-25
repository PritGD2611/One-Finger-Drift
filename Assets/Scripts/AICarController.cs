using UnityEngine;
using System.Collections.Generic;

public class AICarController : MonoBehaviour
{
    [Header("Waypoints")]
    [Tooltip("Parent object containing all waypoint children in order")]
    public Transform waypointsParent;
    [Tooltip("How far ahead to look for steering target")]
    public float lookAheadDistance = 10f;
    [Tooltip("Distance to consider waypoint reached")]
    public float waypointReachDistance = 3f;

    [Header("Speed")]
    public float baseSpeed = 10f;
    public float maxSpeed = 20f;

    [Header("Turning")]
    public float turnStrength = 90f;
    [Tooltip("Angle threshold to start slowing down")]
    public float turnSlowdownAngle = 30f;
    [Tooltip("Speed multiplier at sharp turns")]
    public float turnSlowdownMultiplier = 0.5f;

    [Header("Drift Physics")]
    public float grip = 0.9f;
    public float driftGrip = 0.6f;
    public float driftThreshold = 0.4f;
    public float driftTurnMultiplier = 1.3f;

    [Header("Drift Boost")]
    public float driftBoostRate = 3f;
    public float driftBoostMax = 8f;
    public float boostDecayRate = 5f;

    [Header("Finish")]
    public Transform finishLine;
    public float finishDistance = 3f;

    [Header("AI Mistakes (Optional)")]
    [Range(0f, 1f)]
    public float perfectDriveChance = 0.7f;
    public float mistakeDuration = 0.3f;

    // Components
    Rigidbody rb;

    // Waypoint tracking
    List<Transform> waypoints = new List<Transform>();
    int currentWaypointIndex;
    Vector3 targetPosition;
    bool allWaypointsCompleted;

    // State
    bool drifting;
    float currentDriftBoost;
    bool hasFinished;
    bool isMakingMistake;
    float mistakeTimer;
    float mistakeSteerOffset;

    public bool HasFinished => hasFinished;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void Start()
    {
        // Collect waypoints from parent
        if (waypointsParent != null)
        {
            foreach (Transform child in waypointsParent)
            {
                waypoints.Add(child);
            }
        }

        if (waypoints.Count == 0)
        {
            Debug.LogError("AICarController: No waypoints found! Please assign waypointsParent.");
            enabled = false;
            return;
        }

        // Find closest waypoint to start from
        currentWaypointIndex = FindClosestWaypointIndex();
        UpdateTargetPosition();
    }

    void FixedUpdate()
    {
        if (hasFinished || waypoints.Count == 0) return;

        // Check finish line
        if (finishLine != null && HorizontalDistance(transform.position, finishLine.position) <= finishDistance)
        {
            Finish();
            return;
        }

        // Update waypoint progress
        UpdateWaypointProgress();

        // Calculate steering
        float turnInput = CalculateSteering();

        // Update drift state
        drifting = Mathf.Abs(turnInput) > driftThreshold;
        UpdateDriftBoost();

        // Apply movement
        ApplyTurning(turnInput);
        ApplyMovement(turnInput);
    }

    int FindClosestWaypointIndex()
    {
        float minDist = float.MaxValue;
        int closest = 0;

        for (int i = 0; i < waypoints.Count; i++)
        {
            Vector3 toWP = waypoints[i].position - transform.position;
            toWP.y = 0;
            float dist = toWP.magnitude;

            // Prefer waypoints that are ahead of the car
            float dot = Vector3.Dot(transform.forward, toWP.normalized);

            // Only consider waypoints in front of the car (dot > 0)
            // or fall back to closest if none are ahead
            if (dot > 0 && dist < minDist)
            {
                minDist = dist;
                closest = i;
            }
        }

        // If no waypoint is ahead, just use the closest one
        if (minDist == float.MaxValue)
        {
            for (int i = 0; i < waypoints.Count; i++)
            {
                float dist = HorizontalDistance(transform.position, waypoints[i].position);
                if (dist < minDist)
                {
                    minDist = dist;
                    closest = i;
                }
            }
        }

        return closest;
    }

    void UpdateWaypointProgress()
    {
        if (allWaypointsCompleted) return;

        // Check multiple waypoints per frame in case car is fast and waypoints are close
        bool advanced = true;
        while (advanced && currentWaypointIndex < waypoints.Count)
        {
            advanced = false;

            // Check if we've gone past the last waypoint
            if (currentWaypointIndex >= waypoints.Count)
            {
                allWaypointsCompleted = true;
                return;
            }

            Vector3 waypointPos = waypoints[currentWaypointIndex].position;

            // Method 1: Simple distance check
            float dist = HorizontalDistance(transform.position, waypointPos);
            if (dist <= waypointReachDistance)
            {
                currentWaypointIndex++;
                advanced = true;
                continue;
            }

            // Method 2: "Passed" check - have we gone past this waypoint?
            Vector3 pathDir;
            if (currentWaypointIndex < waypoints.Count - 1)
            {
                pathDir = waypoints[currentWaypointIndex + 1].position - waypointPos;
            }
            else if (finishLine != null)
            {
                // Last waypoint: path direction points toward finish line
                pathDir = finishLine.position - waypointPos;
            }
            else
            {
                pathDir = waypointPos - waypoints[currentWaypointIndex - 1].position;
            }
            pathDir.y = 0;
            pathDir.Normalize();

            Vector3 waypointToCar = transform.position - waypointPos;
            waypointToCar.y = 0;

            float dot = Vector3.Dot(waypointToCar, pathDir);
            if (dot > 0)
            {
                currentWaypointIndex++;
                advanced = true;
            }
        }

        if (currentWaypointIndex >= waypoints.Count)
        {
            allWaypointsCompleted = true;
        }
    }

    float HorizontalDistance(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    void UpdateTargetPosition()
    {
        // If all waypoints completed, drive straight to finish line
        if (allWaypointsCompleted)
        {
            if (finishLine != null)
            {
                Vector3 finish = finishLine.position;
                finish.y = transform.position.y;
                targetPosition = finish;
            }
            return;
        }

        // Walk along the waypoint chain to find a point lookAheadDistance ahead
        float distanceRemaining = lookAheadDistance;
        int lookIndex = currentWaypointIndex;
        Vector3 segStart = transform.position;

        while (lookIndex < waypoints.Count)
        {
            Vector3 segEnd = waypoints[lookIndex].position;
            float segLength = HorizontalDistance(segStart, segEnd);

            if (segLength >= distanceRemaining && segLength > 0.01f)
            {
                float t = distanceRemaining / segLength;
                Vector3 result = Vector3.Lerp(segStart, segEnd, t);
                result.y = transform.position.y;
                targetPosition = result;
                return;
            }

            distanceRemaining -= segLength;
            segStart = segEnd;
            lookIndex++;
        }

        // Past all waypoints, aim at finish line if available
        if (finishLine != null)
        {
            Vector3 finish = finishLine.position;
            finish.y = transform.position.y;
            targetPosition = finish;
        }
        else
        {
            Vector3 lastWP = waypoints[waypoints.Count - 1].position;
            lastWP.y = transform.position.y;
            targetPosition = lastWP;
        }
    }

    float CalculateSteering()
    {
        // Update target position along waypoint chain
        UpdateTargetPosition();

        // Calculate angle to target on horizontal plane
        Vector3 toTarget = targetPosition - transform.position;
        toTarget.y = 0;

        if (toTarget.sqrMagnitude < 0.001f) return 0f;
        toTarget.Normalize();

        float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);

        // Convert to -1 to 1 range
        float steer = Mathf.Clamp(angle / 45f, -1f, 1f);

        // Apply mistakes occasionally
        if (isMakingMistake)
        {
            mistakeTimer -= Time.fixedDeltaTime;
            steer += mistakeSteerOffset;
            steer = Mathf.Clamp(steer, -1f, 1f);

            if (mistakeTimer <= 0)
                isMakingMistake = false;
        }
        else if (Random.value > perfectDriveChance && Mathf.Abs(angle) > 20f)
        {
            // Start a new mistake on turns
            isMakingMistake = true;
            mistakeTimer = mistakeDuration;
            mistakeSteerOffset = Random.Range(-0.3f, 0.3f);
        }

        return steer;
    }

    void ApplyTurning(float turnInput)
    {
        if (Mathf.Abs(turnInput) < 0.01f) return;

        Vector3 toTarget = targetPosition - transform.position;
        toTarget.y = 0;

        if (toTarget.sqrMagnitude < 0.001f) return;
        toTarget.Normalize();

        Quaternion targetRotation = Quaternion.LookRotation(toTarget, Vector3.up);

        float rotationSpeed = turnStrength * Mathf.Abs(turnInput);
        if (drifting)
            rotationSpeed *= driftTurnMultiplier;

        rb.MoveRotation(Quaternion.RotateTowards(rb.rotation, targetRotation, rotationSpeed * Time.fixedDeltaTime));
    }

    void ApplyMovement(float turnInput)
    {
        // Constant speed - matches player car behavior
        float targetSpeed = baseSpeed;

        // Only slow down for upcoming turns
        float upcomingAngle = GetUpcomingTurnAngle();
        if (upcomingAngle > turnSlowdownAngle)
        {
            float slowdown = Mathf.InverseLerp(turnSlowdownAngle, 90f, upcomingAngle);
            targetSpeed *= Mathf.Lerp(1f, turnSlowdownMultiplier, slowdown);
        }

        // Apply velocity directly - no gradual acceleration
        Vector3 forwardVelocity = transform.forward * targetSpeed;

        // Apply grip (allows drifting)
        float currentGrip = drifting ? driftGrip : grip;
        Vector3 sideVelocity = transform.right * Vector3.Dot(rb.linearVelocity, transform.right);

        rb.linearVelocity = new Vector3(
            forwardVelocity.x + sideVelocity.x * (1f - currentGrip),
            rb.linearVelocity.y,
            forwardVelocity.z + sideVelocity.z * (1f - currentGrip)
        );
    }

    float GetUpcomingTurnAngle()
    {
        if (currentWaypointIndex >= waypoints.Count - 1) return 0f;

        // Get direction to current waypoint
        Vector3 toCurrentWP = (waypoints[currentWaypointIndex].position - transform.position).normalized;

        // Get direction to next waypoint from current
        int nextIndex = Mathf.Min(currentWaypointIndex + 1, waypoints.Count - 1);
        Vector3 toNextWP = (waypoints[nextIndex].position - waypoints[currentWaypointIndex].position).normalized;

        return Vector3.Angle(toCurrentWP, toNextWP);
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

    void Finish()
    {
        hasFinished = true;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = true;
        Debug.Log("AI Car Finished!");
    }

    // Reset for new race
    public void ResetRace(Vector3 startPosition, Quaternion startRotation)
    {
        hasFinished = false;
        rb.isKinematic = false;
        transform.position = startPosition;
        transform.rotation = startRotation;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        currentDriftBoost = 0f;
        allWaypointsCompleted = false;
        currentWaypointIndex = FindClosestWaypointIndex();
    }

    // Visualize waypoints and path in editor
    void OnDrawGizmos()
    {
        // Draw waypoints
        if (waypointsParent != null)
        {
            Gizmos.color = Color.cyan;
            Transform prev = null;

            foreach (Transform wp in waypointsParent)
            {
                Gizmos.DrawWireSphere(wp.position, 0.5f);

                if (prev != null)
                {
                    Gizmos.DrawLine(prev.position, wp.position);
                }
                prev = wp;
            }
        }

        // Draw current target
        if (Application.isPlaying)
        {
            Vector3 drawTarget = new Vector3(targetPosition.x, transform.position.y + 0.5f, targetPosition.z);
            Vector3 drawCar = transform.position + Vector3.up * 0.5f;

            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(drawTarget, 1f);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(drawCar, drawTarget);
        }
    }
}
