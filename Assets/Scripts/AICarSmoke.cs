using UnityEngine;

public class AICarSmoke : MonoBehaviour
{
    [Header("Smoke Setup")]
    [Tooltip("Transforms where smoke particles should appear (e.g., wheel positions)")]
    public Transform[] smokePositions;
    
    [Tooltip("The smoke particle system prefab to use")]
    public ParticleSystem smokeParticlePrefab;

    [Header("Settings")]
    public float minSpeedForSmoke = 2f;
    public float turnThreshold = 0.3f;

    AICarController aiController;
    Rigidbody rb;
    ParticleSystem[] smokeParticles;
    GameObject smokeContainer;
    bool isSmoking;

    void Start()
    {
        aiController = GetComponent<AICarController>();
        rb = GetComponent<Rigidbody>();
        
        if (smokeParticlePrefab == null)
        {
            Debug.LogError("AICarSmoke: No smoke particle prefab assigned!");
            return;
        }

        if (smokePositions == null || smokePositions.Length == 0)
        {
            Debug.LogError("AICarSmoke: No smoke positions assigned!");
            return;
        }

        CreateSmokeObjects();
    }

    void CreateSmokeObjects()
    {
        // Create parent container
        smokeContainer = new GameObject("Smoke");
        smokeContainer.transform.SetParent(transform);
        smokeContainer.transform.localPosition = Vector3.zero;
        smokeContainer.transform.localRotation = Quaternion.identity;

        // Create smoke particles at each position
        smokeParticles = new ParticleSystem[smokePositions.Length];

        for (int i = 0; i < smokePositions.Length; i++)
        {
            // Create child object at smoke position
            GameObject smokeObj = new GameObject($"Smoke_{i}");
            smokeObj.transform.SetParent(smokeContainer.transform);
            smokeObj.transform.position = smokePositions[i].position;
            smokeObj.transform.rotation = smokePositions[i].rotation;

            // Make it follow the smoke position transform
            AICarSmokeFollower follower = smokeObj.AddComponent<AICarSmokeFollower>();
            follower.targetTransform = smokePositions[i];

            // Instantiate particle system as child
            ParticleSystem smoke = Instantiate(smokeParticlePrefab, smokeObj.transform);
            smoke.transform.localPosition = Vector3.zero;
            smoke.transform.localRotation = Quaternion.identity;
            
            // Stop initially
            smoke.Stop();
            
            smokeParticles[i] = smoke;
        }
    }

    void Update()
    {
        if (aiController == null || smokeParticles == null || rb == null) return;

        // Check if AI has finished
        if (aiController.HasFinished)
        {
            if (isSmoking) StopSmoke();
            return;
        }

        // Check if AI is drifting/turning
        bool isDrifting = IsDrifting();

        // Get current speed
        float speed = rb.linearVelocity.magnitude;
        bool isMoving = speed >= minSpeedForSmoke;

        // Smoke when moving AND not drifting (not turning)
        bool shouldSmoke = isMoving && !isDrifting;

        if (shouldSmoke && !isSmoking)
        {
            StartSmoke();
        }
        else if (!shouldSmoke && isSmoking)
        {
            StopSmoke();
        }
    }

    bool IsDrifting()
    {
        if (aiController == null) return false;

        // Calculate turn amount based on angular velocity or side slip
        float sideSlip = Vector3.Dot(rb.linearVelocity, transform.right);
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);

        // If there's significant side slip relative to forward speed, AI is drifting
        if (forwardSpeed > 0.1f)
        {
            float slipRatio = Mathf.Abs(sideSlip) / forwardSpeed;
            if (slipRatio > turnThreshold)
                return true;
        }

        // Also check angular velocity for turning
        if (Mathf.Abs(rb.angularVelocity.y) > 0.5f)
            return true;

        return false;
    }

    void StartSmoke()
    {
        isSmoking = true;
        foreach (ParticleSystem ps in smokeParticles)
        {
            if (ps != null && !ps.isPlaying)
                ps.Play();
        }
    }

    void StopSmoke()
    {
        isSmoking = false;
        foreach (ParticleSystem ps in smokeParticles)
        {
            if (ps != null && ps.isPlaying)
                ps.Stop();
        }
    }

    void OnDestroy()
    {
        if (smokeContainer != null)
            Destroy(smokeContainer);
    }
}

// Helper component to make smoke follow a transform
public class AICarSmokeFollower : MonoBehaviour
{
    public Transform targetTransform;

    void LateUpdate()
    {
        if (targetTransform != null)
        {
            transform.position = targetTransform.position;
            transform.rotation = targetTransform.rotation;
        }
    }
}
