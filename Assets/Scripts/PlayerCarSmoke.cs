using UnityEngine;

public class PlayerCarSmoke : MonoBehaviour
{
    [Header("Smoke Setup")]
    [Tooltip("Transforms where smoke particles should appear (e.g., wheel positions)")]
    public Transform[] smokePositions;
    
    [Tooltip("The smoke particle system prefab to use")]
    public ParticleSystem smokeParticlePrefab;

    [Header("Settings")]
    public float minSpeedForSmoke = 2f;

    CarController carController;
    ParticleSystem[] smokeParticles;
    GameObject smokeContainer;
    bool isSmoking;

    void Start()
    {
        carController = GetComponent<CarController>();
        
        if (smokeParticlePrefab == null)
        {
            Debug.LogError("PlayerCarSmoke: No smoke particle prefab assigned!");
            return;
        }

        if (smokePositions == null || smokePositions.Length == 0)
        {
            Debug.LogError("PlayerCarSmoke: No smoke positions assigned!");
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
            SmokeFollower follower = smokeObj.AddComponent<SmokeFollower>();
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
        if (carController == null || smokeParticles == null) return;

        // Get current speed
        Rigidbody rb = carController.GetComponent<Rigidbody>();
        float speed = rb != null ? rb.linearVelocity.magnitude : 0f;
        bool isMoving = speed >= minSpeedForSmoke;

        // Check if in drift mode
        bool isDriftMode = carController.IsDriftMode;

        // Smoke when moving AND not in drift mode
        bool shouldSmoke = isMoving && !isDriftMode;

        if (shouldSmoke && !isSmoking)
        {
            StartSmoke();
        }
        else if (!shouldSmoke && isSmoking)
        {
            StopSmoke();
        }
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
public class SmokeFollower : MonoBehaviour
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
