using UnityEngine;
using UnityEngine.SceneManagement;

public class PlayerCollision : MonoBehaviour
{
    [Header("Settings")]
    public string obstacleTag = "Obstacle";
    public string sceneName = "Dessert";
    public float restartDelay = 0f;

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag(obstacleTag))
        {
            Die();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(obstacleTag))
        {
            Die();
        }
    }

    void Die()
    {
        if (restartDelay > 0f)
        {
            Invoke(nameof(RestartScene), restartDelay);
        }
        else
        {
            RestartScene();
        }
    }

    void RestartScene()
    {
        SceneManager.LoadScene(sceneName);
    }
}
