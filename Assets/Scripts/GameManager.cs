using UnityEngine;
using UnityEngine.SceneManagement;

public class GameManager : MonoBehaviour
{
    public Transform player;
    public float fallThreshold = -10f;

    void Update()
    {
        if (player != null && player.position.y < fallThreshold)
            SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
}