using UnityEngine;

public class SmoothLoop : MonoBehaviour
{
    public AudioSource source;

    private void Update()
    {
        if (source.time > source.clip.length * 0.95f)
        {
            source.time = 0.1f;
        }
    }
}
