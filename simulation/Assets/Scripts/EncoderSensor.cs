using UnityEngine;

public class EncoderSensor : MonoBehaviour
{
    [Header("Settings")]
    public WheelCollider targetWheel;
    
    [Header("Outputs")]
    public float totalDistance = 0f;
    public float currentSpeedVelocity = 0f; // m/s cinsinden hız

    void FixedUpdate()
    {
        if (targetWheel == null) return;
        
        // rpm = rev/min. rps = rev/sec.
        float rps = targetWheel.rpm / 60f;
        
        // Tekerlek çevresi = 2 * PI * r
        float wheelCircumference = 2f * Mathf.PI * targetWheel.radius;
        
        // Bu frame'de (saniyede 50 kere) gidilen mesafe
        float distanceThisFrame = rps * wheelCircumference * Time.fixedDeltaTime;
        
        totalDistance += Mathf.Abs(distanceThisFrame);
        
        // Anlık hız (Yönlü, geri giderse eksi)
        currentSpeedVelocity = distanceThisFrame / Time.fixedDeltaTime;
    }
}
