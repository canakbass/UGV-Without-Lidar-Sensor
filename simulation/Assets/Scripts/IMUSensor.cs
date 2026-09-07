using UnityEngine;

public class IMUSensor : MonoBehaviour
{
    [Header("Settings")]
    public Rigidbody robotRb;

    [Header("Outputs")]
    public float pitch; // X ekseni dönüşü (Eğim çıkma/inme)
    public float yaw;   // Y ekseni dönüşü (Sağa/Sola dönüş)
    public float roll;  // Z ekseni dönüşü (Yan eğim)

    public Vector3 angularVelocity;
    public Vector3 linearAcceleration;
    
    private Vector3 lastVelocity;

    void Start()
    {
        if (robotRb == null)
        {
            robotRb = GetComponentInParent<Rigidbody>();
        }
        
        if (robotRb != null)
        {
            lastVelocity = robotRb.velocity;
        }
    }

    void FixedUpdate()
    {
        if (robotRb == null) return;

        // Açıları 0-360'dan -180 ile 180 arasına çeviriyoruz (IMU gibi)
        Vector3 euler = transform.rotation.eulerAngles;
        pitch = NormalizeAngle(euler.x);
        yaw   = NormalizeAngle(euler.y);
        roll  = NormalizeAngle(euler.z);

        angularVelocity = robotRb.angularVelocity;
        
        // İvme hesaplama
        Vector3 currentVelocity = robotRb.velocity;
        linearAcceleration = (currentVelocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = currentVelocity;
    }

    float NormalizeAngle(float angle)
    {
        while (angle > 180f) angle -= 360f;
        while (angle < -180f) angle += 360f;
        return angle;
    }
}
