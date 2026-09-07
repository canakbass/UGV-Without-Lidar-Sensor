using UnityEngine;

public class UltrasonicSensor : MonoBehaviour
{
    [Header("Settings")]
    public float maxDistance = 4.0f; // 4 metre menzil
    public string sensorName = "US_Front";

    [Header("Output")]
    public float currentDistance;

    void FixedUpdate()
    {
        RaycastHit hit;
        // Sensörün baktığı yöne (mavi ok - forward) ışın gönder
        if (Physics.Raycast(transform.position, transform.forward, out hit, maxDistance))
        {
            currentDistance = hit.distance;
        }
        else
        {
            currentDistance = maxDistance; // Çarpan bir şey yoksa maksimum değer
        }
    }

    void OnDrawGizmos()
    {
        // Editörde sensörün baktığı yeri ve ölçümünü görmek için çizgi çiz
        Gizmos.color = (currentDistance < maxDistance) ? Color.red : Color.green;
        Vector3 direction = transform.forward * currentDistance;
        Gizmos.DrawRay(transform.position, direction);
        Gizmos.DrawWireSphere(transform.position + direction, 0.05f);
    }
}
