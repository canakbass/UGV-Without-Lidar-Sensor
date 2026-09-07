using UnityEngine;

public class RoverController : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeft;
    public WheelCollider frontRight;
    public WheelCollider rearLeft;
    public WheelCollider rearRight;

    [Header("Visual Meshes")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    [Header("Motor Settings")]
    public float maxMotorTorque = 500f; // Gerçekçi kütleler için artırıldı
    public float maxBrakeTorque = 1000f;
    public float turnTorqueMultiplier = 1.5f; // Dönüş motor gücü çarpanı

    [Header("Skid Steer Assist")]
    [Tooltip("4 tekerlekli tank sürüşünde dönüş direncini kırmak için şasiye ekstra dönüş kuvveti (Tork) uygular.")]
    public float turnAssistPower = 15f; 

    [Header("Inputs (-1 to 1)")]
    public float leftTargetTorque = 0f;
    public float rightTargetTorque = 0f;
    public bool applyBrake = false;

    [Header("Testing")]
    public bool useKeyboard = true;
    public bool rosBrake = false;

    private Rigidbody rb;
    private float currentTurnInput = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Ağırlık merkezini aşağı çekip devrilmeyi önler
        if (rb != null)
        {
            rb.centerOfMass = new Vector3(0, -0.3f, 0);
        }
    }

    void Update()
    {
        if (useKeyboard)
        {
            float vertical = Input.GetAxis("Vertical");   // W/S
            float horizontal = Input.GetAxis("Horizontal"); // A/D
            
            // Araba mantığında geri giderken A'ya basarsan arkanın sola gitmesini (önün sağa dönmesini) beklersin.
            float turn = horizontal;
            if (vertical < 0) 
            {
                turn = -horizontal;
            }
            
            currentTurnInput = turn;

            // Differential Drive (Tank Sürüşü) Karışımı
            leftTargetTorque = vertical + (turn * turnTorqueMultiplier);
            rightTargetTorque = vertical - (turn * turnTorqueMultiplier);

            leftTargetTorque = Mathf.Clamp(leftTargetTorque, -1f, 1f);
            rightTargetTorque = Mathf.Clamp(rightTargetTorque, -1f, 1f);
            
            applyBrake = Input.GetKey(KeyCode.Space) || rosBrake;
        }
        else 
        {
            applyBrake = rosBrake;
        }

        UpdateWheelMeshes();
    }

    void FixedUpdate()
    {
        float lTorque = applyBrake ? 0f : leftTargetTorque * maxMotorTorque;
        float rTorque = applyBrake ? 0f : rightTargetTorque * maxMotorTorque;
        float bTorque = applyBrake ? maxBrakeTorque : 0f;

        // Tork Uygulama
        frontLeft.motorTorque = lTorque;
        rearLeft.motorTorque = lTorque;
        frontRight.motorTorque = rTorque;
        rearRight.motorTorque = rTorque;

        // Fren Uygulama
        frontLeft.brakeTorque = bTorque;
        rearLeft.brakeTorque = bTorque;
        frontRight.brakeTorque = bTorque;
        rearRight.brakeTorque = bTorque;

        // 2. DÖNÜŞ YARDIMCISI (Skid Steer / Tank Drive Fix)
        // Tekerlekler direksiyonla dönmediği için yana doğru kaymaya direnç gösterir (Sideways friction).
        // Bu yüzden arabanın dönüşünü fiziksel olarak kendi ekseninde çevirerek desteklemeliyiz.
        if (Mathf.Abs(currentTurnInput) > 0.05f && !applyBrake)
        {
            // rb.mass ile çarparak aracın ağırlığı ne olursa olsun aynı hissi verir
            rb.AddRelativeTorque(Vector3.up * currentTurnInput * turnAssistPower * rb.mass);
        }
    }

    // ROS modunda da turn assist çalışsın diye dışarıdan çağrılabilir
    public void SetTurnInput(float turn)
    {
        currentTurnInput = Mathf.Clamp(turn, -1f, 1f);
    }

    void UpdateWheelMeshes()
    {
        UpdateWheelPose(frontLeft, frontLeftMesh);
        UpdateWheelPose(frontRight, frontRightMesh);
        UpdateWheelPose(rearLeft, rearLeftMesh);
        UpdateWheelPose(rearRight, rearRightMesh);
    }

    void UpdateWheelPose(WheelCollider col, Transform mesh)
    {
        if (mesh == null) return;
        Vector3 pos;
        Quaternion rot;
        col.GetWorldPose(out pos, out rot);
        
        mesh.position = pos;
        
        // Unity'nin varsayılan silindirleri dik (Y ekseni) durur.
        // WheelCollider ise X ekseni etrafında döner.
        // Bu yüzden silindiri yatırmak için Z ekseninde 90 derece lokal olarak döndürüyoruz.
        mesh.rotation = rot * Quaternion.Euler(0, 0, 90);
    }
}
