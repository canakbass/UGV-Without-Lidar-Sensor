using UnityEngine;
using System.Collections;

public class TurretController : MonoBehaviour
{
    [Header("Hardware transforms")]
    public Transform turretBase; // Y ekseni dönme (Yaw/Pan)
    public Transform turretGun;  // X ekseni dönme (Pitch/Tilt)
    
    [Header("Cameras")]
    public Camera mainCamera;
    public Camera turretCamera;
    public CameraStreamer cameraStreamer;

    [Header("Laser Setup")]
    public Transform laserOrigin;
    public GameObject laserHitDotPrefab;
    private GameObject currentLaserDot;

    [Header("Settings")]
    public float panSpeed = 90f;
    public float pitchSpeed = 60f;
    public float minPitch = -45f;
    public float maxPitch = 45f;
    public float minPan = -90f;
    public float maxPan = 90f;

    [Header("State")]
    public bool isFiringMode = false;
    private float currentPan = 0f;
    private float currentPitch = 0f;
    private string lastRosState = "";
    private bool manualOverride = false; // F tuşuyla açıldıysa ROS state resetlemesin

    void Start()
    {
        if (laserHitDotPrefab != null)
        {
            currentLaserDot = Instantiate(laserHitDotPrefab);
            currentLaserDot.SetActive(false);
            currentLaserDot.GetComponent<Renderer>().material.color = Color.red;
            currentLaserDot.GetComponent<Renderer>().material.EnableKeyword("_EMISSION");
            currentLaserDot.GetComponent<Renderer>().material.SetColor("_EmissionColor", Color.red * 2f);
        }
        
        if (cameraStreamer == null) cameraStreamer = FindObjectOfType<CameraStreamer>();
        
        // Kameraları otomatik bul (Inspector'da atanmamışsa)
        if (mainCamera == null)
        {
            mainCamera = Camera.main;
            Debug.Log("<color=yellow>[Turret] mainCamera Inspector'da boş, Camera.main kullanılıyor: " + mainCamera + "</color>");
        }
        if (turretCamera == null)
        {
            // Sahnedeki tüm kameraları tara, mainCamera olmayanı bul
            foreach (Camera cam in FindObjectsOfType<Camera>(true))
            {
                if (cam != mainCamera && cam.gameObject.name.ToLower().Contains("turret"))
                {
                    turretCamera = cam;
                    Debug.Log("<color=yellow>[Turret] turretCamera otomatik bulundu: " + cam.gameObject.name + "</color>");
                    break;
                }
            }
        }
        
        Debug.Log($"<color=cyan>[Turret] Başlatıldı: main={mainCamera}, turret={turretCamera}, streamer={cameraStreamer}</color>");
        SetCamera(false);
    }

    void Update()
    {
        // ═══ F TUŞU: Hangi moddaysa diğerine geç ═══
        if (Input.GetKeyDown(KeyCode.F))
        {
            isFiringMode = !isFiringMode;
            manualOverride = isFiringMode;
            SetCamera(isFiringMode);
            
            // Araç hareketi kilitle/aç
            var rover = FindObjectOfType<RoverController>();
            if (rover != null) rover.enabled = !isFiringMode;
            
            Debug.Log($"<color=yellow>[F] Atış Modu: {(isFiringMode ? "AÇIK (Araç KİLİTLİ)" : "KAPALI (Araç SERBEST)")}</color>");
        }

        // ═══ Taret Modunda: WASD/Ok tuşları ile taret kontrolü ═══
        if (isFiringMode)
        {
            float h = 0f;
            float v = 0f;
            
            // WASD veya Ok tuşları
            if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) h = -1f;
            if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) h = 1f;
            if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) v = 1f;
            if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) v = -1f;

            if (Mathf.Abs(h) > 0.01f || Mathf.Abs(v) > 0.01f)
            {
                currentPan += h * panSpeed * Time.deltaTime;
                currentPitch -= v * pitchSpeed * Time.deltaTime;
                ApplyLimitsAndRotate();
            }

            // E ile ateş
            if (Input.GetKeyDown(KeyCode.E))
            {
                Debug.Log("<color=red>[MANUAL] 🔥 ATEŞ!</color>");
                FireLaser();
            }
        }
    }

    private void ApplyLimitsAndRotate()
    {
        currentPan = Mathf.Clamp(currentPan, minPan, maxPan);
        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);

        if (turretBase != null)
            turretBase.localEulerAngles = new Vector3(0, currentPan, 0);
        
        if (turretGun != null)
            turretGun.localEulerAngles = new Vector3(currentPitch, 0, 0);
    }

    public void FireLaser()
    {
        if (laserOrigin == null || currentLaserDot == null) return;

        RaycastHit hit;
        int layerMask = ~LayerMask.GetMask("Ignore Raycast");
        
        if (Physics.Raycast(laserOrigin.position, laserOrigin.forward, out hit, 100f, layerMask))
        {
            Debug.Log($"<color=red>🔥 FIRE! Hit: {hit.collider.gameObject.name} @ {hit.point}</color>");
            currentLaserDot.transform.position = hit.point;
            currentLaserDot.transform.rotation = Quaternion.LookRotation(hit.normal); 
            currentLaserDot.SetActive(true);
            StopAllCoroutines();
            StartCoroutine(HideLaserDot());
        }
        else
        {
            Debug.Log("<color=yellow>🔥 FIRE! Karavana (boşa gitti)</color>");
        }
    }

    private IEnumerator HideLaserDot()
    {
        yield return new WaitForSeconds(0.5f);
        currentLaserDot.SetActive(false);
    }

    public void SetCamera(bool useTurret)
    {
        if (mainCamera == null || turretCamera == null || cameraStreamer == null)
        {
            Debug.LogWarning($"<color=red>[Turret] SetCamera BASARISIZ! main={mainCamera}, turret={turretCamera}, streamer={cameraStreamer}</color>");
            return;
        }

        // Sadece CameraStreamer'ın baktığı kamerayı değiştir. GameObject kapatma!
        cameraStreamer.streamCamera = useTurret ? turretCamera : mainCamera;
        isFiringMode = useTurret;
        Debug.Log($"<color=cyan>[Turret] Kamera → {(useTurret ? "TARET" : "ANA")}</color>");
    }

    /// <summary>
    /// SADECE state DEĞIŞTİĞINDE tetiklenir (her mesajda değil!).
    /// Manuel override varsa dokunmaz.
    /// </summary>
    public void OnStateChanged(string newState)
    {
        // Aynı state tekrar geliyorsa (periyodik yayın) - YAPMA BİR ŞEY
        if (newState == lastRosState) return;
        
        string oldState = lastRosState;
        lastRosState = newState;
        
        // Manuel override aktifse ROS state değişikliği kamerayı resetlemesin
        if (manualOverride)
        {
            Debug.Log($"<color=yellow>[Turret] State {oldState}→{newState} ama Manuel Override aktif, kamera değişmiyor</color>");
            return;
        }
        
        if ((newState == "IDLE" || newState == "MANUAL") && isFiringMode)
        {
            SetCamera(false);
            Debug.Log("<color=yellow>[Turret] " + oldState + "→" + newState + " : Ana kameraya dönüldü.</color>");
        }
    }

    /// <summary>
    /// ROS2 Otonom Komut: [yaw, pitch, fire, camMode]
    /// </summary>
    public void ProcessRosCommand(float yawOffset, float pitchOffset, float fireCmd, float switchCamCmd)
    {
        // Otonom komut geldiğinde manuel override'ı iptal et
        manualOverride = false;
        
        if (switchCamCmd > 0.5f && !isFiringMode)
        {
            // Otonom geçiş: taret açılarını sıfırla + kamerayı aç
            currentPan = 0f;
            currentPitch = 0f;
            ApplyLimitsAndRotate();
            SetCamera(true);
            Debug.Log("<color=cyan>[Turret] Otonom: Açılar sıfırlandı + Taret kamera AKTİF</color>");
        }
        else if (switchCamCmd <= 0.5f && isFiringMode)
            SetCamera(false);

        currentPan += yawOffset;
        currentPitch += pitchOffset;
        ApplyLimitsAndRotate();

        if (fireCmd > 0.5f)
        {
            FireLaser();
        }
    }
}
