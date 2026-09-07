using UnityEngine;
using System.Net.WebSockets;
using System.Threading;
using System.Threading.Tasks;
using System.Text;
using System.Collections.Concurrent;
using System;

// ========== SIMPLE ROS MESSAGE CLASSES ==========
// We avoid complex nav_msgs/Odometry to prevent ROS2 Jazzy compatibility issues.
// Instead we publish all telemetry as Float32MultiArray (simple and bulletproof).

[System.Serializable]
public class RosBaseMsg
{
    public string op;
    public string id;
    public string topic;
    public string type;
}

[System.Serializable]
public class RosSubscribeMsg : RosBaseMsg { }

[System.Serializable]
public class RosPublishFloat32Array : RosBaseMsg
{
    public FloatArrayMsg msg;
}

[System.Serializable]
public class RosPublishString : RosBaseMsg
{
    public StringMsg msg;
}

[System.Serializable]
public class FloatArrayMsg
{
    public float[] data;
}

[System.Serializable]
public class StringMsg
{
    public string data;
}

public enum TelemetryIdx {
    PosX = 0, PosY = 1, PosZ = 2,
    Speed = 3, Yaw = 4, Pitch = 5, Roll = 6,
    EncoderDist = 7,
    LeftPower = 8, RightPower = 9
}

// ========== MAIN BRIDGE ==========
public class RosBridgeClient : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosbridgeUrl = "ws://localhost:9090";
    public RoverController roverController;
    public EncoderSensor encoderLeft;
    public EncoderSensor encoderRight;
    public IMUSensor imu;
    
    [Header("7 Ultrasonic Sensors")]
    [Tooltip("Ön Merkez")]
    public UltrasonicSensor usFrontCenter;
    [Tooltip("Ön Sola Bakan")]
    public UltrasonicSensor usFrontLeft;
    [Tooltip("Ön Sağa Bakan")]
    public UltrasonicSensor usFrontRight;
    
    [Tooltip("Ön Sol Köşe (45 Derece)")]
    public UltrasonicSensor usCornerFL;
    [Tooltip("Ön Sağ Köşe (45 Derece)")]
    public UltrasonicSensor usCornerFR;
    [Tooltip("Arka Sol Köşe (135 Derece)")]
    public UltrasonicSensor usCornerRL;
    [Tooltip("Arka Sağ Köşe (135 Derece)")]
    public UltrasonicSensor usCornerRR;

    private ClientWebSocket ws;
    private CancellationTokenSource cts;
    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    private bool isConnected = false;

    private float publishTimer = 0f;
    public float publishRate = 0.1f; // 10Hz
    
    // ROS kontrol zaman aşımı: Son ROS komutu bu kadar saniye önce geldiyse klavyeye geri dön
    private float lastRosCommandTime = -10f;
    private float rosTimeoutSeconds = 1.0f;

    async void Start()
    {
        ws = new ClientWebSocket();
        cts = new CancellationTokenSource();

        try
        {
            await ws.ConnectAsync(new Uri(rosbridgeUrl), cts.Token);
            Debug.Log("<color=green>Connected to Rosbridge Server!</color>");

            _ = ReceiveLoop();

            // Subscribe to /cmd_vel
            RosSubscribeMsg sub = new RosSubscribeMsg
            {
                op = "subscribe",
                id = "sub_cmd_vel",
                topic = "/cmd_vel",
                type = "geometry_msgs/msg/Twist"
            };
            await SendMessage(JsonUtility.ToJson(sub));
            
            // Subscribe to /cmd_turret
            RosSubscribeMsg subTurret = new RosSubscribeMsg
            {
                op = "subscribe",
                id = "sub_cmd_turret",
                topic = "/cmd_turret",
                type = "std_msgs/msg/Float32MultiArray"
            };
            await SendMessage(JsonUtility.ToJson(subTurret));
            
            // Subscribe to /robot_state
            RosSubscribeMsg sub2 = new RosSubscribeMsg
            {
                op = "subscribe",
                id = "sub_robot_state",
                topic = "/robot_state",
                type = "std_msgs/msg/String"
            };
            await SendMessage(JsonUtility.ToJson(sub2));

            // ÖNCE advertise → SONRA isConnected (publish izni)
            await SendMessage("{\"op\":\"advertise\",\"topic\":\"/unity/telemetry\",\"type\":\"std_msgs/msg/Float32MultiArray\"}");
            await SendMessage("{\"op\":\"advertise\",\"topic\":\"/sensors/ultrasonic\",\"type\":\"std_msgs/msg/Float32MultiArray\"}");
            await SendMessage("{\"op\":\"advertise\",\"topic\":\"/camera/image_base64\",\"type\":\"std_msgs/msg/String\"}");
            
            // Advertise tamamlandıktan SONRA publish'e izin ver
            isConnected = true;
            Debug.Log("<color=green>Topics advertised + ready to publish!</color>");
        }
        catch (Exception e)
        {
            Debug.LogError("ROS Connection Failed: " + e.Message);
        }
    }

    void Update()
    {
        while (messageQueue.TryDequeue(out string json))
        {
            ProcessMessage(json);
        }

        // ROS komutu bir süredir gelmiyorsa klavyeye geri dön
        if (roverController != null && Time.time - lastRosCommandTime > rosTimeoutSeconds)
        {
            roverController.useKeyboard = true;
        }

        publishTimer += Time.deltaTime;
        if (publishTimer >= publishRate && isConnected)
        {
            publishTimer = 0;
            PublishTelemetry();
            PublishUltrasonic();
        }
    }

    private void ProcessMessage(string json)
    {
        if (json.Contains("/cmd_vel"))
        {
            try
            {
                float linearX = ExtractFloat(json, "\"linear\":", "\"x\":");
                float linearY = ExtractFloat(json, "\"linear\":", "\"y\":");
                float angularZ = ExtractFloat(json, "\"angular\":", "\"z\":");

                if (roverController != null)
                {
                    lastRosCommandTime = Time.time;
                    roverController.useKeyboard = false;
                    
                    // Differential Drive dönüşümü: linear.x = ileri/geri, angular.z = dönüş
                    float leftPower = linearX + angularZ;
                    float rightPower = linearX - angularZ;
                    
                    roverController.leftTargetTorque = Mathf.Clamp(leftPower, -1f, 1f);
                    roverController.rightTargetTorque = Mathf.Clamp(rightPower, -1f, 1f);
                    
                    // Frenleme sinyali linear.y üzerinden geliyor (1.0 = FREN)
                    roverController.rosBrake = (linearY > 0.5f);
                    
                    // BUG 3 FIX: Turn assist'i de ROS modunda çalıştır
                    roverController.SetTurnInput(angularZ);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("cmd_vel parse error: " + e.Message);
            }
        }
        else if (json.Contains("cmd_turret"))
        {
            Debug.Log("<color=magenta>[ROS] cmd_turret mesajı alındı: " + json.Substring(0, Mathf.Min(json.Length, 200)) + "</color>");
            try
            {
                // Float32MultiArray arıyoruz: [yaw_offset, pitch_offset, fire_trigger(1=Ateş), camera_mode(1=TaretCmr)]
                int dataTokenIndex = json.IndexOf("\"data\":");
                if (dataTokenIndex != -1)
                {
                    int startIndex = json.IndexOf("[", dataTokenIndex) + 1;
                    int endIndex = json.IndexOf("]", startIndex);
                    string arrayStr = json.Substring(startIndex, endIndex - startIndex);
                    string[] parts = arrayStr.Split(',');
                    if (parts.Length >= 4)
                    {
                        float yaw = float.Parse(parts[0], System.Globalization.CultureInfo.InvariantCulture);
                        float pitch = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
                        float fire = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);
                        float cam = float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);
                        
                        TurretController tc = FindObjectOfType<TurretController>();
                        if (tc != null) 
                        {
                            tc.ProcessRosCommand(yaw, pitch, fire, cam);
                        }
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("cmd_turret parse error: " + e.Message);
            }
        }
        else if (json.Contains("/robot_state"))
        {
            try
            {
                // robot_state mesajından state string'ini çıkar
                int dataIdx = json.IndexOf("\"data\":\"");
                if (dataIdx != -1)
                {
                    dataIdx += 8;
                    int endIdx = json.IndexOf("\"", dataIdx);
                    string stateStr = json.Substring(dataIdx, endIdx - dataIdx);
                    
                    // Taret kamerasını Manuel/IDLE modunda normale döndür
                    TurretController tc = FindObjectOfType<TurretController>();
                    if (tc != null)
                    {
                        tc.OnStateChanged(stateStr);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("robot_state parse error: " + e.Message);
            }
        }
    }

    private float ExtractFloat(string source, string blockKey, string valKey)
    {
        int blockIdx = source.IndexOf(blockKey);
        if (blockIdx == -1) return 0f;
        int valIdx = source.IndexOf(valKey, blockIdx);
        if (valIdx == -1) return 0f;
        
        int startStr = valIdx + valKey.Length;
        int endStr = source.IndexOfAny(new char[] { ',', '}' }, startStr);
        string numStr = source.Substring(startStr, endStr - startStr).Trim();
        
        if (float.TryParse(numStr, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float val))
            return val;
        return 0f;
    }

    // BUG 1 & 2 FIX: nav_msgs/Odometry yerine basit Float32MultiArray kullanarak
    // header.seq ve covariance sorunlarını tamamen ortadan kaldırıyoruz
    private async void PublishTelemetry()
    {
        if (roverController == null) return;
        
        RosPublishFloat32Array pub = new RosPublishFloat32Array
        {
            op = "publish",
            topic = "/unity/telemetry",
            type = "std_msgs/msg/Float32MultiArray"
        };

        // [0]:posX, [1]:posY, [2]:posZ, [3]:speed, [4]:yaw, [5]:pitch, [6]:roll, 
        // [7]:encoderDist, [8]:leftPower, [9]:rightPower
        Vector3 pos = roverController.transform.position;
        float speed = 0f;
        float encoderDist = 0f;
        
        if (encoderLeft != null)
        {
            speed = encoderLeft.currentSpeedVelocity;
            encoderDist = encoderLeft.totalDistance;
        }
        
        float yaw = imu != null ? imu.yaw : 0f;
        float pitch = imu != null ? imu.pitch : 0f;
        float roll = imu != null ? imu.roll : 0f;

        float[] tData = new float[10];
        tData[(int)TelemetryIdx.PosX] = pos.x;
        tData[(int)TelemetryIdx.PosY] = pos.y;
        tData[(int)TelemetryIdx.PosZ] = pos.z;
        tData[(int)TelemetryIdx.Speed] = speed;
        tData[(int)TelemetryIdx.Yaw] = yaw;
        tData[(int)TelemetryIdx.Pitch] = pitch;
        tData[(int)TelemetryIdx.Roll] = roll;
        tData[(int)TelemetryIdx.EncoderDist] = encoderDist;
        tData[(int)TelemetryIdx.LeftPower] = roverController.leftTargetTorque;
        tData[(int)TelemetryIdx.RightPower] = roverController.rightTargetTorque;

        pub.msg = new FloatArrayMsg { data = tData };
        
        await SendMessage(JsonUtility.ToJson(pub));
    }

    private async void PublishUltrasonic()
    {
        // [FC, FL, FR, CFL, CFR, CRL, CRR]
        float fc  = usFrontCenter != null ? usFrontCenter.currentDistance : 4.0f;
        float fl  = usFrontLeft   != null ? usFrontLeft.currentDistance   : 4.0f;
        float fr  = usFrontRight  != null ? usFrontRight.currentDistance  : 4.0f;
        float cfl = usCornerFL    != null ? usCornerFL.currentDistance    : 4.0f;
        float cfr = usCornerFR    != null ? usCornerFR.currentDistance    : 4.0f;
        float crl = usCornerRL    != null ? usCornerRL.currentDistance    : 4.0f;
        float crr = usCornerRR    != null ? usCornerRR.currentDistance    : 4.0f;

        // Raw JSON kullan (JsonUtility id="" alanı sorun yaratıyor olabilir)
        string json = string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "{{\"op\":\"publish\",\"topic\":\"/sensors/ultrasonic\",\"msg\":{{\"data\":[{0},{1},{2},{3},{4},{5},{6}]}}}}",
            fc, fl, fr, cfl, cfr, crl, crr);
        await SendMessage(json);
    }

    // SignDetector ve diğer bileşenler tarafından doğrudan mesaj göndermek için
    public async void SendRawMessage(string json)
    {
        await SendMessage(json);
    }

    private async Task SendMessage(string message)
    {
        if (ws != null && ws.State == WebSocketState.Open)
        {
            byte[] bytes = Encoding.UTF8.GetBytes(message);
            await ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, cts.Token);
        }
    }

    private async Task ReceiveLoop()
    {
        byte[] buffer = new byte[8192];
        while (ws != null && ws.State == WebSocketState.Open)
        {
            try
            {
                WebSocketReceiveResult result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);
                if (result.MessageType == WebSocketMessageType.Text)
                {
                    string message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    messageQueue.Enqueue(message);
                }
                else if (result.MessageType == WebSocketMessageType.Close)
                {
                    await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, cts.Token);
                }
            }
            catch (Exception)
            {
                break;
            }
        }
        isConnected = false;
        Debug.LogWarning("ROS Websocket disconnected.");
    }

    void OnDestroy()
    {
        if (ws != null)
        {
            cts.Cancel();
            ws.Dispose();
        }
    }
}
