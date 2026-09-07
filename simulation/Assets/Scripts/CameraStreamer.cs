using UnityEngine;
using System;
using System.Net;
using System.Threading;
using System.Text;

public class CameraStreamer : MonoBehaviour
{
    [Header("Settings")]
    public Camera streamCamera;
    public int imageWidth = 640;
    public int imageHeight = 360;
    public int jpegQuality = 50;
    public int streamPort = 8080;
    public float frameRate = 30f;

    [Header("ROS Vision Feed")]
    public RosBridgeClient rosBridge;
    public float visionFps = 10f;
    public int visionJpegQuality = 40;

    private RenderTexture renderTexture;
    private Texture2D screenShot;
    private HttpListener httpListener;
    private Thread listenerThread;
    private bool isRunning = false;
    private byte[] latestJpeg;
    private readonly object lockObj = new object();

    // Vision: sadece mevcut JPEG'i base64'e çevir (ekstra render YOK)
    private float visionTimer = 0f;

    void Start()
    {
        if (rosBridge == null)
            rosBridge = FindObjectOfType<RosBridgeClient>();

        if (streamCamera == null)
        {
            streamCamera = GetComponent<Camera>();
            if (streamCamera == null)
                streamCamera = Camera.main;
        }

        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        screenShot = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        isRunning = true;
        listenerThread = new Thread(HttpListenerLoop);
        listenerThread.IsBackground = true;
        listenerThread.Start();

        InvokeRepeating("CaptureFrame", 1f, 1f / frameRate);

        Debug.Log($"<color=cyan>Camera Streamer: http://localhost:{streamPort}/stream @ {frameRate} FPS</color>");
        if (rosBridge != null)
            Debug.Log($"<color=cyan>Vision feed: /camera/image_base64 @ {visionFps} FPS</color>");
    }

    void CaptureFrame()
    {
        if (streamCamera == null) return;

        RenderTexture prevRT = streamCamera.targetTexture;
        streamCamera.targetTexture = renderTexture;
        streamCamera.Render();

        RenderTexture prevActive = RenderTexture.active;
        RenderTexture.active = renderTexture;
        screenShot.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        screenShot.Apply();
        RenderTexture.active = prevActive;
        streamCamera.targetTexture = prevRT;

        byte[] jpeg = screenShot.EncodeToJPG(jpegQuality);
        lock (lockObj)
        {
            latestJpeg = jpeg;
        }
    }

    void Update()
    {
        // Vision feed: mevcut JPEG'i ROS'a gönder (ekstra render yapmadan)
        if (rosBridge == null) return;
        
        visionTimer += Time.deltaTime;
        if (visionTimer < 1f / visionFps) return;
        visionTimer = 0f;

        byte[] jpeg;
        lock (lockObj)
        {
            jpeg = latestJpeg;
        }
        
        if (jpeg == null || jpeg.Length == 0) return;

        string base64 = Convert.ToBase64String(jpeg);
        string json = "{\"op\":\"publish\",\"topic\":\"/camera/image_base64\"," +
                      "\"type\":\"std_msgs/msg/String\"," +
                      "\"msg\":{\"data\":\"" + base64 + "\"}}";
        rosBridge.SendRawMessage(json);
    }

    void HttpListenerLoop()
    {
        httpListener = new HttpListener();
        httpListener.Prefixes.Add($"http://*:{streamPort}/");
        
        try { httpListener.Start(); }
        catch (Exception e)
        {
            Debug.LogError("HTTP Listener failed: " + e.Message);
            return;
        }

        while (isRunning)
        {
            try
            {
                HttpListenerContext context = httpListener.GetContext();
                string path = context.Request.Url.AbsolutePath;

                if (path == "/stream")
                    ServeStream(context);
                else if (path == "/snapshot")
                    ServeSnapshot(context);
                else
                {
                    byte[] data = Encoding.UTF8.GetBytes("<html><body><h2>Unity Camera</h2><img src='/stream' style='width:100%'/></body></html>");
                    context.Response.ContentType = "text/html";
                    context.Response.ContentLength64 = data.Length;
                    context.Response.OutputStream.Write(data, 0, data.Length);
                    context.Response.Close();
                }
            }
            catch (Exception)
            {
                if (!isRunning) break;
            }
        }
    }

    void ServeStream(HttpListenerContext context)
    {
        context.Response.ContentType = "multipart/x-mixed-replace; boundary=frame";
        context.Response.Headers.Add("Cache-Control", "no-cache");
        context.Response.Headers.Add("Access-Control-Allow-Origin", "*");

        try
        {
            while (isRunning)
            {
                byte[] jpeg;
                lock (lockObj) { jpeg = latestJpeg; }

                if (jpeg != null && jpeg.Length > 0)
                {
                    string header = $"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {jpeg.Length}\r\n\r\n";
                    byte[] headerBytes = Encoding.UTF8.GetBytes(header);
                    context.Response.OutputStream.Write(headerBytes, 0, headerBytes.Length);
                    context.Response.OutputStream.Write(jpeg, 0, jpeg.Length);
                    context.Response.OutputStream.Write(Encoding.UTF8.GetBytes("\r\n"), 0, 2);
                    context.Response.OutputStream.Flush();
                }
                Thread.Sleep((int)(1000f / frameRate));
            }
        }
        catch (Exception) { }
    }

    void ServeSnapshot(HttpListenerContext context)
    {
        byte[] jpeg;
        lock (lockObj) { jpeg = latestJpeg; }
        if (jpeg != null)
        {
            context.Response.ContentType = "image/jpeg";
            context.Response.Headers.Add("Access-Control-Allow-Origin", "*");
            context.Response.ContentLength64 = jpeg.Length;
            context.Response.OutputStream.Write(jpeg, 0, jpeg.Length);
        }
        context.Response.Close();
    }

    void OnDestroy()
    {
        isRunning = false;
        CancelInvoke();
        if (httpListener != null && httpListener.IsListening)
        {
            httpListener.Stop();
            httpListener.Close();
        }
        if (renderTexture != null)
            renderTexture.Release();
    }
}
