using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Proc = System.Diagnostics;
using System.IO;
using Newtonsoft.Json;

#region Serializable
[Serializable] public class DroneState {
    public Position position;
    public Orientation orientation;
    public Velocity velocity;
    public AngularVelocity angular_velocity;
    public Engine engine;
    public Meta meta;
}
[Serializable] public class Position { public float lat, lon, alt, unity_x, unity_y, unity_z; }
[Serializable] public class Orientation { public float roll, pitch, yaw; }
[Serializable] public class Velocity { public float u, v, w, airspeed; }
[Serializable] public class AngularVelocity { public float p, q, r; }
[Serializable] public class Engine { public float rpm, thrust; }
[Serializable] public class Meta { public string model; public bool rotorcraft; }
[Serializable] public class ControlInputs {
    public float throttle, aileron, elevator, rudder;
    public bool engine_start;
}
#endregion

public class JSBSimManager : MonoBehaviour
{
    [Header("Network")]
    public string host = "127.0.0.1";
    public int sendPort = 5555;
    public int receivePort = 5556;

    [Header("Python")]
    public string pythonPath = "";                 // leave blank to use PATH
    public string scriptPath = "jsbsim_bridge.py"; // place at project root (sibling of Assets)

    [Header("Controls")]
    [Range(0f,1f)] public float throttle = 0f;     // Drone: collective / Fixed wing: throttle
    [Range(-1f,1f)] public float roll = 0f;        // Drone: lateral cyclic / Fixed wing: aileron
    [Range(-1f,1f)] public float pitch = 0f;       // Drone: longitudinal cyclic / Fixed wing: elevator
    [Range(-1f,1f)] public float yaw = 0f;         // Drone: anti-torque / Fixed wing: rudder

    [Header("Debug")]
    public bool showDebugLogs = true;
    public bool showDetailedState = false;
    public int messagesReceived = 0;
    public int messagesSent = 0;

    private UdpClient sendClient;
    private UdpClient receiveClient;
    private Proc.Process pythonProcess;
    private DroneState currentState;
    private bool isConnected = false;
    private bool engineStartRequested = false;

    private float debugTimer = 0f;
    private float lastThrottle = 0f;

    private System.Text.StringBuilder logBuilder = new System.Text.StringBuilder();
    private string logFilePath;

    void Start()
    {
        Application.runInBackground = true;
        SetupLogFile();

        string projectRoot = Directory.GetParent(Application.dataPath).FullName;
        string fullScriptPath = Path.Combine(projectRoot, scriptPath);
        if (!File.Exists(fullScriptPath)) {
            Debug.LogError($"Python script not found at: {fullScriptPath}");
            return;
        }

        StartPythonBridge(fullScriptPath);
        InitializeNetwork();
    }

    void SetupLogFile()
    {
        string logsDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Logs");
        if (!Directory.Exists(logsDir)) Directory.CreateDirectory(logsDir);
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        logFilePath = Path.Combine(logsDir, $"JSBSimLog_{timestamp}.txt");
    }

    void StartPythonBridge(string fullPath)
    {
        try {
            string exe = (string.IsNullOrWhiteSpace(pythonPath) || !File.Exists(pythonPath)) ? "python" : pythonPath;
            pythonProcess = new Proc.Process();
            pythonProcess.StartInfo.FileName = exe;
            pythonProcess.StartInfo.Arguments = $"\"{fullPath}\"";
            pythonProcess.StartInfo.UseShellExecute = false;
            pythonProcess.StartInfo.RedirectStandardOutput = true;
            pythonProcess.StartInfo.RedirectStandardError = true;
            pythonProcess.StartInfo.CreateNoWindow = true;
            pythonProcess.StartInfo.WorkingDirectory = Directory.GetParent(Application.dataPath).FullName;
            pythonProcess.OutputDataReceived += (s, a) => { if (!string.IsNullOrEmpty(a.Data)) Debug.Log($"[Python] {a.Data}"); };
            pythonProcess.ErrorDataReceived  += (s, a) => { if (!string.IsNullOrEmpty(a.Data)) Debug.LogError($"[Python Error] {a.Data}"); };
            pythonProcess.Start();
            pythonProcess.BeginOutputReadLine();
            pythonProcess.BeginErrorReadLine();
            Debug.Log("Python bridge started");
        } catch (Exception e) {
            Debug.LogError($"Failed to start Python bridge: {e.Message}");
        }
    }

    void InitializeNetwork()
    {
        try {
            sendClient = new UdpClient(AddressFamily.InterNetwork);
            receiveClient = new UdpClient(receivePort, AddressFamily.InterNetwork);
            receiveClient.Client.ReceiveTimeout = 20;
            isConnected = true;
            Debug.Log($"Network: send {host}:{sendPort}, recv {receivePort}");
        } catch (Exception e) {
            Debug.LogError($"Network init failed: {e.Message}");
        }
    }

    void Update()
    {
        if (!isConnected) return;

        if (Input.GetKeyDown(KeyCode.J)) {
            engineStartRequested = true;   // one-shot in SendControls()
            Debug.Log("Engine start requested (J).");
        }

        SendControls();
        ReceiveState();

        if (showDebugLogs) {
            debugTimer += Time.deltaTime;
            if (debugTimer >= 3f) {
                Debug.Log($"NET: sent={messagesSent}, recv={messagesReceived}");
                debugTimer = 0f;
            }
        }
    }

    void SendControls()
    {
        try {
            var controls = new ControlInputs {
                throttle = throttle,
                aileron  = roll,
                elevator = pitch,
                rudder   = yaw,
                engine_start = engineStartRequested
            };

            string json = JsonConvert.SerializeObject(controls);
            byte[] data = System.Text.Encoding.UTF8.GetBytes(json);
            sendClient.Send(data, data.Length, host, sendPort);
            messagesSent++;
            engineStartRequested = false;

            if (Mathf.Abs(throttle - lastThrottle) > 0.01f) lastThrottle = throttle;
        }
        catch (Exception e) {
            Debug.LogError($"Send failed: {e.Message}");
        }
    }

    void ReceiveState()
    {
        try {
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, receivePort);
            byte[] data = receiveClient.Receive(ref ep);
            string json = System.Text.Encoding.UTF8.GetString(data);
            currentState = JsonConvert.DeserializeObject<DroneState>(json);
            messagesReceived++;

            if (showDetailedState && currentState != null && (messagesReceived % 50 == 0)) {
                var m = currentState.meta;
                Debug.Log($"Model={m?.model}, rotorcraft={m?.rotorcraft}");
            }
        }
        catch (SocketException) {
            // timeout; ignore
        }
        catch (Exception e) {
            Debug.LogError($"Receive failed: {e.Message}");
        }
    }

    public DroneState GetCurrentState() => currentState;

    public Vector3 GetUnityPosition()
    {
        var p = currentState?.position;
        return (p == null) ? Vector3.zero : new Vector3(p.unity_x, p.unity_y, p.unity_z);
    }

    public Quaternion GetUnityRotation()
    {
        var o = currentState?.orientation;
        if (o == null) return Quaternion.identity;
        float rollDeg = o.roll * Mathf.Rad2Deg;
        float pitchDeg = o.pitch * Mathf.Rad2Deg;
        float yawDeg = o.yaw * Mathf.Rad2Deg;
        return Quaternion.Euler(-pitchDeg, yawDeg, -rollDeg);
    }

    void OnApplicationQuit()
    {
        try { if (pythonProcess != null && !pythonProcess.HasExited) pythonProcess.Kill(); } catch { }
        try { sendClient?.Close(); } catch { }
        try { receiveClient?.Close(); } catch { }
    }
}
