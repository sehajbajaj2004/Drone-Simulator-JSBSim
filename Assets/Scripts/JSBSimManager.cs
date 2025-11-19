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
    public string pythonPath = "";                 // leave blank to use PATH / venv
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
    private float logFlushTimer = 0f;

    // ---- file logging restored ----
    private StringBuilder logBuilder = new StringBuilder();
    private string logFilePath;

    void Start()
    {
        Application.runInBackground = true;

        SetupLogFile();
        Log("=== JSBSim Manager Session Started ===");
        Log($"Timestamp: {DateTime.Now}");

        string projectRoot = Directory.GetParent(Application.dataPath).FullName;
        string fullScriptPath = Path.Combine(projectRoot, scriptPath);
        if (!File.Exists(fullScriptPath)) {
            Log($"[ERROR] Python script not found at: {fullScriptPath}");
            Debug.LogError($"Python script not found at: {fullScriptPath}");
            return;
        }

        Log($"Found Python script at: {fullScriptPath}");
        StartPythonBridge(fullScriptPath);
        InitializeNetwork();
    }

    // -------- Logging (restored) --------
    private void SetupLogFile()
    {
        string logsDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Logs");
        if (!Directory.Exists(logsDir)) Directory.CreateDirectory(logsDir);
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        logFilePath = Path.Combine(logsDir, $"JSBSimLog_{timestamp}.txt");
    }

    private void Log(string message)
    {
        string line = $"[{DateTime.Now:HH:mm:ss}] {message}";
        logBuilder.AppendLine(line);
        if (showDebugLogs) Debug.Log(message);
    }

    private void WriteLogsToFile()
    {
        try {
            File.WriteAllText(logFilePath, logBuilder.ToString());
            Debug.Log($"Logs written to: {logFilePath}");
        } catch (Exception e) {
            Debug.LogError($"Failed to write log file: {e.Message}");
        }
    }

    // -------- Python Bridge --------
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

            pythonProcess.OutputDataReceived += (s, a) =>
            {
                if (!string.IsNullOrEmpty(a.Data))
                {
                    Log($"[Python] {a.Data}");
                }
            };
            pythonProcess.ErrorDataReceived += (s, a) =>
            {
                if (!string.IsNullOrEmpty(a.Data))
                {
                    Log($"[Python Error] {a.Data}");
                }
            };

            pythonProcess.Start();
            pythonProcess.BeginOutputReadLine();
            pythonProcess.BeginErrorReadLine();

            Log("Python bridge started");
        } catch (Exception e) {
            Log($"[ERROR] Failed to start Python bridge: {e.Message}");
        }
    }

    // -------- Networking --------
    void InitializeNetwork()
    {
        try {
            sendClient = new UdpClient(AddressFamily.InterNetwork);
            receiveClient = new UdpClient(receivePort, AddressFamily.InterNetwork);
            receiveClient.Client.ReceiveTimeout = 20;
            isConnected = true;

            Log($"Network initialized - Sending to {host}:{sendPort}, Receiving on {receivePort}");
        } catch (Exception e) {
            Log($"[ERROR] Network init failed: {e.Message}");
        }
    }

    void Update()
    {
        if (!isConnected) return;

        // J → start sequence (one-shot)
        if (Input.GetKeyDown(KeyCode.J)) {
            engineStartRequested = true;
            Log("Engine start requested (J).");
        }

        SendControls();
        ReceiveState();

        // Console & file status every 3s
        debugTimer += Time.deltaTime;
        logFlushTimer += Time.deltaTime;

        if (debugTimer >= 3f) {
            Log($"NET: sent={messagesSent}, recv={messagesReceived}");
            if (showDetailedState && currentState?.meta != null) {
                Log($"Model={currentState.meta.model}, rotorcraft={currentState.meta.rotorcraft}");
            }
            debugTimer = 0f;
        }

        // Periodic flush to disk (optional; keeps log alive if Unity crashes)
        if (logFlushTimer >= 10f) {
            WriteLogsToFile();
            logFlushTimer = 0f;
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
            byte[] data = Encoding.UTF8.GetBytes(json);
            sendClient.Send(data, data.Length, host, sendPort);
            messagesSent++;

            // clear the one-shot flag only after a successful send
            engineStartRequested = false;

            // lightweight change log
            if (Mathf.Abs(throttle - lastThrottle) > 0.05f) {
                Log($"Throttle -> {throttle:F2}");
                lastThrottle = throttle;
            }
        }
        catch (Exception e) {
            Log($"[ERROR] Send failed: {e.Message}");
        }
    }

    void ReceiveState()
    {
        try {
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, receivePort);
            byte[] data = receiveClient.Receive(ref ep); // throws on timeout
            string json = Encoding.UTF8.GetString(data);
            currentState = JsonConvert.DeserializeObject<DroneState>(json);
            messagesReceived++;

            if (messagesReceived == 1)
                Log("✓ First state received from JSBSim.");
        }
        catch (SocketException) {
            // timeout; ignore
        }
        catch (Exception e) {
            Log($"[ERROR] Receive failed: {e.Message}");
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
        Log("=== Application Quit ===");
        try {
            if (pythonProcess != null && !pythonProcess.HasExited) {
                pythonProcess.Kill();
                Log("Python process terminated");
            }
        } catch { }

        try { sendClient?.Close(); } catch { }
        try { receiveClient?.Close(); } catch { }

        Log($"Final counts - Sent: {messagesSent}, Received: {messagesReceived}");
        WriteLogsToFile();  // <-- save to file on exit
    }
}
