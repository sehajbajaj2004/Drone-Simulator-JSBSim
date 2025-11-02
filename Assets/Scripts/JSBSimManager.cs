using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Diagnostics;
using System.IO;
using Newtonsoft.Json;
using System.Collections.Generic;
using System.Text;

#region Serializable Classes
[Serializable]
public class DroneState
{
    public Position position;
    public Orientation orientation;
    public Velocity velocity;
    public AngularVelocity angular_velocity;
    public Engine engine;
}

[Serializable]
public class Position
{
    public float lat;
    public float lon;
    public float alt;
    public float unity_x;
    public float unity_y;
    public float unity_z;
}

[Serializable]
public class Orientation
{
    public float roll;
    public float pitch;
    public float yaw;
}

[Serializable]
public class Velocity
{
    public float u;
    public float v;
    public float w;
    public float airspeed;
}

[Serializable]
public class AngularVelocity
{
    public float p;
    public float q;
    public float r;
}

[Serializable]
public class Engine
{
    public float rpm;
    public float thrust;
}

[Serializable]
public class ControlInputs
{
    public float throttle;
    public float aileron;   // Roll control
    public float elevator;  // Pitch control
    public float rudder;    // Yaw control
}
#endregion

public class JSBSimManager : MonoBehaviour
{
    [Header("Network Settings")]
    public int sendPort = 5555;
    public int receivePort = 5556;

    [Header("Python Settings")]
    private string pythonPath = @"C:\Users\Sehaj\AppData\Local\Programs\Python\Python310\python.exe";
    public string scriptPath = "jsbsim_bridge.py";

    [Header("Flight Controls")]
    [Range(0f, 1f)]
    public float throttle = 0f;
    
    [Range(-1f, 1f)]
    public float roll = 0f;      // Aileron control
    
    [Range(-1f, 1f)]
    public float pitch = 0f;     // Elevator control
    
    [Range(-1f, 1f)]
    public float yaw = 0f;       // Rudder control

    [Header("Control Sensitivity")]
    public float throttleRate = 1.0f;
    public float controlRate = 2.0f;

    [Header("Debug")]
    public bool showDebugLogs = true;
    public bool showDetailedState = false;
    public int messagesReceived = 0;
    public int messagesSent = 0;

    private UdpClient sendClient;
    private UdpClient receiveClient;
    private Process pythonProcess;
    private DroneState currentState;
    private bool isConnected = false;

    private float debugTimer = 0f;
    private float lastThrottle = 0f;

    // Log Collection
    private StringBuilder logBuilder = new StringBuilder();
    private string logFilePath;

    void Start()
    {
        SetupLogFile();

        string fullScriptPath = Path.Combine(Directory.GetParent(Application.dataPath).FullName, scriptPath);
        if (!File.Exists(fullScriptPath))
        {
            Log($"[ERROR] Python script not found at: {fullScriptPath}");
            UnityEngine.Debug.LogError($"Python script not found at: {fullScriptPath}");
            UnityEngine.Debug.LogError("Please make sure jsbsim_bridge.py is in the project root folder!");
            return;
        }

        Log($"Found Python script at: {fullScriptPath}");
        UnityEngine.Debug.Log($"Found Python script at: {fullScriptPath}");

        StartPythonBridge(fullScriptPath);
        InitializeNetwork();
    }

    #region Logging Helpers
    private void SetupLogFile()
    {
        string logsDir = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Logs");
        if (!Directory.Exists(logsDir)) Directory.CreateDirectory(logsDir);

        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        logFilePath = Path.Combine(logsDir, $"JSBSimLog_{timestamp}.txt");

        Log("=== JSBSim Manager Session Started ===");
        Log($"Timestamp: {DateTime.Now}");
    }

    private void Log(string message)
    {
        logBuilder.AppendLine($"[{DateTime.Now:HH:mm:ss}] {message}");
    }

    private void WriteLogsToFile()
    {
        try
        {
            File.WriteAllText(logFilePath, logBuilder.ToString());
            UnityEngine.Debug.Log($"Logs written to: {logFilePath}");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Failed to write log file: {e.Message}");
        }
    }
    #endregion

    void StartPythonBridge(string fullPath)
    {
        try
        {
            pythonProcess = new Process();
            pythonProcess.StartInfo.FileName = pythonPath;
            pythonProcess.StartInfo.Arguments = $"\"{fullPath}\"";
            pythonProcess.StartInfo.UseShellExecute = false;
            pythonProcess.StartInfo.RedirectStandardOutput = true;
            pythonProcess.StartInfo.RedirectStandardError = true;
            pythonProcess.StartInfo.CreateNoWindow = false;
            pythonProcess.StartInfo.WorkingDirectory = Directory.GetParent(Application.dataPath).FullName;

            pythonProcess.OutputDataReceived += (sender, args) =>
            {
                if (!string.IsNullOrEmpty(args.Data))
                {
                    UnityEngine.Debug.Log($"[Python] {args.Data}");
                    Log($"[Python] {args.Data}");
                }
            };

            pythonProcess.ErrorDataReceived += (sender, args) =>
            {
                if (!string.IsNullOrEmpty(args.Data))
                {
                    UnityEngine.Debug.LogError($"[Python Error] {args.Data}");
                    Log($"[Python Error] {args.Data}");
                }
            };

            pythonProcess.Start();
            pythonProcess.BeginOutputReadLine();
            pythonProcess.BeginErrorReadLine();

            UnityEngine.Debug.Log("Python bridge started successfully");
            Log("Python bridge started successfully");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Failed to start Python bridge: {e.Message}");
            Log($"[ERROR] Failed to start Python bridge: {e.Message}");
        }
    }

    void InitializeNetwork()
    {
        try
        {
            sendClient = new UdpClient();
            receiveClient = new UdpClient(receivePort);
            receiveClient.Client.ReceiveTimeout = 100;
            isConnected = true;

            UnityEngine.Debug.Log($"Network initialized - Sending on port {sendPort}, Receiving on port {receivePort}");
            Log($"Network initialized - Sending on port {sendPort}, Receiving on port {receivePort}");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Network initialization failed: {e.Message}");
            Log($"[ERROR] Network initialization failed: {e.Message}");
        }
    }

    void Update()
    {
        if (!isConnected) return;

        SendControls();
        ReceiveState();

        // Periodic debug logs
        if (showDebugLogs)
        {
            debugTimer += Time.deltaTime;
            if (debugTimer >= 3f)
            {
                string netStatus = $"Messages Sent: {messagesSent}, Messages Received: {messagesReceived}";
                UnityEngine.Debug.Log($"=== NETWORK STATUS ===\n{netStatus}");
                Log($"NETWORK STATUS | {netStatus}");

                if (messagesReceived == 0)
                {
                    UnityEngine.Debug.LogWarning("NOT RECEIVING DATA FROM PYTHON!");
                    UnityEngine.Debug.LogWarning("Check Python script / console output.");
                    Log("[WARNING] NOT RECEIVING DATA FROM PYTHON!");
                }

                // Show detailed state if enabled
                if (showDetailedState && currentState != null)
                {
                    var pos = currentState.position;
                    var orient = currentState.orientation;
                    UnityEngine.Debug.Log($"Aircraft State - Unity Pos: ({pos.unity_x:F1}, {pos.unity_y:F1}, {pos.unity_z:F1})");
                    UnityEngine.Debug.Log($"Orientation - Roll: {orient.roll * Mathf.Rad2Deg:F1}°, Pitch: {orient.pitch * Mathf.Rad2Deg:F1}°, Yaw: {orient.yaw * Mathf.Rad2Deg:F1}°");
                    if (currentState.velocity != null)
                        UnityEngine.Debug.Log($"Airspeed: {currentState.velocity.airspeed:F1} kts");
                }

                debugTimer = 0f;
            }
        }
    }

    void SendControls()
    {
        try
        {
            ControlInputs controls = new ControlInputs
            {
                throttle = throttle,
                aileron = roll,      // Roll control maps to aileron
                elevator = pitch,    // Pitch control maps to elevator
                rudder = yaw        // Yaw control maps to rudder
            };

            string json = JsonConvert.SerializeObject(controls);
            byte[] data = Encoding.UTF8.GetBytes(json);

            sendClient.Send(data, data.Length, "localhost", sendPort);
            messagesSent++;

            // Log throttle changes
            if (Mathf.Abs(throttle - lastThrottle) > 0.01f)
            {
                Log($"Throttle changed: {lastThrottle:F2} -> {throttle:F2}");
                lastThrottle = throttle;
            }
        }
        catch (Exception e)
        {
            if (showDebugLogs)
                UnityEngine.Debug.LogError($"Send failed: {e.Message}");
            Log($"[ERROR] Send failed: {e.Message}");
        }
    }

    void ReceiveState()
    {
        try
        {
            IPEndPoint endPoint = new IPEndPoint(IPAddress.Any, receivePort);
            byte[] data = receiveClient.Receive(ref endPoint);
            string json = Encoding.UTF8.GetString(data);

            currentState = JsonConvert.DeserializeObject<DroneState>(json);
            messagesReceived++;

            if (showDebugLogs && messagesReceived == 1)
            {
                UnityEngine.Debug.Log("✓ First message received from JSBSim! Communication working.");
                Log("✓ First message received from JSBSim! Communication working.");
            }
        }
        catch (SocketException)
        {
            // Timeout is expected — no log needed
        }
        catch (Exception e)
        {
            if (showDebugLogs)
                UnityEngine.Debug.LogError($"Receive failed: {e.Message}");
            Log($"[ERROR] Receive failed: {e.Message}");
        }
    }

    public DroneState GetCurrentState()
    {
        return currentState;
    }

    public Vector3 GetUnityPosition()
    {
        if (currentState?.position != null)
        {
            return new Vector3(currentState.position.unity_x, 
                             currentState.position.unity_y, 
                             currentState.position.unity_z);
        }
        return Vector3.zero;
    }

    public Quaternion GetUnityRotation()
    {
        if (currentState?.orientation != null)
        {
            float roll = currentState.orientation.roll * Mathf.Rad2Deg;
            float pitch = currentState.orientation.pitch * Mathf.Rad2Deg;
            float yaw = currentState.orientation.yaw * Mathf.Rad2Deg;
            
            // Convert from aviation coordinates to Unity coordinates
            return Quaternion.Euler(-pitch, yaw, -roll);
        }
        return Quaternion.identity;
    }

    void OnApplicationQuit()
    {
        Log("=== Application Quit Triggered ===");

        if (pythonProcess != null && !pythonProcess.HasExited)
        {
            pythonProcess.Kill();
            UnityEngine.Debug.Log("Python process terminated");
            Log("Python process terminated");
        }

        if (sendClient != null) sendClient.Close();
        if (receiveClient != null) receiveClient.Close();

        Log($"Final Message Count - Sent: {messagesSent}, Received: {messagesReceived}");
        Log("=== End of Session ===");

        WriteLogsToFile();
    }

    void OnGUI()
    {
        if (!showDebugLogs) return;

        GUILayout.BeginArea(new Rect(Screen.width - 350, 10, 340, 200));
        GUILayout.Label("=== JSBSim Manager ===");
        GUILayout.Label($"Python Running: {(pythonProcess != null && !pythonProcess.HasExited ? "Yes" : "No")}");
        GUILayout.Label($"Network: {(isConnected ? "Connected" : "Disconnected")}");
        GUILayout.Label($"Sent: {messagesSent} | Received: {messagesReceived}");
        GUILayout.Label($"Data Status: {(currentState != null ? "✓ OK" : "✗ None")}");
        
        GUILayout.Space(10);
        GUILayout.Label("=== Flight Controls ===");
        GUILayout.Label($"Throttle: {throttle:F2}");
        GUILayout.Label($"Roll: {roll:F2} | Pitch: {pitch:F2} | Yaw: {yaw:F2}");
        
        if (currentState?.position != null)
        {
            var pos = currentState.position;
            GUILayout.Label($"Unity Pos: ({pos.unity_x:F1}, {pos.unity_y:F1}, {pos.unity_z:F1})");
        }
        
        if (currentState?.velocity != null)
        {
            GUILayout.Label($"Airspeed: {currentState.velocity.airspeed:F1} kts");
        }
        
        GUILayout.EndArea();
    }
}