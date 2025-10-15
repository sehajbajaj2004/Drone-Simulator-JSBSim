using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Diagnostics;
using System.IO;
using Newtonsoft.Json;
using System.Collections.Generic;
using System.Text; // For StringBuilder

#region Serializable Classes
[Serializable]
public class DroneState
{
    public Position position;
    public Orientation orientation;
    public Velocity velocity;
    public AngularVelocity angular_velocity;
}

[Serializable]
public class Position
{
    public float lat;
    public float lon;
    public float alt;
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
}

[Serializable]
public class AngularVelocity
{
    public float p;
    public float q;
    public float r;
}

[Serializable]
public class ControlInputs
{
    public float throttle;
    public float aileron;
    public float elevator;
    public float rudder;
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

    [Header("Control Inputs")]
    public float throttle = 0f;
    public float aileron = 0f;
    public float elevator = 0f;
    public float rudder = 0f;

    [Header("Debug")]
    public bool showDebugLogs = true;
    public int messagesReceived = 0;
    public int messagesSent = 0;

    private UdpClient sendClient;
    private UdpClient receiveClient;
    private Process pythonProcess;
    private DroneState currentState;
    private bool isConnected = false;

    private float debugTimer = 0f;

    // ========= Log Collection =========
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
        // Create Logs directory if not exists
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
                aileron = aileron,
                elevator = elevator,
                rudder = rudder
            };

            string json = JsonConvert.SerializeObject(controls);
            byte[] data = Encoding.UTF8.GetBytes(json);

            sendClient.Send(data, data.Length, "localhost", sendPort);
            messagesSent++;
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

        GUILayout.BeginArea(new Rect(Screen.width - 310, 10, 300, 150));
        GUILayout.Label("=== JSBSim Manager ===");
        GUILayout.Label($"Python Running: {(pythonProcess != null && !pythonProcess.HasExited ? "Yes" : "No")}");
        GUILayout.Label($"Network: {(isConnected ? "Connected" : "Disconnected")}");
        GUILayout.Label($"Sent: {messagesSent} | Received: {messagesReceived}");
        GUILayout.Label($"Data Status: {(currentState != null ? "✓ OK" : "✗ None")}");
        GUILayout.EndArea();
    }
}
