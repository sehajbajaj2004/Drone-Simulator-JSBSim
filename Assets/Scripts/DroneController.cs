using UnityEngine;

public class DroneController : MonoBehaviour
{
    private JSBSimManager jsbsim;
    
    [Header("Input Settings")]
    public bool useKeyboardInput = true;
    public bool useMouseForView = true;
    
    [Header("Control Sensitivity")]
    [Range(0.1f, 5.0f)]
    public float throttleSensitivity = 1.0f;
    
    [Range(0.1f, 5.0f)]
    public float pitchSensitivity = 2.0f;
    
    [Range(0.1f, 5.0f)]
    public float rollSensitivity = 2.0f;
    
    [Range(0.1f, 5.0f)]
    public float yawSensitivity = 1.5f;
    
    [Header("Control Smoothing")]
    [Range(0.1f, 10.0f)]
    public float controlSmoothing = 5.0f;
    
    [Header("Display")]
    public bool showControlsOnScreen = true;
    
    // Internal control values for smoothing
    private float targetThrottle = 0f;
    private float targetRoll = 0f;
    private float targetPitch = 0f;
    private float targetYaw = 0f;
    
    void Start()
    {
        jsbsim = FindObjectOfType<JSBSimManager>();
        
        if (jsbsim == null)
        {
            Debug.LogError("JSBSimManager not found! Please add JSBSimManager to the scene.");
            enabled = false;
            return;
        }
        
        Debug.Log("DroneController initialized successfully!");
        Debug.Log("Controls: WASD/Arrow Keys = Pitch/Roll, QE = Yaw, RF/Shift+Ctrl = Throttle");
    }
    
    void Update()
    {
        if (jsbsim == null) return;
        
        if (useKeyboardInput)
        {
            HandleKeyboardInput();
        }
        
        // Smooth control transitions
        SmoothControls();
        
        // Apply controls to JSBSim
        ApplyControls();
        
        // Update drone transform from JSBSim state
        UpdateTransform();
    }
    
    void HandleKeyboardInput()
    {
        float deltaTime = Time.deltaTime;
        
        // Throttle Control (R/F keys or Shift/Ctrl)
        if (Input.GetKey(KeyCode.R) || Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            targetThrottle += throttleSensitivity * deltaTime;
        }
        else if (Input.GetKey(KeyCode.F) || Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
        {
            targetThrottle -= throttleSensitivity * deltaTime;
        }
        
        // Pitch Control (W/S or Up/Down arrows) - Elevator
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            targetPitch = pitchSensitivity; // Nose up
        }
        else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {
            targetPitch = -pitchSensitivity; // Nose down
        }
        else
        {
            targetPitch = 0f;
        }
        
        // Roll Control (A/D or Left/Right arrows) - Aileron
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            targetRoll = -rollSensitivity; // Roll left
        }
        else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            targetRoll = rollSensitivity; // Roll right
        }
        else
        {
            targetRoll = 0f;
        }
        
        // Yaw Control (Q/E) - Rudder
        if (Input.GetKey(KeyCode.Q))
        {
            targetYaw = -yawSensitivity; // Yaw left
        }
        else if (Input.GetKey(KeyCode.E))
        {
            targetYaw = yawSensitivity; // Yaw right
        }
        else
        {
            targetYaw = 0f;
        }
        
        // Clamp throttle to valid range
        targetThrottle = Mathf.Clamp01(targetThrottle);
        
        // Clamp other controls to valid ranges
        targetRoll = Mathf.Clamp(targetRoll, -1f, 1f);
        targetPitch = Mathf.Clamp(targetPitch, -1f, 1f);
        targetYaw = Mathf.Clamp(targetYaw, -1f, 1f);
    }
    
    void SmoothControls()
    {
        float smoothRate = controlSmoothing * Time.deltaTime;
        
        // Smooth throttle (slower response)
        jsbsim.throttle = Mathf.Lerp(jsbsim.throttle, targetThrottle, smoothRate * 0.5f);
        
        // Smooth flight controls
        jsbsim.roll = Mathf.Lerp(jsbsim.roll, targetRoll, smoothRate);
        jsbsim.pitch = Mathf.Lerp(jsbsim.pitch, targetPitch, smoothRate);
        jsbsim.yaw = Mathf.Lerp(jsbsim.yaw, targetYaw, smoothRate);
    }
    
    void ApplyControls()
    {
        // Controls are automatically sent by JSBSimManager.Update()
        // This method is here for any additional control logic if needed
    }
    
    void UpdateTransform()
    {
        if (jsbsim == null) return;
        
        // Get Unity position and rotation from JSBSim
        Vector3 unityPosition = jsbsim.GetUnityPosition();
        Quaternion unityRotation = jsbsim.GetUnityRotation();
        
        // Apply to transform
        transform.position = unityPosition;
        transform.rotation = unityRotation;
        
        // Optional: Add some debug logging for significant position changes
        if (Vector3.Distance(transform.position, unityPosition) > 1f)
        {
            Debug.Log($"Drone moved to Unity position: {unityPosition}");
        }
    }
    
    void OnGUI()
    {
        if (!showControlsOnScreen) return;
        
        // Display controls on screen
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("=== Flight Controls ===");
        GUILayout.Label("Throttle: R (Up) / F (Down) or Shift/Ctrl");
        GUILayout.Label("Pitch: W (Up) / S (Down) or ↑/↓");
        GUILayout.Label("Roll: A (Left) / D (Right) or ←/→");
        GUILayout.Label("Yaw: Q (Left) / E (Right)");
        GUILayout.Space(10);
        GUILayout.Label("=== Current Values ===");
        if (jsbsim != null)
        {
            GUILayout.Label($"Throttle: {jsbsim.throttle:F2}");
            GUILayout.Label($"Roll: {jsbsim.roll:F2}");
            GUILayout.Label($"Pitch: {jsbsim.pitch:F2}");
            GUILayout.Label($"Yaw: {jsbsim.yaw:F2}");
        }
        GUILayout.EndArea();
    }
    
    // Public methods for external control (e.g., from UI or other scripts)
    public void SetThrottle(float value)
    {
        targetThrottle = Mathf.Clamp01(value);
    }
    
    public void SetRoll(float value)
    {
        targetRoll = Mathf.Clamp(value, -1f, 1f);
    }
    
    public void SetPitch(float value)
    {
        targetPitch = Mathf.Clamp(value, -1f, 1f);
    }
    
    public void SetYaw(float value)
    {
        targetYaw = Mathf.Clamp(value, -1f, 1f);
    }
    
    public void ResetControls()
    {
        targetThrottle = 0f;
        targetRoll = 0f;
        targetPitch = 0f;
        targetYaw = 0f;
    }
}