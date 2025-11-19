using UnityEngine;

public class DroneController : MonoBehaviour
{
    private JSBSimManager jsbsim;

    [Header("Input Settings")]
    public bool useKeyboardInput = true;
    public bool useMouseForView = true;

    [Header("Control Sensitivity")]
    [Range(0.1f, 5.0f)] public float throttleSensitivity = 1.0f;
    [Range(0.1f, 5.0f)] public float pitchSensitivity = 2.0f;
    [Range(0.1f, 5.0f)] public float rollSensitivity = 2.0f;
    [Range(0.1f, 5.0f)] public float yawSensitivity = 1.5f;

    [Header("Control Smoothing")]
    [Range(0.1f, 10.0f)] public float controlSmoothing = 5.0f;

    [Header("Display")]
    public bool showControlsOnScreen = true;

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

        Debug.Log("DroneController ready. Press J to start the engine.");
        Debug.Log("Controls: WASD/Arrows = Pitch/Roll, Q/E = Yaw, R(or Shift)/F(or Ctrl) = Throttle.");
    }

    void Update()
    {
        if (jsbsim == null) return;

        if (useKeyboardInput)
            HandleKeyboardInput();

        SmoothControls();
        ApplyControls();
        UpdateTransform();
    }

    void HandleKeyboardInput()
    {
        float dt = Time.deltaTime;

        // Throttle: R / Shift increase, F / Ctrl decrease
        if (Input.GetKey(KeyCode.R) || Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            targetThrottle += throttleSensitivity * dt;
        else if (Input.GetKey(KeyCode.F) || Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
            targetThrottle -= throttleSensitivity * dt;

        // Pitch (elevator)
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            targetPitch = pitchSensitivity;
        else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            targetPitch = -pitchSensitivity;
        else
            targetPitch = 0f;

        // Roll (aileron)
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            targetRoll = -rollSensitivity;
        else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            targetRoll = rollSensitivity;
        else
            targetRoll = 0f;

        // Yaw (rudder)
        if (Input.GetKey(KeyCode.Q))
            targetYaw = -yawSensitivity;
        else if (Input.GetKey(KeyCode.E))
            targetYaw = yawSensitivity;
        else
            targetYaw = 0f;

        targetThrottle = Mathf.Clamp01(targetThrottle);
        targetRoll = Mathf.Clamp(targetRoll, -1f, 1f);
        targetPitch = Mathf.Clamp(targetPitch, -1f, 1f);
        targetYaw = Mathf.Clamp(targetYaw, -1f, 1f);
    }

    void SmoothControls()
    {
        float t = controlSmoothing * Time.deltaTime;
        jsbsim.throttle = Mathf.Lerp(jsbsim.throttle, targetThrottle, t * 0.5f);
        jsbsim.roll     = Mathf.Lerp(jsbsim.roll,     targetRoll,     t);
        jsbsim.pitch    = Mathf.Lerp(jsbsim.pitch,    targetPitch,    t);
        jsbsim.yaw      = Mathf.Lerp(jsbsim.yaw,      targetYaw,      t);
    }

    void ApplyControls()
    {
        // JSBSimManager handles send/receive
    }

    void UpdateTransform()
    {
        Vector3 unityPosition = jsbsim.GetUnityPosition();
        Quaternion unityRotation = jsbsim.GetUnityRotation();
        transform.position = unityPosition;
        transform.rotation = unityRotation;
    }

    void OnGUI()
    {
        if (!showControlsOnScreen) return;

        GUILayout.BeginArea(new Rect(10, 10, 360, 220));
        GUILayout.Label("=== Flight Controls ===");
        GUILayout.Label("Press J to START ENGINE");
        GUILayout.Label("Throttle: R (Up) / F (Down) or Shift/Ctrl");
        GUILayout.Label("Pitch: W (Up) / S (Down) or ↑/↓");
        GUILayout.Label("Roll: A (Left) / D (Right) or ←/→");
        GUILayout.Label("Yaw: Q (Left) / E (Right)");
        GUILayout.Space(10);
        GUILayout.Label("=== Current Values ===");
        GUILayout.Label($"Throttle: {jsbsim.throttle:F2}");
        GUILayout.Label($"Roll: {jsbsim.roll:F2}");
        GUILayout.Label($"Pitch: {jsbsim.pitch:F2}");
        GUILayout.Label($"Yaw: {jsbsim.yaw:F2}");
        GUILayout.EndArea();
    }

    // Optional external setters
    public void SetThrottle(float value) { targetThrottle = Mathf.Clamp01(value); }
    public void SetRoll(float value)     { targetRoll = Mathf.Clamp(value, -1f, 1f); }
    public void SetPitch(float value)    { targetPitch = Mathf.Clamp(value, -1f, 1f); }
    public void SetYaw(float value)      { targetYaw = Mathf.Clamp(value, -1f, 1f); }
    public void ResetControls()          { targetThrottle = targetRoll = targetPitch = targetYaw = 0f; }
}
