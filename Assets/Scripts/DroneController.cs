using UnityEngine;

public class DroneController : MonoBehaviour
{
    private JSBSimManager jsbsim;

    [Header("Input Settings")]
    public bool useKeyboardInput = true;

    [Header("Sensitivity")]
    [Range(0.1f, 5.0f)] public float throttleSensitivity = 1.2f;
    [Range(0.1f, 5.0f)] public float pitchSensitivity = 2.0f;
    [Range(0.1f, 5.0f)] public float rollSensitivity = 2.0f;
    [Range(0.1f, 5.0f)] public float yawSensitivity = 1.5f;

    [Header("Smoothing")]
    [Range(0.1f, 10.0f)] public float controlSmoothing = 6.0f;

    [Header("HUD")]
    public bool showControlsOnScreen = true;

    private float targetThrottle = 0f, targetRoll = 0f, targetPitch = 0f, targetYaw = 0f;

    void Start()
    {
        jsbsim = FindObjectOfType<JSBSimManager>();
        if (jsbsim == null) { Debug.LogError("JSBSimManager not found!"); enabled = false; return; }
        Debug.Log("Press J to start the engine. Use R/F or Shift/Ctrl (Throttle), WASD or Arrows (Roll/Pitch), Q/E (Yaw).");
    }

    void Update()
    {
        if (useKeyboardInput) HandleKeyboard();
        SmoothControls();
        transform.position = jsbsim.GetUnityPosition();
        transform.rotation = jsbsim.GetUnityRotation();
    }

    void HandleKeyboard()
    {
        float dt = Time.deltaTime;

        // Throttle (collective in rotorcraft)
        if (Input.GetKey(KeyCode.R) || Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            targetThrottle += throttleSensitivity * dt;
        else if (Input.GetKey(KeyCode.F) || Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
            targetThrottle -= throttleSensitivity * dt;

        // Pitch (longitudinal)
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            targetPitch = pitchSensitivity;
        else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            targetPitch = -pitchSensitivity;
        else
            targetPitch = 0f;

        // Roll (lateral)
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            targetRoll = -rollSensitivity;
        else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            targetRoll = rollSensitivity;
        else
            targetRoll = 0f;

        // Yaw (anti-torque)
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

        // Push to manager each frame
        jsbsim.throttle = targetThrottle;
        jsbsim.roll = targetRoll;
        jsbsim.pitch = targetPitch;
        jsbsim.yaw = targetYaw;
    }

    void SmoothControls()
    {
        float t = controlSmoothing * Time.deltaTime;
        jsbsim.throttle = Mathf.Lerp(jsbsim.throttle, targetThrottle, t * 0.5f);
        jsbsim.roll     = Mathf.Lerp(jsbsim.roll,     targetRoll,     t);
        jsbsim.pitch    = Mathf.Lerp(jsbsim.pitch,    targetPitch,    t);
        jsbsim.yaw      = Mathf.Lerp(jsbsim.yaw,      targetYaw,      t);
    }

    void OnGUI()
    {
        if (!showControlsOnScreen) return;
        GUILayout.BeginArea(new Rect(10,10,360,220));
        GUILayout.Label("=== Drone Controls ===");
        GUILayout.Label("J: Start Engine (clears all brakes)");
        GUILayout.Label("Throttle: R / Shift (+), F / Ctrl (-)");
        GUILayout.Label("Pitch: W / S  |  Arrows ↑/↓");
        GUILayout.Label("Roll:  A / D  |  Arrows ←/→");
        GUILayout.Label("Yaw:   Q / E");
        GUILayout.Space(8);
        GUILayout.Label($"Throttle: {jsbsim.throttle:F2}  Roll: {jsbsim.roll:F2}  Pitch: {jsbsim.pitch:F2}  Yaw: {jsbsim.yaw:F2}");
        GUILayout.EndArea();
    }
}
