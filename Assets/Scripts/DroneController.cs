using UnityEngine;

public class DroneController : MonoBehaviour
{
    private JSBSimManager jsbsim;
    
    [Header("Initial Position")]
    public Vector3 originLatLonAlt = new Vector3(37.4f, -122.4f, 100f);
    
    [Header("Conversion Settings")]
    public float metersPerFoot = 0.3048f;
    public float degreesToMeters = 111320f; // Approximate at equator
    
    void Start()
    {
        jsbsim = FindObjectOfType<JSBSimManager>();
        
        if (jsbsim == null)
        {
            Debug.LogError("JSBSimManager not found!");
        }
    }
    
    void Update()
    {
        if (jsbsim == null) return;
        
        // Get keyboard input for controls
        HandleInput();
        
        // Update drone position and rotation from JSBSim
        UpdateTransform();
    }
    
    void HandleInput()
    {
        // Throttle (W/S or Up/Down arrows)
        float throttleInput = 0;
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            throttleInput = 1f;
        else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            throttleInput = -1f;
        
        jsbsim.throttle = Mathf.Clamp01(jsbsim.throttle + throttleInput * Time.deltaTime);
        
        // Aileron (A/D or Left/Right arrows)
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            jsbsim.aileron = -1f;
        else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            jsbsim.aileron = 1f;
        else
            jsbsim.aileron = 0f;
        
        // Elevator (Q/E)
        if (Input.GetKey(KeyCode.Q))
            jsbsim.elevator = 1f;
        else if (Input.GetKey(KeyCode.E))
            jsbsim.elevator = -1f;
        else
            jsbsim.elevator = 0f;
        
        // Rudder (Z/X)
        if (Input.GetKey(KeyCode.Z))
            jsbsim.rudder = -1f;
        else if (Input.GetKey(KeyCode.X))
            jsbsim.rudder = 1f;
        else
            jsbsim.rudder = 0f;
    }
    
    void UpdateTransform()
    {
        DroneState state = jsbsim.GetCurrentState();
        
        if (state == null || state.position == null) return;
        
        // Convert lat/lon/alt to Unity coordinates
        float x = (state.position.lon - originLatLonAlt.y) * degreesToMeters;
        float z = (state.position.lat - originLatLonAlt.x) * degreesToMeters;
        float y = (state.position.alt - originLatLonAlt.z) * metersPerFoot;
        
        transform.position = new Vector3(x, y, z);
        
        // Convert roll, pitch, yaw to Unity rotation
        if (state.orientation != null)
        {
            float roll = state.orientation.roll * Mathf.Rad2Deg;
            float pitch = state.orientation.pitch * Mathf.Rad2Deg;
            float yaw = state.orientation.yaw * Mathf.Rad2Deg;
            
            transform.rotation = Quaternion.Euler(-pitch, yaw, -roll);
        }
    }
}