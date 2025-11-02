import jsbsim
import socket
import json
import sys
import time
import math

class JSBSimBridge:
    def __init__(self, port=5555):
        self.port = port
        self.fdm = None
        self.running = False
        self.origin_lat = 37.4
        self.origin_lon = -122.4
        self.origin_alt = 100.0
        
    def initialize(self, aircraft='c172p'):
        try:
            print("[DEBUG] Initializing JSBSim...")
            print(f"[DEBUG] JSBSim root directory: {jsbsim.get_default_root_dir()}")
            
            # Initialize JSBSim
            self.fdm = jsbsim.FGFDMExec(None)
            
            print(f"[DEBUG] Loading aircraft: {aircraft}")
            
            # Load aircraft - try multiple default aircraft
            aircraft_list = ['c172p', 'c172x', 'f16', 'T37', 'ah1s']
            loaded = False
            
            for ac in aircraft_list:
                if self.fdm.load_model(ac):
                    print(f"[SUCCESS] Loaded aircraft: {ac}")
                    loaded = True
                    break
                else:
                    print(f"[WARNING] Failed to load aircraft: {ac}")
            
            if not loaded:
                print("[ERROR] Failed to load any aircraft model")
                return False
            
            print("[DEBUG] Aircraft loaded successfully")
            
            # Set initial conditions at origin
            print("[DEBUG] Setting initial conditions...")
            self.fdm['ic/h-sl-ft'] = self.origin_alt
            self.fdm['ic/lat-geod-deg'] = self.origin_lat
            self.fdm['ic/long-gc-deg'] = self.origin_lon
            self.fdm['ic/psi-true-deg'] = 0.0  # Heading
            self.fdm['ic/theta-deg'] = 0.0     # Pitch
            self.fdm['ic/phi-deg'] = 0.0       # Roll
            self.fdm['ic/u-fps'] = 0.0         # Forward velocity
            self.fdm['ic/v-fps'] = 0.0         # Side velocity
            self.fdm['ic/w-fps'] = 0.0         # Vertical velocity
            self.fdm['ic/p-rad_sec'] = 0.0     # Roll rate
            self.fdm['ic/q-rad_sec'] = 0.0     # Pitch rate
            self.fdm['ic/r-rad_sec'] = 0.0     # Yaw rate
            
            # Initialize engines
            self.fdm['propulsion/engine[0]/set-running'] = 1
            self.fdm['propulsion/starter_cmd'] = 1
            self.fdm['propulsion/cutoff_cmd'] = 0
            
            # Set some initial control positions
            self.fdm['fcs/throttle-cmd-norm[0]'] = 0.0
            self.fdm['fcs/aileron-cmd-norm'] = 0.0
            self.fdm['fcs/elevator-cmd-norm'] = 0.0
            self.fdm['fcs/rudder-cmd-norm'] = 0.0
            
            print("[DEBUG] Running initial conditions...")
            # Run initial conditions
            if not self.fdm.run_ic():
                print("[ERROR] Failed to run initial conditions")
                return False
            
            # Run a few steps to stabilize
            for i in range(10):
                self.fdm.run()
            
            print("[SUCCESS] JSBSim initialized successfully!")
            return True
            
        except Exception as e:
            print(f"[ERROR] Error initializing JSBSim: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def step(self, dt=0.02):
        """Run one simulation step"""
        if self.fdm:
            result = self.fdm.run()
            if result:
                return self.get_state()
        return None
    
    def get_state(self):
        """Get current aircraft state"""
        if not self.fdm:
            return None
            
        try:
            # Get raw position data
            lat = float(self.fdm['position/lat-geod-deg'])
            lon = float(self.fdm['position/long-gc-deg'])
            alt = float(self.fdm['position/h-sl-ft'])
            
            # Convert to Unity coordinates (relative to origin)
            # Unity coordinates: X = East-West, Y = Up-Down, Z = North-South
            lat_diff = lat - self.origin_lat
            lon_diff = lon - self.origin_lon
            alt_diff = alt - self.origin_alt
            
            # Convert degrees to meters (approximate)
            meters_per_degree_lat = 111320.0  # meters per degree latitude
            meters_per_degree_lon = 111320.0 * math.cos(math.radians(lat))  # longitude varies with latitude
            meters_per_foot = 0.3048
            
            unity_x = lon_diff * meters_per_degree_lon  # East-West
            unity_y = alt_diff * meters_per_foot        # Up-Down
            unity_z = lat_diff * meters_per_degree_lat  # North-South
            
            state = {
                'position': {
                    'lat': lat,
                    'lon': lon,
                    'alt': alt,
                    'unity_x': unity_x,
                    'unity_y': unity_y,
                    'unity_z': unity_z
                },
                'orientation': {
                    'roll': float(self.fdm['attitude/roll-rad']),
                    'pitch': float(self.fdm['attitude/pitch-rad']),
                    'yaw': float(self.fdm['attitude/psi-rad'])
                },
                'velocity': {
                    'u': float(self.fdm['velocities/u-fps']),
                    'v': float(self.fdm['velocities/v-fps']),
                    'w': float(self.fdm['velocities/w-fps']),
                    'airspeed': float(self.fdm['velocities/vc-kts'])
                },
                'angular_velocity': {
                    'p': float(self.fdm['velocities/p-rad_sec']),
                    'q': float(self.fdm['velocities/q-rad_sec']),
                    'r': float(self.fdm['velocities/r-rad_sec'])
                },
                'engine': {
                    'rpm': float(self.fdm['propulsion/engine[0]/engine-rpm']),
                    'thrust': float(self.fdm['propulsion/engine[0]/thrust-lbs'])
                }
            }
            return state
        except Exception as e:
            print(f"[ERROR] Failed to get state: {e}")
            return None
    
    def set_controls(self, throttle=0, aileron=0, elevator=0, rudder=0):
        """Set control inputs - now properly mapped to flight controls"""
        if self.fdm:
            try:
                # Clamp values to valid ranges
                throttle = max(0.0, min(1.0, throttle))      # Throttle: 0 to 1
                aileron = max(-1.0, min(1.0, aileron))       # Roll: -1 to 1
                elevator = max(-1.0, min(1.0, elevator))     # Pitch: -1 to 1
                rudder = max(-1.0, min(1.0, rudder))         # Yaw: -1 to 1
                
                # Set controls
                self.fdm['fcs/throttle-cmd-norm[0]'] = throttle
                self.fdm['fcs/aileron-cmd-norm'] = aileron
                self.fdm['fcs/elevator-cmd-norm'] = elevator
                self.fdm['fcs/rudder-cmd-norm'] = rudder
                
                # Also set mixture and prop controls for better engine response
                self.fdm['fcs/mixture-cmd-norm[0]'] = 1.0 if throttle > 0 else 0.8
                
            except Exception as e:
                print(f"[ERROR] Failed to set controls: {e}")
    
    def run_server(self):
        """Run UDP server for Unity communication"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('localhost', self.port))
            sock.settimeout(0.001)
            
            print(f"[SUCCESS] JSBSim bridge listening on port {self.port}")
            print(f"[INFO] Sending data to Unity on port {self.port + 1}")
            print("[INFO] Ready to receive control inputs from Unity...")
            print("[INFO] Origin coordinates set to: Lat={}, Lon={}, Alt={}ft".format(
                self.origin_lat, self.origin_lon, self.origin_alt))
            
            self.running = True
            last_time = time.time()
            
            # Statistics
            packets_received = 0
            packets_sent = 0
            last_status_time = time.time()
            
            # Default controls
            controls = {'throttle': 0, 'aileron': 0, 'elevator': 0, 'rudder': 0}
            
            while self.running:
                try:
                    # Receive control commands from Unity
                    data, addr = sock.recvfrom(1024)
                    controls = json.loads(data.decode())
                    packets_received += 1
                    
                    self.set_controls(
                        controls.get('throttle', 0),
                        controls.get('aileron', 0),
                        controls.get('elevator', 0),
                        controls.get('rudder', 0)
                    )
                except socket.timeout:
                    pass
                except Exception as e:
                    print(f"[WARNING] Error receiving data: {e}")
                
                # Run simulation step
                current_time = time.time()
                dt = current_time - last_time
                
                if dt >= 0.02:  # 50 Hz update rate
                    state = self.step()
                    
                    if state:
                        # Send state back to Unity
                        response = json.dumps(state).encode()
                        try:
                            sock.sendto(response, ('localhost', self.port + 1))
                            packets_sent += 1
                        except Exception as e:
                            print(f"[WARNING] Error sending data: {e}")
                    
                    last_time = current_time
                
                # Print status every 5 seconds
                if current_time - last_status_time >= 5.0:
                    print(f"[STATUS] Packets - Received: {packets_received}, Sent: {packets_sent}")
                    if state:
                        pos = state['position']
                        print(f"[STATUS] Unity Pos: X={pos['unity_x']:.1f}m, Y={pos['unity_y']:.1f}m, Z={pos['unity_z']:.1f}m")
                        print(f"[STATUS] Controls: T={controls.get('throttle', 0):.2f}, A={controls.get('aileron', 0):.2f}, E={controls.get('elevator', 0):.2f}, R={controls.get('rudder', 0):.2f}")
                    last_status_time = current_time
            
            sock.close()
            print("[INFO] Server stopped")
            
        except Exception as e:
            print(f"[ERROR] Server error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == '__main__':
    # Print debug info
    print("[DEBUG] Python Executable:", sys.executable)
    print("[DEBUG] Python Version:", sys.version)
    
    bridge = JSBSimBridge()
    
    # Try to initialize with available aircraft
    if bridge.initialize():
        print("[INFO] Starting server loop...")
        try:
            bridge.run_server()
        except KeyboardInterrupt:
            print("[INFO] Shutting down...")
            bridge.running = False
    else:
        print("[FATAL] Failed to initialize JSBSim")
        print("[INFO] Checking available aircraft...")
        try:
            import os
            aircraft_dir = os.path.join(jsbsim.get_default_root_dir(), 'aircraft')
            if os.path.exists(aircraft_dir):
                print(f"[INFO] Aircraft directory: {aircraft_dir}")
                print("[INFO] Available aircraft:")
                for item in sorted(os.listdir(aircraft_dir))[:15]:
                    print(f"  - {item}")
        except:
            pass
        sys.exit(1)