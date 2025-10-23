import jsbsim
import socket
import json
import sys
import time

class JSBSimBridge:
    def __init__(self, port=5555):
        self.port = port
        self.fdm = None
        self.running = False
        
    def initialize(self, aircraft='ah1s'):  # Changed to use Cessna 172 (comes with JSBSim)
        try:
            print("[DEBUG] Initializing JSBSim...")
            print(f"[DEBUG] JSBSim root directory: {jsbsim.get_default_root_dir()}")
            
            # Initialize JSBSim with default aircraft data
            self.fdm = jsbsim.FGFDMExec(None)
            
            print(f"[DEBUG] Loading aircraft: {aircraft}")
            
            # Load aircraft - using default Cessna 172
            if not self.fdm.load_model(aircraft):
                print(f"[ERROR] Failed to load aircraft model: {aircraft}")
                return False
            
            print("[DEBUG] Aircraft loaded successfully")
            
            # Set initial conditions
            print("[DEBUG] Setting initial conditions...")
            self.fdm['ic/h-sl-ft'] = 100.0  # Initial altitude (100 feet)
            self.fdm['ic/lat-geod-deg'] = 37.4
            self.fdm['ic/long-gc-deg'] = -122.4
            self.fdm['ic/psi-true-deg'] = 0.0
            self.fdm['ic/u-fps'] = 0.0
            self.fdm['ic/v-fps'] = 0.0
            self.fdm['ic/w-fps'] = 0.0
            self.fdm['ic/p-rad_sec'] = 0.0
            self.fdm['ic/q-rad_sec'] = 0.0
            self.fdm['ic/r-rad_sec'] = 0.0
            
            # Initialize engines
            self.fdm['propulsion/engine[0]/set-running'] = 1
            
            print("[DEBUG] Running initial conditions...")
            # Run initial conditions
            if not self.fdm.run_ic():
                print("[ERROR] Failed to run initial conditions")
                return False
            
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
            state = {
                'position': {
                    'lat': float(self.fdm['position/lat-geod-deg']),
                    'lon': float(self.fdm['position/long-gc-deg']),
                    'alt': float(self.fdm['position/h-sl-ft'])
                },
                'orientation': {
                    'roll': float(self.fdm['attitude/roll-rad']),
                    'pitch': float(self.fdm['attitude/pitch-rad']),
                    'yaw': float(self.fdm['attitude/psi-rad'])
                },
                'velocity': {
                    'u': float(self.fdm['velocities/u-fps']),
                    'v': float(self.fdm['velocities/v-fps']),
                    'w': float(self.fdm['velocities/w-fps'])
                },
                'angular_velocity': {
                    'p': float(self.fdm['velocities/p-rad_sec']),
                    'q': float(self.fdm['velocities/q-rad_sec']),
                    'r': float(self.fdm['velocities/r-rad_sec'])
                }
            }
            return state
        except Exception as e:
            print(f"[ERROR] Failed to get state: {e}")
            return None
    
    def set_controls(self, throttle=0, aileron=0, elevator=0, rudder=0):
        """Set control inputs"""
        if self.fdm:
            try:
                # Clamp values to valid ranges
                throttle = max(0.0, min(1.0, throttle))
                aileron = max(-1.0, min(1.0, aileron))
                elevator = max(-1.0, min(1.0, elevator))
                rudder = max(-1.0, min(1.0, rudder))
                
                self.fdm['fcs/throttle-cmd-norm[0]'] = throttle
                self.fdm['fcs/aileron-cmd-norm'] = aileron
                self.fdm['fcs/elevator-cmd-norm'] = elevator
                self.fdm['fcs/rudder-cmd-norm'] = rudder
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
            
            self.running = True
            last_time = time.time()
            
            # Statistics
            packets_received = 0
            packets_sent = 0
            last_status_time = time.time()
            
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
                        print(f"[STATUS] Alt: {state['position']['alt']:.1f}ft, Throttle: {controls.get('throttle', 0):.2f}")
                    last_status_time = current_time
            
            sock.close()
            print("[INFO] Server stopped")
            
        except Exception as e:
            print(f"[ERROR] Server error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == '__main__':
    # Print debug info
    import sys
    print("[DEBUG] Python Executable:", sys.executable)
    print("[DEBUG] Python Version:", sys.version)
    
    bridge = JSBSimBridge()
    
    # Try to initialize with default Cessna 172
    # JSBSim comes with several default aircraft: c172p, f16, 737, etc.
    if bridge.initialize(aircraft='c172p'):
        print("[INFO] Starting server loop...")
        bridge.run_server()
    else:
        print("[FATAL] Failed to initialize JSBSim")
        print("[INFO] Checking available aircraft...")
        try:
            import os
            aircraft_dir = os.path.join(jsbsim.get_default_root_dir(), 'aircraft')
            if os.path.exists(aircraft_dir):
                print(f"[INFO] Aircraft directory: {aircraft_dir}")
                print("[INFO] Available aircraft:")
                for item in os.listdir(aircraft_dir)[:10]:  # Show first 10
                    print(f"  - {item}")
        except:
            pass
        sys.exit(1)