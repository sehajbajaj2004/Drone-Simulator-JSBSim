import jsbsim
import socket
import json
import sys
import time
import math
import os
import traceback

class JSBSimBridge:
    def __init__(self, port=5555, prefer_rotorcraft=True):
        self.port = port
        self.fdm = None
        self.running = False

        # Treat origin_alt as terrain elevation (MSL)
        self.origin_lat = 37.4
        self.origin_lon = -122.4
        self.origin_alt = 100.0

        self.sim_dt = 0.01     # 100 Hz for smoother rotor control
        self.prefer_rotorcraft = prefer_rotorcraft
        self.model_name = None
        self.is_rotorcraft = False
        self.engine_running = False

    # ---------------- Initialization ----------------
    def initialize(self):
        try:
            print("[DEBUG] Initializing JSBSim...")
            root = jsbsim.get_default_root_dir()
            print(f"[DEBUG] JSBSim root: {root}")

            self.fdm = jsbsim.FGFDMExec(root)
            self.fdm.set_aircraft_path(os.path.join(root, "aircraft"))
            self.fdm.set_engine_path(os.path.join(root, "engine"))
            self.fdm.set_systems_path(os.path.join(root, "systems"))
            self.fdm.set_dt(self.sim_dt)

            # Prefer rotorcraft (include 'ah1s-jsbsim' which your log shows)
            rotor_candidates = [
                "quadrotor", "quad", "multirotor", "ah1s-jsbsim", "ah1s", "r22", "r44"
            ]
            fixed_candidates = ["c172p", "c172x", "T37", "f16"]

            candidates = (rotor_candidates + fixed_candidates) if self.prefer_rotorcraft \
                        else (fixed_candidates + rotor_candidates)

            loaded = False
            for ac in candidates:
                try:
                    if self.fdm.load_model(ac):
                        self.model_name = ac
                        # Consider anything not obviously fixed-wing as rotorcraft
                        self.is_rotorcraft = ac in rotor_candidates or "ah1s" in ac.lower()
                        print(f"[SUCCESS] Loaded aircraft: {ac} (rotorcraft={self.is_rotorcraft})")
                        loaded = True
                        break
                except Exception as e:
                    print(f"[WARNING] load_model('{ac}') failed: {e}")

            if not loaded:
                print("[ERROR] No aircraft could be loaded.")
                return False

            self._set_initial_conditions_on_ground()

            print("[DEBUG] Running initial conditions...")
            if not self.fdm.run_ic():
                print("[ERROR] run_ic failed")
                return False

            # Settle integrators
            for _ in range(50):
                self.fdm.run()

            print("[SUCCESS] Initialized: on ground, engine OFF, zero velocity.")
            return True

        except Exception as e:
            print(f"[ERROR] initialize failed: {e}")
            traceback.print_exc()
            return False

    def _set_initial_conditions_on_ground(self):
        # Position
        self.fdm["ic/lat-geod-deg"] = self.origin_lat
        self.fdm["ic/long-gc-deg"] = self.origin_lon
        self.fdm["ic/terrain-elevation-ft"] = self.origin_alt
        self.fdm["ic/h-agl-ft"] = 0.0

        # Attitude & rates
        self.fdm["ic/psi-true-deg"] = 0.0
        self.fdm["ic/theta-deg"] = 0.0
        self.fdm["ic/phi-deg"] = 0.0
        self.fdm["ic/u-fps"] = 0.0
        self.fdm["ic/v-fps"] = 0.0
        self.fdm["ic/w-fps"] = 0.0
        self.fdm["ic/p-rad_sec"] = 0.0
        self.fdm["ic/q-rad_sec"] = 0.0
        self.fdm["ic/r-rad_sec"] = 0.0

        # FUEL: make sure we actually have fuel (your log showed 0%)
        # Try several approaches depending on what the model supports.
        set_ok = False
        for prop, val in [
            ("ic/fuel-fraction", 0.5),            # 50% fuel if supported
            ("ic/fuel-qty-lbs", 500.0),           # or set an absolute qty
        ]:
            try:
                self.fdm[prop] = val
                set_ok = True
            except Exception:
                pass
        # Fallback: populate the first two tanks if present
        if not set_ok:
            for tank_idx, qty in [(0, 400.0), (1, 400.0)]:
                try:
                    self.fdm[f"propulsion/tank[{tank_idx}]/contents-lbs"] = qty
                    set_ok = True
                except Exception:
                    pass

        # Engine OFF and controls neutral
        self._engine_off()
        self._neutral_controls()

        # Helicopter-specific: kill AFCS/heading hold at init so it won't fight us
        self._force_afcs_manual(False)

        # Park brake ON until we press J (does nothing for skids, but harmless)
        self._apply_all_brakes(1.0)

    # ---------------- Systems helpers ----------------
    def _apply_all_brakes(self, v: float):
        v = max(0.0, min(1.0, float(v)))
        for name in [
            "fcs/left-brake-cmd",
            "fcs/right-brake-cmd",
            "fcs/brake-cmd-norm",
            "gear/parking-brake",
        ]:
            try:
                self.fdm[name] = v
            except Exception:
                pass

    def _engine_off(self):
        props_int0 = [
            "propulsion/starter_cmd",
            "propulsion/cutoff_cmd",
            "propulsion/primer_cmd",
            "propulsion/ignition_cmd"
        ]
        for p in props_int0:
            try: self.fdm[p] = 0
            except Exception: pass
        props_float0 = [
            "fcs/mixture-cmd-norm[0]",
            "fcs/throttle-cmd-norm[0]",
            "fcs/prop-cmd-norm[0]"
        ]
        for p in props_float0:
            try: self.fdm[p] = 0.0
            except Exception: pass
        try: self.fdm["propulsion/engine[0]/set-running"] = 0
        except Exception: pass
        self.engine_running = False

    def _neutral_controls(self):
        for p in ["fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm", "fcs/rudder-cmd-norm"]:
            try: self.fdm[p] = 0.0
            except Exception: pass
        for p in ["fcs/collective-cmd-norm", "fcs/roll-cmd-norm", "fcs/pitch-cmd-norm", "fcs/yaw-cmd-norm"]:
            try: self.fdm[p] = 0.0
            except Exception: pass

    def _force_afcs_manual(self, enable: bool):
        """enable=False turns AFCS off; True would turn it on."""
        val = 1.0 if enable else 0.0
        for p in [
            "ap/afcs/roll-channel-active-norm",
            "ap/afcs/pitch-channel-active-norm",
            "ap/afcs/yaw-channel-active-norm",
            "ap/afcs/altitude-channel-active-norm",
        ]:
            try: self.fdm[p] = val
            except Exception: pass
        # Heading hold off by default
        try: self.fdm["ap/afcs/heading-hold-enable"] = 0.0 if not enable else 1.0
        except Exception: pass
        # Set trims to zero
        for p in [
            "ap/afcs/phi-trim-rad",
            "ap/afcs/theta-trim-rad",
            "ap/afcs/psi-trim-rad",
            "ap/afcs/altitude-trim-ft",
        ]:
            try: self.fdm[p] = 0.0
            except Exception: pass

    def _configure_engine_for_start(self):
        # Fuel & ignition
        try: self.fdm["propulsion/cutoff_cmd"] = 0
        except Exception: pass
        try: self.fdm["fcs/mixture-cmd-norm[0]"] = 1.0
        except Exception: pass
        try: self.fdm["propulsion/ignition_cmd"] = 1
        except Exception: pass

        # Helicopter: turn governor ON and set nominal RPM if present
        for p, val in [
            ("fcs/rpm-governor-active-norm", 1.0),
            ("fcs/nominal-rpm", 324.0),
            ("propulsion/engine[0]/governor", 1),
        ]:
            try: self.fdm[p] = val
            except Exception: pass

        # Make sure AFCS stays MANUAL after start
        self._force_afcs_manual(False)

    def _start_engine_sequence(self):
        if self.engine_running:
            print("[INFO] Engine already running.")
            return

        print(f"[INFO] Starting engine on model '{self.model_name}' ...")
        # Clear all brakes on start
        self._apply_all_brakes(0.0)
        # Configure systems
        self._configure_engine_for_start()

        # Starter pulse
        try: self.fdm["propulsion/starter_cmd"] = 1
        except Exception: pass

        for _ in range(int(1.0 / self.sim_dt)):
            self.fdm.run()

        try: self.fdm["propulsion/starter_cmd"] = 0
        except Exception: pass

        # Force running if supported
        try: self.fdm["propulsion/engine[0]/set-running"] = 1
        except Exception: pass

        # Stabilize RPM/governor
        for _ in range(100):
            self.fdm.run()

        self.engine_running = True
        print("[SUCCESS] Engine started. AFCS OFF, governor ON, brakes released.")

    # --------------- Sim loop ---------------
    def step(self):
        if self.fdm and self.fdm.run():
            return self.get_state()
        return None

    def get_state(self):
        try:
            lat = float(self.fdm['position/lat-geod-deg'])
            lon = float(self.fdm['position/long-gc-deg'])
            alt_sl_ft = float(self.fdm['position/h-sl-ft'])

            lat_diff = lat - self.origin_lat
            lon_diff = lon - self.origin_lon
            alt_diff = alt_sl_ft - self.origin_alt

            m_per_deg_lat = 111320.0
            m_per_deg_lon = 111320.0 * math.cos(math.radians(lat))
            m_per_ft = 0.3048

            unity_x = lon_diff * m_per_deg_lon
            unity_y = alt_diff * m_per_ft
            unity_z = lat_diff * m_per_deg_lat

            return {
                'position': {'lat':lat,'lon':lon,'alt':alt_sl_ft,
                             'unity_x':unity_x,'unity_y':unity_y,'unity_z':unity_z},
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
                },
                'meta': {
                    'model': self.model_name,
                    'rotorcraft': self.is_rotorcraft
                }
            }
        except Exception as e:
            print(f"[ERROR] get_state: {e}")
            return None

    def set_controls(self, throttle=0, roll=0, pitch=0, yaw=0):
        throttle = max(0.0, min(1.0, float(throttle)))
        roll     = max(-1.0, min(1.0, float(roll)))
        pitch    = max(-1.0, min(1.0, float(pitch)))
        yaw      = max(-1.0, min(1.0, float(yaw)))

        if self.is_rotorcraft:
            # Drone/heli mapping
            for p, val in [
                ("fcs/collective-cmd-norm", throttle),  # vertical
                ("fcs/roll-cmd-norm", roll),            # lateral cyclic
                ("fcs/pitch-cmd-norm", pitch),          # longitudinal cyclic
                ("fcs/yaw-cmd-norm", yaw),              # anti-torque
            ]:
                try: self.fdm[p] = val
                except Exception: pass
        else:
            # Fixed wing fallback
            for p, val in [
                ("fcs/throttle-cmd-norm[0]", throttle),
                ("fcs/aileron-cmd-norm", roll),
                ("fcs/elevator-cmd-norm", pitch),
                ("fcs/rudder-cmd-norm", yaw)
            ]:
                try: self.fdm[p] = val
                except Exception: pass

    def run_server(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', self.port))
            sock.settimeout(0.001)

            print(f"[SUCCESS] Bridge listening 127.0.0.1:{self.port}")
            print(f"[INFO] Sending to 127.0.0.1:{self.port+1}")
            print(f"[INFO] Press J in Unity to start engine (AFCS OFF, governor ON).")
            print(f"[INFO] Loaded model: {self.model_name} (rotorcraft={self.is_rotorcraft})")

            self.running = True
            next_tick = time.perf_counter()

            while self.running:
                # receive controls
                try:
                    data, _ = sock.recvfrom(2048)
                    msg = json.loads(data.decode())

                    if msg.get('engine_start', False):
                        self._start_engine_sequence()

                    self.set_controls(
                        throttle=msg.get('throttle', 0.0),
                        roll=msg.get('aileron', 0.0),
                        pitch=msg.get('elevator', 0.0),
                        yaw=msg.get('rudder', 0.0)
                    )
                except socket.timeout:
                    pass
                except Exception as e:
                    print(f"[WARN] recv: {e}")

                # fixed-rate tick
                now = time.perf_counter()
                if now >= next_tick:
                    state = self.step()
                    next_tick += self.sim_dt
                    if state:
                        try:
                            sock.sendto(json.dumps(state).encode(), ('127.0.0.1', self.port + 1))
                        except Exception as e:
                            print(f"[WARN] send: {e}")

            sock.close()
            print("[INFO] Server stopped.")
        except Exception as e:
            print(f"[ERROR] server: {e}")
            traceback.print_exc()

if __name__ == '__main__':
    print("[DEBUG] Python Executable:", sys.executable)
    print("[DEBUG] Python Version:", sys.version)
    bridge = JSBSimBridge()
    if bridge.initialize():
        try:
            bridge.run_server()
        except KeyboardInterrupt:
            bridge.running = False
            print("[INFO] Shutting down...")
    else:
        sys.exit(1)
