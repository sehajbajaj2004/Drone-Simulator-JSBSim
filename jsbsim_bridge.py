import jsbsim
import socket
import json
import sys
import time
import math
import os
import traceback
from datetime import datetime

class JSBSimBridge:
    def __init__(self, port=5555, prefer_rotorcraft=True, file_logging=False):
        self.port = port
        self.fdm = None
        self.running = False

        # Origin; treat origin_alt as terrain elevation (MSL)
        self.origin_lat = 37.4
        self.origin_lon = -122.4
        self.origin_alt = 100.0

        self.sim_dt = 0.01     # 100 Hz for smoother rotor control
        self.prefer_rotorcraft = prefer_rotorcraft
        self.model_name = None
        self.is_rotorcraft = False
        self.engine_running = False

        # optional python-side log file
        self.file_logging = file_logging
        self.log_path = None
        if self.file_logging:
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.log_path = os.path.abspath(f"JSBSimBridge_{ts}.log")

    # ---------------- Utilities ----------------
    def _log(self, msg):
        print(msg)
        if self.file_logging and self.log_path:
            try:
                with open(self.log_path, "a", encoding="utf-8") as f:
                    f.write(msg + "\n")
            except Exception:
                pass

    # ---------------- Initialization ----------------
    def initialize(self):
        try:
            self._log("[DEBUG] Initializing JSBSim...")
            root = jsbsim.get_default_root_dir()
            self._log(f"[DEBUG] JSBSim root: {root}")

            self.fdm = jsbsim.FGFDMExec(root)
            self.fdm.set_aircraft_path(os.path.join(root, "aircraft"))
            self.fdm.set_engine_path(os.path.join(root, "engine"))
            self.fdm.set_systems_path(os.path.join(root, "systems"))
            self.fdm.set_dt(self.sim_dt)

            # Prefer rotorcraft (include ah1s variants; add your quad model names here if installed)
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
                        self.is_rotorcraft = (ac in rotor_candidates) or ("ah1s" in ac.lower())
                        self._log(f"[SUCCESS] Loaded aircraft: {ac} (rotorcraft={self.is_rotorcraft})")
                        loaded = True
                        break
                except Exception as e:
                    self._log(f"[WARNING] load_model('{ac}') failed: {e}")

            if not loaded:
                self._log("[ERROR] No aircraft could be loaded.")
                return False

            self._set_initial_conditions_on_ground()

            self._log("[DEBUG] Running initial conditions...")
            if not self.fdm.run_ic():
                self._log("[ERROR] run_ic failed")
                return False

            # In case fuel didn't stick, try to ensure some fuel exists
            self._ensure_fuel_after_ic()

            # Settle integrators
            for _ in range(50):
                self.fdm.run()

            self._log("[SUCCESS] Initialized: on ground, engine OFF, zero velocity.")
            return True

        except Exception as e:
            self._log(f"[ERROR] initialize failed: {e}")
            traceback.print_exc()
            return False

    def _set_initial_conditions_on_ground(self):
        # Position; ensure both AGL ground and sea-level are consistent
        self.fdm["ic/lat-geod-deg"] = self.origin_lat
        self.fdm["ic/long-gc-deg"] = self.origin_lon
        self.fdm["ic/terrain-elevation-ft"] = self.origin_alt
        self.fdm["ic/h-agl-ft"] = 0.0
        # Also set sea-level height to match (some FDMs honor this instead of terrain)
        self.fdm["ic/h-sl-ft"] = self.origin_alt

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

        # Fuel attempts (some models ignore some variants)
        set_ok = False
        for prop, val in [("ic/fuel-fraction", 0.5), ("ic/fuel-qty-lbs", 500.0)]:
            try:
                self.fdm[prop] = val
                set_ok = True
            except Exception:
                pass
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

        # Kill AFCS/heading hold at init
        self._force_afcs_manual(False)

        # Park brake ON until J (harmless on skids; holds on wheels)
        self._apply_all_brakes(1.0)

    def _ensure_fuel_after_ic(self):
        # If contents still zero after run_ic, try again
        try:
            qty0 = self.fdm.get_property_value("propulsion/tank[0]/contents-lbs")
            qty1 = self.fdm.get_property_value("propulsion/tank[1]/contents-lbs")
            if (qty0 is None or qty0 <= 0.1) and (qty1 is None or qty1 <= 0.1):
                for tank_idx, qty in [(0, 400.0), (1, 400.0)]:
                    try:
                        self.fdm[f"propulsion/tank[{tank_idx}]/contents-lbs"] = qty
                    except Exception:
                        pass
                self._log("[INFO] Fuel topped up after IC (tanks appeared empty).")
        except Exception:
            pass

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
        for p in ["propulsion/starter_cmd", "propulsion/cutoff_cmd",
                  "propulsion/primer_cmd", "propulsion/ignition_cmd"]:
            try: self.fdm[p] = 0
            except Exception: pass
        for p in ["fcs/mixture-cmd-norm[0]", "fcs/throttle-cmd-norm[0]", "fcs/prop-cmd-norm[0]"]:
            try: self.fdm[p] = 0.0
            except Exception: pass
        try: self.fdm["propulsion/engine[0]/set-running"] = 0
        except Exception: pass
        self.engine_running = False

    def _neutral_controls(self):
        for p in ["fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm", "fcs/rudder-cmd-norm"]:
            try: self.fdm[p] = 0.0
            except Exception: pass
        for p in ["fcs/collective-cmd-norm", "fcs/rotor-collective-cmd-norm",
                  "fcs/roll-cmd-norm", "fcs/pitch-cmd-norm",
                  "fcs/yaw-cmd-norm", "fcs/anti-torque-cmd-norm"]:
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
            "ap/afcs/sas-active-norm",
            "ap/afcs/attitude-hold-active-norm",
        ]:
            try: self.fdm[p] = val
            except Exception: pass
        try: self.fdm["ap/afcs/heading-hold-enable"] = 1.0 if enable else 0.0
        except Exception: pass
        for p in ["ap/afcs/phi-trim-rad", "ap/afcs/theta-trim-rad",
                  "ap/afcs/psi-trim-rad", "ap/afcs/altitude-trim-ft"]:
            try: self.fdm[p] = 0.0
            except Exception: pass

    def _configure_engine_for_start(self):
        # Fuel & ignition
        for p, val in [("propulsion/cutoff_cmd", 0),
                       ("fcs/mixture-cmd-norm[0]", 1.0),
                       ("propulsion/ignition_cmd", 1)]:
            try: self.fdm[p] = val
            except Exception: pass

        # Helicopter: governor props and RPM if present
        for p, val in [
            ("fcs/rpm-governor-active-norm", 1.0),
            ("fcs/nominal-rpm", 324.0),
            ("propulsion/engine[0]/governor", 1),
        ]:
            try: self.fdm[p] = val
            except Exception: pass

        # Keep AFCS OFF
        self._force_afcs_manual(False)

    def _start_engine_sequence(self):
        if self.engine_running:
            self._log("[INFO] Engine already running.")
            return

        self._log(f"[INFO] Starting engine on model '{self.model_name}' ...")
        self._apply_all_brakes(0.0)
        self._configure_engine_for_start()

        try: self.fdm["propulsion/starter_cmd"] = 1
        except Exception: pass

        for _ in range(int(1.0 / self.sim_dt)):
            self.fdm.run()

        try: self.fdm["propulsion/starter_cmd"] = 0
        except Exception: pass

        try: self.fdm["propulsion/engine[0]/set-running"] = 1
        except Exception: pass

        for _ in range(100):
            self.fdm.run()

        self.engine_running = True
        self._log("[SUCCESS] Engine started. AFCS OFF, governor ON, brakes released.")

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
            self._log(f"[ERROR] get_state: {e}")
            return None

    def _set_if_exists(self, names, value):
        """Try several candidate properties until one sticks."""
        for n in names:
            try:
                self.fdm[n] = value
                return True
            except Exception:
                continue
        return False

    def set_controls(self, throttle=0, roll=0, pitch=0, yaw=0):
        throttle = max(0.0, min(1.0, float(throttle)))
        roll     = max(-1.0, min(1.0, float(roll)))
        pitch    = max(-1.0, min(1.0, float(pitch)))
        yaw      = max(-1.0, min(1.0, float(yaw)))

        if self.is_rotorcraft:
            # Try primary names; if not present, try fallbacks used by some heli models.
            self._set_if_exists(["fcs/collective-cmd-norm", "fcs/rotor-collective-cmd-norm"], throttle)
            self._set_if_exists(["fcs/roll-cmd-norm"], roll)               # lateral cyclic
            self._set_if_exists(["fcs/pitch-cmd-norm"], pitch)             # longitudinal cyclic
            self._set_if_exists(["fcs/yaw-cmd-norm", "fcs/anti-torque-cmd-norm"], yaw)
        else:
            self._set_if_exists(["fcs/throttle-cmd-norm[0]"], throttle)
            self._set_if_exists(["fcs/aileron-cmd-norm"], roll)
            self._set_if_exists(["fcs/elevator-cmd-norm"], pitch)
            self._set_if_exists(["fcs/rudder-cmd-norm"], yaw)

    def run_server(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', self.port))
            sock.settimeout(0.001)

            self._log(f"[SUCCESS] Bridge listening 127.0.0.1:{self.port}")
            self._log(f"[INFO] Sending to 127.0.0.1:{self.port+1}")
            self._log(f"[INFO] Press J in Unity to start engine (AFCS OFF, governor ON).")
            self._log(f"[INFO] Loaded model: {self.model_name} (rotorcraft={self.is_rotorcraft})")

            self.running = True
            next_tick = time.perf_counter()
            recv_ct = send_ct = 0
            last_status = time.time()

            while self.running:
                # receive controls
                try:
                    data, _ = sock.recvfrom(2048)
                    msg = json.loads(data.decode())
                    recv_ct += 1

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
                    self._log(f"[WARN] recv: {e}")

                # fixed-rate tick
                now = time.perf_counter()
                if now >= next_tick:
                    state = self.step()
                    next_tick += self.sim_dt
                    if state:
                        try:
                            sock.sendto(json.dumps(state).encode(), ('127.0.0.1', self.port + 1))
                            send_ct += 1
                        except Exception as e:
                            self._log(f"[WARN] send: {e}")

                if time.time() - last_status >= 5.0:
                    self._log(f"[STATUS] recv={recv_ct}, sent={send_ct}, model={self.model_name}, rotorcraft={self.is_rotorcraft}")
                    last_status = time.time()

            sock.close()
            self._log("[INFO] Server stopped.")
        except Exception as e:
            self._log(f"[ERROR] server: {e}")
            traceback.print_exc()

if __name__ == '__main__':
    print("[DEBUG] Python Executable:", sys.executable)
    print("[DEBUG] Python Version:", sys.version)
    bridge = JSBSimBridge(file_logging=False)  # set True to also save Python logs to a file
    if bridge.initialize():
        try:
            bridge.run_server()
        except KeyboardInterrupt:
            bridge.running = False
            print("[INFO] Shutting down...")
    else:
        sys.exit(1)
