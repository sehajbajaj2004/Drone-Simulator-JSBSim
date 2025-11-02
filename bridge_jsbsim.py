# NEW CODE




#!/usr/bin/env python3
import json
import logging
import socket
import time
import select
import os
from typing import Tuple

try:
    import jsbsim
except ImportError as e:
    raise SystemExit("JSBSim not found. Install with: pip install jsbsim") from e


# -----------------------
# Config
# -----------------------
UDP_LISTEN_IP = "127.0.0.1"
UDP_LISTEN_PORT = 5555     # Unity -> Python (controls)
UDP_SEND_PORT = 5556       # Python -> Unity (telemetry)
AIRCRAFT = "c172p"         # default stock JSBSim aircraft
DT = 0.01                  # 100 Hz sim step

# -----------------------
# Logging
# -----------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("JSBSimBridge")


def _guess_jsbsim_root() -> str:
    """
    Try to locate JSBSim's data root automatically from the installed package.
    Falls back to env var JSBSIM_ROOT if needed.
    """
    # 1) Environment variable
    env_root = os.environ.get("JSBSIM_ROOT")
    if env_root and os.path.isdir(env_root):
        return env_root

    # 2) Package data path (commonly site-packages/jsbsim/data)
    pkg_data = os.path.join(os.path.dirname(jsbsim.__file__), "data")
    if os.path.isdir(pkg_data):
        return pkg_data

    raise RuntimeError(
        "Could not locate JSBSim data root. "
        "Set JSBSIM_ROOT env var to the JSBSim 'data' folder."
    )


class JSBSimBridge:
    def __init__(self,
                 listen_ip: str = UDP_LISTEN_IP,
                 listen_port: int = UDP_LISTEN_PORT,
                 send_port: int = UDP_SEND_PORT,
                 aircraft: str = AIRCRAFT,
                 dt: float = DT):
        self.listen_addr: Tuple[str, int] = (listen_ip, listen_port)
        self.send_port = send_port
        self.aircraft = aircraft
        self.dt = dt

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.listen_addr)
        self.sock.setblocking(False)

        self.client_addr = None  # will be set after first packet
        self.seq_last_rx = -1
        self.fdm = None
        self.sim_time = 0.0

    # ------------ JSBSim init ------------
    def init_jsbsim(self):
        root = _guess_jsbsim_root()
        log.info(f"Using JSBSim root: {root}")
        self.fdm = jsbsim.FGFDMExec(root_dir=root)
        self.fdm.set_airplane(self.aircraft)
        log.info(f"Loaded aircraft: {self.aircraft}")

        # Example initial conditions (slightly above ground, 0 KIAS, facing north)
        self.fdm.set_property_value("ic/h-sl-ft", 50.0)
        self.fdm.set_property_value("ic/psi-true-deg", 0.0)
        self.fdm.set_property_value("ic/lat-gc-deg", 37.618805)  # near KSFO for fun
        self.fdm.set_property_value("ic/long-gc-deg", -122.375416)

        # Initialize
        self.fdm.run_ic()

        # Try to ensure engine is running (if supported by aircraft)
        # Not all aircraft share the exact same property names; log and continue if not found.
        for prop in [
            "propulsion/engine[0]/set-running",
            "propulsion/engine/set-running"  # fallback
        ]:
            try:
                self.fdm[ prop ] = 1
                log.info(f"Set engine running via '{prop}'")
                break
            except Exception:
                pass

        # Full mixture and some throttle by default
        for prop, val in [
            ("propulsion/mixture-cmd-norm", 1.0),
            ("propulsion/throttle-cmd-norm", 0.1)
        ]:
            try:
                self.fdm[prop] = val
            except Exception:
                pass

        log.info("JSBSim initialized and ready.")

    # ------------ Networking helpers ------------
    def _recv_controls(self):
        """Non-blocking receive. Returns (addr, data_dict) or (None, None)."""
        ready, _, _ = select.select([self.sock], [], [], 0.0)
        if not ready:
            return None, None

        data, addr = self.sock.recvfrom(8192)
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception as e:
            log.warning(f"Invalid JSON from {addr}: {e}")
            return None, None

        return addr, msg

    def _send_telemetry(self, addr, msg: dict):
        payload = json.dumps(msg).encode("utf-8")
        self.sock.sendto(payload, (addr[0], self.send_port))

    # ------------ Main loop ------------
    def run(self):
        self.init_jsbsim()
        log.info(f"UDP listening on {self.listen_addr}. Telemetry will be sent to port {self.send_port}.")
        log.info("Awaiting first control packet from Unity...")

        last_log = time.time()
        steps = 0

        while True:
            t0 = time.perf_counter()

            # ---- Receive controls (if any) ----
            addr, controls = self._recv_controls()

            if addr and controls:
                # Remember client to send telemetry to
                self.client_addr = addr
                self.seq_last_rx = int(controls.get("seq", -1))

                # Map controls → JSBSim
                thr = float(controls.get("throttle", 0.0))
                ail = float(controls.get("aileron", 0.0))
                ele = float(controls.get("elevator", 0.0))
                rud = float(controls.get("rudder", 0.0))

                # Write to properties (standard normalized command properties)
                self.fdm["propulsion/throttle-cmd-norm"] = max(0.0, min(1.0, thr))
                self.fdm["fcs/aileron-cmd-norm"] = max(-1.0, min(1.0, ail))
                self.fdm["fcs/elevator-cmd-norm"] = max(-1.0, min(1.0, ele))
                self.fdm["fcs/rudder-cmd-norm"] = max(-1.0, min(1.0, rud))

                log.info(
                    f"[RX] seq={self.seq_last_rx} "
                    f"thr={thr:+.2f} ail={ail:+.2f} ele={ele:+.2f} rud={rud:+.2f}"
                )

            # ---- Step sim ----
            self.fdm.set_dt(self.dt)
            self.fdm.run()
            self.sim_time = self.fdm.get_sim_time_s()
            steps += 1

            # ---- Send telemetry (if we have a client) ----
            if self.client_addr:
                telemetry = {
                    "seq": self.seq_last_rx,            # echo last received seq
                    "sim_time": self.sim_time,
                    "lat_deg": self.fdm["position/lat-gc-deg"],
                    "lon_deg": self.fdm["position/long-gc-deg"],
                    "alt_m": self.fdm["position/h-sl-meters"],
                    "agl_m": self.fdm["position/h-agl-meters"],
                    "roll_deg": self.fdm["attitude/roll-deg"],
                    "pitch_deg": self.fdm["attitude/pitch-deg"],
                    "heading_deg": self.fdm["attitude/heading-deg"],
                    "p_rad_s": self.fdm["rates/p-rad_sec"],
                    "q_rad_s": self.fdm["rates/q-rad_sec"],
                    "r_rad_s": self.fdm["rates/r-rad_sec"],
                    "vt_fps": self.fdm["velocities/vt-fps"],
                    "vc_kts": self.fdm["velocities/vc-kts"],
                }
                self._send_telemetry(self.client_addr, telemetry)

                log.info(
                    f"[TX] seq={telemetry['seq']} "
                    f"t={telemetry['sim_time']:.2f}s pos=({telemetry['lat_deg']:.6f},"
                    f"{telemetry['lon_deg']:.6f}) alt={telemetry['alt_m']:.1f}m "
                    f"att(R/P/H)=({telemetry['roll_deg']:.1f}/{telemetry['pitch_deg']:.1f}/"
                    f"{telemetry['heading_deg']:.1f}) vc={telemetry['vc_kts']:.1f}kt"
                )

            # ---- Periodic heartbeat ----
            now = time.time()
            if now - last_log > 5.0 and not self.client_addr:
                log.info("No client yet… still waiting for controls on UDP.")
                last_log = now

            # ---- Sleep to hold real-time DT ----
            elapsed = time.perf_counter() - t0
            to_sleep = max(0.0, self.dt - elapsed)
            time.sleep(to_sleep)


if __name__ == "__main__":
    log.info("Starting JSBSim Python Bridge…")
    try:
        JSBSimBridge().run()
    except KeyboardInterrupt:
        log.info("Shutting down gracefully.")
