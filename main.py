#
# Sample Python client for the AI GP controller
#

import time
from datetime import datetime
from pathlib import Path

from replay_logger import ReplayLogger
from setup import setup_components

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "127.0.0.1"
SIM_SERVER_UDP_PORT = 14550

# time since sim started ms
system_boot_ms = int(time.time() * 1000)

# arbitrary shared data between the various components
shared_data = {}

run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
run_dir = Path("runs") / run_id
replay_logger = ReplayLogger(
    run_dir=run_dir,
    save_every_n_frames=1,
)
shared_data["replay_logger"] = replay_logger
shared_data["run_dir"] = run_dir
print(f"[REPLAY] Logging run to: {run_dir}")

# setup components
components = setup_components(shared_data, system_boot_ms, SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT)
controller = components['controller']
ts_loop = components['ts_loop']
mavlink_rx = components['mavlink_rx']
vision_rx = components['vision_rx']

# Controller.update() handles arming (and re-arming) internally based on observed armed
# state, since a single arm() call doesn't reliably stick in this sim.
print("Starting control loop...", flush=True)
is_running = True
try:
    while is_running:
        controller.update()
except KeyboardInterrupt:
    print("[MAIN] KeyboardInterrupt received, shutting down.")
finally:
    # exit
    ts_loop.get_thread_for_join().join(timeout=1.0)
    mavlink_rx.get_thread_for_join().join(timeout=1.0)
    vision_rx.get_thread_for_join().join(timeout=1.0)
    replay_logger.close()
    print(f"[REPLAY] Closed logger. Run folder: {run_dir}")

    print("Client exited!", flush=True)
