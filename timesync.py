import time
import threading

from pymavlink import mavutil

TIMESYNC_REQUEST_HZ = 10

# Spec 5.2 lists "maintain heartbeat messages" as a client responsibility, and 4.4 sets a
# 2Hz minimum. Many MAVLink autopilots gate arming/offboard authority on detecting a live
# companion-computer heartbeat, separate from the vehicle's own - send ours alongside timesync.
def send_client_heartbeat(mavlink_conn):
    mavlink_conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,  # base_mode
        0,  # custom_mode
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

class TimeSync:

    def __init__(self, mavlink_connection, data):
        self.mavlink_conn = mavlink_connection
        self.data = data
        self.thread = None
        self.is_running = False

    @classmethod
    def create_timesync(cls, mavlink_connection, data):
        ts = cls(mavlink_connection, data)
        ts.thread = threading.Thread(
            target=ts.timesync_loop,
            daemon = False
        )
        ts.is_running = True
        ts.thread.start()
        return ts

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def timesync_loop(self):
        while self.is_running:
            now = int(time.time_ns())
            self.mavlink_conn.mav.timesync_send(
                now,  # tc1 = client time
                0     # ts1 = 0 (request)
            )
            send_client_heartbeat(self.mavlink_conn)
            time.sleep(1.0 / TIMESYNC_REQUEST_HZ)
