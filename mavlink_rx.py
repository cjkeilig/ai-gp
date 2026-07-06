import math
import struct
import time
import threading

from pymavlink import mavutil

ENCAPSULATED_RACE_STATUS_MSG_ID = 1
ENCAPSULATED_TRACK_INFO_MSG_ID  = 2

class MAVLinkRX:

    def __init__(self, mavlink_connection, data):
        self.mavlink_conn = mavlink_connection
        self.data = data
        self.thread = None
        self.is_running = False

        self.track_chunks = {}
        self.expected_num_track_chunks = {}
        self._armed_known = None
        self._last_heartbeat_src = None
        self._last_race_status_print = 0.0
        self._last_actuator_print = 0.0
        self._last_imu_print = 0.0
        self.DIAG_PRINT_INTERVAL_S = 0.5
        self._unhandled_types_seen = set()
        self._last_heartbeat_time = None
        self._last_armed_change_time = None

    @classmethod
    def create_mavlink_rx(cls, mavlink_connection, data):
        rx = cls(mavlink_connection, data)
        rx.thread = threading.Thread(
            target=rx.mavlink_receive_loop,
            daemon = False
        )
        rx.is_running = True
        rx.thread.start()
        return rx

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def mavlink_receive_loop(self):
        """
        Continuously receive MAVLink messages without blocking.
        """
        while self.is_running:

            try:
                msg = self.mavlink_conn.recv_match(blocking=False)
            except ConnectionResetError:
                print('WARNING: ConnectionResetError was thrown. No longer listening to MAVLink port.')
                return

            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()

            if msg_type == "BAD_DATA":
                continue

            # --------------------------------------------------------------------------------------
            # HEARTBEAT
            # --------------------------------------------------------------------------------------
            if msg_type == "HEARTBEAT":
                self.on_heartbeat(msg)

            # --------------------------------------------------------------------------------------
            # TIMESYNC
            # --------------------------------------------------------------------------------------
            elif msg_type == "TIMESYNC":
                self.on_timesync(msg)

            # --------------------------------------------------------------------------------------
            # ATTITUDE
            #
            #
            # PLEASE NOTE:
            # As per the configuration of the latest version of the simulator, Attitude telemetry has been disabled.
            #
            #
            # --------------------------------------------------------------------------------------
            elif msg_type == "ATTITUDE":
                self.on_attitude(msg)

            # --------------------------------------------------------------------------------------
            # LOCAL_POSITION_NED
            #
            #
            # PLEASE NOTE:
            # As per the configuration of the latest version of the simulator, Local Position NED telemetry has been disabled.
            #
            #
            # --------------------------------------------------------------------------------------
            elif msg_type == "LOCAL_POSITION_NED":
                self.on_local_position_ned(msg)

            # --------------------------------------------------------------------------------------
            # ODOMETRY
            #
            #
            # PLEASE NOTE:
            # As per the configuration of the latest version of the simulator, Odometry telemetry has been disabled.
            #
            #
            # --------------------------------------------------------------------------------------
            elif msg_type == "ODOMETRY":
                self.on_odometry(msg)

            # --------------------------------------------------------------------------------------
            # HIGHRES_IMU
            # --------------------------------------------------------------------------------------
            elif msg_type == "HIGHRES_IMU":
                self.on_highres_imu(msg)

            # --------------------------------------------------------------------------------------
            # ENCAPSULATED_DATA
            # --------------------------------------------------------------------------------------
            elif msg_type == "ENCAPSULATED_DATA":
                self.on_encapsulated_data(msg)

            # --------------------------------------------------------------------------------------
            # ACTUATOR_OUTPUT_STATUS
            # --------------------------------------------------------------------------------------
            elif msg_type == "ACTUATOR_OUTPUT_STATUS":
                self.on_actuator_output_status(msg)

            # --------------------------------------------------------------------------------------
            # COLLISION
            # --------------------------------------------------------------------------------------
            elif msg_type == "COLLISION":
                self.on_collision(msg)

            # --------------------------------------------------------------------------------------
            # STATUSTEXT - human-readable status/rejection reasons (e.g. failed arming checks)
            # --------------------------------------------------------------------------------------
            elif msg_type == "STATUSTEXT":
                self.on_statustext(msg)

            # --------------------------------------------------------------------------------------
            # COMMAND_ACK - never checked before now. Reveals whether MAV_CMD_* commands
            # (arm, set_mode, request_message, ...) are being accepted, denied, or unsupported.
            # --------------------------------------------------------------------------------------
            elif msg_type == "COMMAND_ACK":
                self.on_command_ack(msg)

            # --------------------------------------------------------------------------------------
            # DATA_TRANSMISSION_HANDSHAKE - Repurposed and used for upcoming 'Track Data' packets
            # --------------------------------------------------------------------------------------
            elif msg.get_type() == "DATA_TRANSMISSION_HANDSHAKE":
                track_data_transfer_id = msg.width
                self.track_chunks[track_data_transfer_id] = {}
                self.expected_num_track_chunks[track_data_transfer_id] = msg.packets

            # --------------------------------------------------------------------------------------
            # Catch-all - log any message type we don't explicitly handle, so nothing arrives
            # silently unnoticed (this is how AVAILABLE_MODES/CURRENT_MODE or anything else
            # unexpected would show up, if the sim ever sends them).
            # --------------------------------------------------------------------------------------
            else:
                self._log_unhandled(msg_type)

    def on_heartbeat(self, msg):
        now = time.time()
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.data['armed'] = armed
        src = (msg.get_srcSystem(), msg.get_srcComponent())

        heartbeat_dt = None if self._last_heartbeat_time is None else now - self._last_heartbeat_time
        self._last_heartbeat_time = now

        if armed != self._armed_known or src != getattr(self, "_last_heartbeat_src", None):
            held_for = None if self._last_armed_change_time is None else now - self._last_armed_change_time
            self._last_armed_change_time = now
            self._armed_known = armed
            self._last_heartbeat_src = src
            heartbeat_dt_str = "n/a" if heartbeat_dt is None else f"{heartbeat_dt:.3f}s"
            held_for_str = "n/a" if held_for is None else f"{held_for:.3f}s"
            print(
                f"Vehicle armed: {armed} "
                f"(src_system={src[0]}, src_component={src[1]}, "
                f"base_mode={msg.base_mode}, custom_mode={msg.custom_mode}, system_status={msg.system_status}, "
                f"heartbeat_dt={heartbeat_dt_str}, prev_state_held_for={held_for_str})",
                flush=True
            )

    def on_statustext(self, msg):
        print(f"STATUSTEXT [severity={msg.severity}]: {msg.text}", flush=True)

    def on_command_ack(self, msg):
        result_name = mavutil.mavlink.enums.get('MAV_RESULT', {}).get(msg.result)
        result_str = result_name.name if result_name else str(msg.result)
        print(
            f"COMMAND_ACK: command={msg.command} result={result_str} ({msg.result})",
            flush=True
        )

    def _log_unhandled(self, msg_type):
        if msg_type not in self._unhandled_types_seen:
            self._unhandled_types_seen.add(msg_type)
            print(f"UNHANDLED MESSAGE TYPE (first occurrence): {msg_type}", flush=True)

    def on_timesync(self, msg):
        request_time = msg.ts1
        response_time = msg.tc1

    def on_attitude(self, msg):
        # DIAGNOSTIC: template claims this is disabled - verify directly instead of trusting that.
        # It was arriving but never stored - the controller had no way to use it even though
        # this is real ground-truth attitude, not a gyro-integrated guess. Store it so
        # Controller can prefer it over dead-reckoning when it's actually live.
        self.data['attitude'] = {
            'roll': msg.roll, 'pitch': msg.pitch, 'yaw': msg.yaw,
            'time': time.time(),
        }
        print(
            f"ATTITUDE (was assumed disabled!): roll={msg.roll:.2f} pitch={msg.pitch:.2f} yaw={msg.yaw:.2f}",
            flush=True
        )

    def on_local_position_ned(self, msg):
        # DIAGNOSTIC: template claims this is disabled - verify directly instead of trusting that.
        print(
            f"LOCAL_POSITION_NED (was assumed disabled!): pos=({msg.x:.2f},{msg.y:.2f},{msg.z:.2f}) "
            f"vel=({msg.vx:.2f},{msg.vy:.2f},{msg.vz:.2f})",
            flush=True
        )

    def on_odometry(self, msg):
        # DIAGNOSTIC: template claims this is disabled - verify directly instead of trusting that.
        pos_x, pos_y, pos_z = msg.x, msg.y, msg.z
        qw, qx, qy, qz = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        vel_x, vel_y, vel_z = msg.vx, msg.vy, msg.vz

        # Convert to Euler (aerospace ZYX / NED convention) so Controller can use this as a
        # second possible source of real attitude ground-truth, in case ATTITUDE specifically
        # is disabled in this sim build but ODOMETRY isn't (or vice versa).
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        self.data['odometry'] = {
            'x': pos_x, 'y': pos_y, 'z': pos_z,
            'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
            'vx': vel_x, 'vy': vel_y, 'vz': vel_z,
            'rollspeed': msg.rollspeed, 'pitchspeed': msg.pitchspeed, 'yawspeed': msg.yawspeed,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'time': time.time(),
        }
        print(
            f"ODOMETRY (was assumed disabled!): pos=({pos_x:.2f},{pos_y:.2f},{pos_z:.2f}) "
            f"vel=({vel_x:.2f},{vel_y:.2f},{vel_z:.2f})",
            flush=True
        )

    def on_highres_imu(self, msg):
        acceleration_x, acceleration_y, acceleration_z = msg.xacc, msg.yacc, msg.zacc
        gyro_x, gyro_y, gyro_z = msg.xgyro, msg.ygyro, msg.zgyro
        time_boot_us = msg.time_usec

        # Body-frame FRD: x=roll axis rate, y=pitch axis rate, z=yaw axis rate - stored so the
        # controller can compare its own commanded rates against what's actually measured.
        self.data['gyro'] = {'roll': gyro_x, 'pitch': gyro_y, 'yaw': gyro_z}

        now = time.time()
        if now - self._last_imu_print >= self.DIAG_PRINT_INTERVAL_S:
            self._last_imu_print = now
            print(
                f"IMU: src=({msg.get_srcSystem()},{msg.get_srcComponent()}) "
                f"accel=({acceleration_x:.2f}, {acceleration_y:.2f}, {acceleration_z:.2f}) "
                f"gyro=({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f})",
                flush=True
            )

    def on_encapsulated_data(self, msg):
        if msg:
            raw_payload = bytes(msg.data)
            data_type = raw_payload[0]

            if int(data_type) == ENCAPSULATED_RACE_STATUS_MSG_ID:
                self.on_race_status(msg)
            elif int(data_type) == ENCAPSULATED_TRACK_INFO_MSG_ID:
                self.on_track_data_packet(msg)

    def on_race_status(self, msg):
        raw_payload = bytes(msg.data)
        # data_type - ID of this message
        # sim_boot_time_ms - elapsed ms on server since sim boot
        # race_start_boot_time_ms - elapsed ms on server since sim boot when race started. None or < 0 if race has not started
        # race_finish_time_ns - elapsed ns on server since sim boot when race finished. None or < 0 if race is ongoing
        # active_gate_index - current index of target race gate
        # last_gate_race_time - race time in seconds when last gate was passed
        data_type, sim_boot_time_ms, race_start_boot_time_ms, race_finish_time_ns, active_gate_index, last_gate_race_time = struct.unpack_from(
            "<BQqqIq", raw_payload)

        self.data['race_status'] = {
            'sim_boot_time_ms': sim_boot_time_ms,
            'race_start_boot_time_ms': race_start_boot_time_ms,
            'race_finish_time_ns': race_finish_time_ns,
            'active_gate_index': int(active_gate_index),
            'last_gate_race_time': last_gate_race_time,
        }

        now = time.time()
        if now - self._last_race_status_print >= self.DIAG_PRINT_INTERVAL_S:
            self._last_race_status_print = now
            print(
                f"RACE_STATUS: race_started={race_start_boot_time_ms >= 0} "
                f"(race_start_boot_time_ms={race_start_boot_time_ms}) "
                f"active_gate_index={active_gate_index} "
                f"race_finish_time_ns={race_finish_time_ns}",
                flush=True
            )

    def on_track_data_packet(self, msg):
        raw_payload = bytes(msg.data)
        # header:
        #   data_type - ID of this message
        #   transfer_id - ID of the group of packets this chunk belongs to
        data_type, transfer_id = struct.unpack_from("<BH", raw_payload)
        if transfer_id not in self.expected_num_track_chunks:
            return
        raw_payload = raw_payload[3:]
        self.track_chunks[transfer_id][msg.seqnr] = raw_payload
        if len(self.track_chunks[transfer_id]) == self.expected_num_track_chunks[transfer_id]:
            full_payload = bytes()
            for i in range(len(self.track_chunks[transfer_id])):
                full_payload = full_payload + self.track_chunks[transfer_id][i]
            del self.track_chunks[transfer_id]
            del self.expected_num_track_chunks[transfer_id]
            self.on_track_data(full_payload)

    def on_track_data(self, payload):
        #
        #
        # PLEASE NOTE:
        # As per the configuration of the latest version of the simulator, gate positions, orientations and dimensions are no longer published in telemetry and will be nulled.
        #
        #
        # header:
        #   num_gates - track gate count
        num_gates, = struct.unpack_from("<H", payload)
        payload = payload[2:]
        for i in range(num_gates):
            # Gate Info
            #   gate_id - range is 0 - num_gates
            #   position_ned_x, position_ned_y, position_ned_z - Position of gate in NED coordinates
            #   orientation_ned_w, orientation_ned_x, orientation_ned_y, orientation_ned_z - Orientation of gate in NED coordinates
            #   width - gate width in metres
            #   height - gate height in metres
            gate_id, position_ned_x, position_ned_y, position_ned_z, orientation_ned_w, orientation_ned_x, orientation_ned_y, orientation_ned_z, width, height = struct.unpack_from(
                "<Hfffffffff", payload)
            payload = payload[38:]

    def on_actuator_output_status(self, msg):
        time_boot_us = msg.time_usec
        motor_front_left = msg.actuator[0]
        motor_front_right = msg.actuator[1]
        motor_back_left = msg.actuator[2]
        motor_back_right = msg.actuator[3]

        now = time.time()
        if now - self._last_actuator_print >= self.DIAG_PRINT_INTERVAL_S:
            self._last_actuator_print = now
            print(
                f"ACTUATORS: src=({msg.get_srcSystem()},{msg.get_srcComponent()}) "
                f"FL={motor_front_left:.3f} FR={motor_front_right:.3f} "
                f"BL={motor_back_left:.3f} BR={motor_back_right:.3f}",
                flush=True
            )

    def on_collision(self, msg):
        # Collision IDs
        # 1001 - Gate
        # 1002 - Environment
        collision_id = msg.id

        threat_level = msg.threat_level # 1-2 with 2 being higher impact collision
        impact = msg.horizontal_minimum_delta # this is not a delta - it is the impulse magnitude in kg m/s

        # This was received and decoded but never surfaced anywhere - there was no direct way
        # to confirm what the drone hit (or how hard) versus inferring it from IMU accel spikes.
        target = {1001: 'GATE', 1002: 'ENVIRONMENT'}.get(collision_id, f'UNKNOWN({collision_id})')
        self.data['last_collision'] = {'target': target, 'threat_level': threat_level, 'impact': impact, 'time': time.time()}
        print(f"COLLISION: target={target} threat_level={threat_level} impact={impact:.2f}", flush=True)