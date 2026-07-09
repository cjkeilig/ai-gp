import time
import math

from pymavlink import mavutil

# --------------------------------------------------------------------------------------
# RESET COMMAND
MAVLINK_CMD_SIM_RESET = 31000

# --------------------------------------------------------------------------------------
# ATTITUDE CONTROLS
# --------------------------------------------------------------------------------------
# Per MAVLink common.xml ATTITUDE_TARGET_TYPEMASK: bit 128 = ATTITUDE_IGNORE (ignore the
# attitude quaternion, use body rates + thrust instead - no self-leveling reference). This
# is acro-style control. Bits 1/2/4 would ignore rates and use the quaternion instead
# (attitude-stabilize mode) - not what we want here.
ACRO_RATE_MASK = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE

def update_attitude_flight_control(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust):
    now_ms = int(time.time() * 1000)

    mavlink_conn.mav.set_attitude_target_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        ACRO_RATE_MASK,
        [1, 0, 0, 0],  # dummy quaternion (ignored by the typemask)
        roll_rate,
        pitch_rate,
        yaw_rate,
        thrust
    )

# --------------------------------------------------------------------------------------
# POSITION CONTROLS (alternative interface - not currently used by Controller)
# --------------------------------------------------------------------------------------
VELOCITY_POSITION_MASK = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

def update_position_flight_control(mavlink_conn, system_boot_ms, vx, vy, vz):
    now_ms = int(time.time() * 1000)
    mavlink_conn.mav.set_position_target_local_ned_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_POSITION_MASK,
        0.0, 0.0, 0.0,  # ignored position NED
        vx, vy, vz,     # commanded velocity NED [m/s]
        0.0, 0.0, 0.0,  # ignored acceleration
        0,              # ignored yaw
        0.0             # ignored yaw rate
    )

# --------------------------------------------------------------------------------------
# Control Loop
# --------------------------------------------------------------------------------------

# Spec caps command rate at <100Hz (drone-race-spec.txt 4.4) - stay comfortably under it.
CONTROL_HZ = 50

# A single arm() call doesn't stick - the vehicle needs to be re-armed until it holds
# (observed heartbeat flapping between armed/disarmed). Retry on an interval instead.
ARM_RETRY_S = 1.0

# custom_mode has stayed 0 in every observed heartbeat. Untested so far: explicitly
# requesting ArduCopter's numeric GUIDED mode (4) via MAV_CMD_DO_SET_MODE (COMMAND_LONG) -
# distinct from the raw SET_MODE message we tried earlier, which only set base_mode flags
# and left custom_mode at 0.
ARDUCOPTER_CUSTOM_MODE_GUIDED = 4

# Camera intrinsics per spec 3.8 (pinhole, no distortion, fx=fy=320px). Used to convert a
# pixel offset into a true bearing angle (atan(offset/f)) instead of an ad hoc linear
# px -> rad/s gain - this lets the yaw target be expressed (and held) as a real angle.
CAMERA_FX_PX = 320.0

HOVER_THRUST = 0.3
HOVER_DURATION_S = 2.0
CRUISE_THRUST = 0.27

# --------------------------------------------------------------------------------------
# ROOT CAUSE (previous design): ACRO/rate control (ATTITUDE_IGNORE) has no self-leveling -
# once a commanded rate is zero, the vehicle simply keeps whatever attitude it drifted to.
# The old controller pitched the nose down for a single 0.3s open-loop pulse at race start,
# then held pitch_rate=0.0 for the *entire rest of the flight*. Any disturbance afterward
# (drag, a graze off gate 1) permanently mistrims pitch with nothing to correct it - this
# explains both "no pitch correction before gate 1" (no closed-loop authority existed yet)
# and "loses the rest of the gates" after a gate-1 graze (pitch got knocked off and the fixed,
# 20deg-up-tilted camera never recovered a usable view).
#
# Roll/yaw had the same underlying flaw one level up: they were driven only by the
# *instantaneous* pixel offset, so as soon as a gate left frame the correction decayed
# (VISION_LOST_HOLD_S) back to flying level in whatever direction the nose happened to be
# pointed - discarding an in-progress turn exactly when the next gate was likely just outside
# the frame edge.
#
# FIX: hold all three axes as closed-loop *angles* against a running attitude estimate
# (gyro-integrated, corrected against real ATTITUDE/ODOMETRY telemetry when the sim actually
# sends it - see _update_attitude_estimate) instead of one-shot pulses / raw per-frame rates.
# A held target angle keeps being commanded - and keeps converging - through brief vision
# dropouts, and self-heals after a knock instead of drifting forever.
# --------------------------------------------------------------------------------------

# Pitch: hold a modest nose-down angle, but as a DUTY-CYCLED series of brief pulses, not a
# continuously-held nonzero rate. Logs from the continuously-held version showed pitch gyro
# climbing to and holding ~2.0-2.9 rad/s within ~100-200ms of any sustained nonzero
# pitch_rate command - a runaway forward flip ("head over heels"), consistently, even from
# the very first, gentle correction, well before any collision could explain it. Roll/yaw
# gyro stayed near zero in the same logs under the same continuous-hold treatment, so this is
# specific to sustaining a nonzero pitch rate, not attitude-hold control in general. The
# original (pre-rewrite) design only ever pitched for a single 0.3s pulse before returning to
# 0.0 forever - it likely avoided this exact failure mode by never sustaining the command.
# Reproduce that shape (short pulse, forced rest) but repeat it periodically so pitch can
# still be corrected over the course of the flight instead of being one-shot.
TARGET_CRUISE_PITCH_RAD = -0.14
K_P_PITCH_RATE_PER_RAD = 2.2
MAX_PITCH_RATE_RADS = 1.0
PITCH_DEADBAND_RAD = 0.05     # don't bother pulsing for errors smaller than this
PITCH_PULSE_ON_S = 0.18       # max continuous duration of a nonzero pitch-rate command
PITCH_PULSE_COOLDOWN_S = 0.95  # forced pitch_rate=0.0 for this long after a pulse ends

# Yaw: target heading is set once per *new* vision frame as
# (current estimated heading + bearing angle to the detected gate), then held via P(D)
# control against the running yaw estimate. Because the target is a persisted angle rather
# than a raw per-frame rate, the correction keeps being applied - and keeps converging - even
# after the gate drops out of frame again a moment later, instead of decaying back to "fly
# level" (this is the fix for "gets past gate 1 but doesn't correct enough to reach the rest
# of the gates").
K_P_YAW_RATE_PER_RAD = 1.8
K_D_YAW_RATE_PER_RATE = 0.5
MAX_YAW_RATE_RADS = 1.2

# Roll assists the same turn (translates the body sideways into it, since a forward-facing
# body-fixed camera re-centers slowly on yaw rotation alone) - driven off the *same* held
# heading error as yaw, so the two axes never disagree and both persist through dropouts.
K_P_ROLL_RATE_PER_RAD = 1.0
K_D_ROLL_RATE_PER_RATE = 0.3
MAX_ROLL_RATE_RADS = 1.0

# The largest, most violent roll/yaw corrections in logged flights all coincided with very
# large bbox areas (i.e. close range) - up close, a small real-world drift maps to a huge
# bearing swing, so a constant gain overreacts exactly when a gentle touch is needed. Scale
# gain down as the gate gets closer: full strength at/below this reference area, tapering off
# (floor kept high enough that close-range corrections stay meaningful) as area grows.
ROLL_YAW_FULL_GAIN_AREA_PX2 = 5000.0
MIN_ROLL_YAW_GAIN_SCALE = 0.4

# Altitude correction uses THRUST, not pitch: pitching the nose down mostly trades vertical
# thrust for forward speed (a weak, coupled way to descend) and the gate is often only
# visible for a couple of ticks before leaving frame - too little time for a rate-based
# pitch change to accumulate into a meaningful attitude/altitude change. Directly scaling
# collective thrust gives immediate, decoupled vertical authority instead.
# gate below center (cy_offset > 0) -> reduce thrust -> descend toward it. Do not add
# thrust when the gate is above center: near gate 1, partial/clipped detections can make the
# box center jump high in the image, and climbing on that ambiguity sends the drone over the
# gate instead of through it.
K_P_THRUST_PER_PX = 0.0008       # thrust adjustment per pixel of vertical offset
FAR_GATE_CLIMB_AREA_PX2 = 2500.0
#
# cy_offset alone was unreliable here: logs show it staying near-zero/negative even as the
# gate's bounding box grows to fill most of the frame (i.e. right before reaching it), so the
# cy_offset term was often still commanding a climb at exactly the moment a real descent was
# needed. bbox area is unambiguous regardless of that - bigger box always means closer - so
# use it to force a deliberate, monotonic descent bias as the gate looms larger, independent
# of whatever cy_offset is doing.
K_AREA_THRUST_DESCENT = 0.0000012  # thrust reduction per px^2 of bounding-box area
MIN_CRUISE_THRUST = 0.20
MAX_CRUISE_THRUST = 0.45

# A human pilot backs off the throttle before a gate that needs a hard correction - buying
# reaction time - then gets back on the power once lined up, rather than flying every
# approach at a fixed speed regardless of how much turning is needed. Ease cruise thrust down
# as the commanded turn rate grows, so sharp corrections get more time to complete instead of
# being outrun by a constant forward speed.
TURN_THRUST_EASE_START_RADS = 0.15   # below this commanded turn rate, no speed reduction
TURN_THRUST_EASE_FULL_RADS = 0.8     # at/above this, apply the full reduction
MAX_TURN_THRUST_EASE = 0.12          # max thrust taken off cruise during a hard turn

# Ground-truth attitude/odometry the spec claims is disabled, but mavlink_rx's diagnostics
# exist precisely because that wasn't confirmed - if a message actually arrived within this
# window, trust it over the gyro-integrated estimate (eliminates integration drift entirely).
ATTITUDE_TELEMETRY_MAX_AGE_S = 0.5

# Clamp the estimator's per-tick dt so a stall/GC pause can't inject a huge integration jump.
MAX_ESTIMATOR_DT_S = 0.1

# Anti-windup: bound the pitch-hold error itself (not just the final rate) before applying the
# P gain. Without this, once est_pitch diverges (e.g. during a real crash/tumble - see
# TUMBLE_GYRO_THRESHOLD_RADS below) the P term saturates and PINS pitch_rate at max-dive
# permanently, even long after conditions normalize, because the raw error never shrinks.
MAX_PITCH_ERROR_RAD = 0.6

# A measured gyro rate this large on any axis is not something our own commands can cause -
# our own ceiling is MAX_*_RATE_RADS (~1.0-1.2 rad/s). Seeing far more than that means the
# vehicle is in an uncontrolled physical event (a real collision/tumble), not a normal flight
# condition the angle-hold math above was designed for. Two reasons not to keep "correcting"
# through it: (1) the small-angle gyro-integration attitude estimate is meaningless at these
# rates, so any command computed from it is acting on garbage; (2) HIGHRES_IMU's gyro sign
# convention relative to SET_ATTITUDE_TARGET's rate convention was never confirmed to match
# (see historical note on K_P_ROLL_RATE_PER_PX in an earlier revision) - if it's flipped, the
# D-terms above are adding energy to the tumble instead of damping it. Back off instead of
# fighting blind.
TUMBLE_GYRO_THRESHOLD_RADS = 2.0
TUMBLE_RECOVERY_THRUST = 0.15


def _wrap_to_pi(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi

class Controller:
    def __init__(self, sim_conn, data, system_boot_ms):
        self.sim_conn = sim_conn
        self.data = data
        self.shared_data = data
        self.system_boot_ms = system_boot_ms
        self.flight_start_time = None
        self._last_arm_attempt = 0.0
        self._requested_mode_info = False
        self._wait_start_anchor_ms = None
        self._last_vision_log = 0.0
        self._prev_gate_frame_id = None
        self._last_tick_time = None
        self._est_roll = 0.0
        self._est_pitch = 0.0
        self._est_yaw = 0.0
        self._target_yaw = 0.0
        self._logged_real_attitude_source = False
        self._pitch_pulsing = False
        self._pitch_pulse_started_at = 0.0
        self._pitch_cooldown_until = 0.0
        self._last_action_phase = "waiting"

    def update(self):
        if not self._requested_mode_info:
            # Best-effort: ask the sim what flight modes it actually supports (MAVLink
            # "Standard Modes" service). Our pymavlink dialect can't decode the response
            # messages (too new), but the COMMAND_ACK alone tells us if this is implemented.
            self.request_available_modes()
            self._requested_mode_info = True

        armed = self.data.get('armed', False)

        if not armed:
            self.flight_start_time = None
            self._wait_start_anchor_ms = None
            # Reset estimator/derivative state too - don't integrate across a crash/re-arm
            # discontinuity from a previous flight attempt.
            self._prev_gate_frame_id = None
            self._last_tick_time = None
            self._est_roll = 0.0
            self._est_pitch = 0.0
            self._est_yaw = 0.0
            self._target_yaw = 0.0
            self._pitch_pulsing = False
            self._pitch_pulse_started_at = 0.0
            self._pitch_cooldown_until = 0.0
            # Stream a neutral setpoint while waiting to arm, in case the sim needs an
            # active command stream present before it grants control.
            self._log_action("waiting", 0.0, 0.0, 0.0, 0.0)
            update_attitude_flight_control(self.sim_conn, self.system_boot_ms, 0.0, 0.0, 0.0, 0.0)
            now = time.time()
            if now - self._last_arm_attempt >= ARM_RETRY_S:
                print('Not armed - requesting GUIDED mode and sending arm command...', flush=True)
                self.request_guided_mode()
                self.arm()
                self._last_arm_attempt = now
            time.sleep(1.0 / CONTROL_HZ)
            return

        # Keep the attitude estimate warm from the moment we're armed, so it isn't cold when
        # cruise control starts needing it.
        self._update_attitude_estimate()

        # Armed, but wait for a genuine (not stale-leftover) race start before applying any
        # real thrust/rates, to avoid an early-start disqualification (spec 7. Compliance).
        if self.flight_start_time is None:
            race_status = self.data.get('race_status')
            if race_status is None:
                self._log_action("waiting", 0.0, 0.0, 0.0, 0.0)
                update_attitude_flight_control(self.sim_conn, self.system_boot_ms, 0.0, 0.0, 0.0, 0.0)
                time.sleep(1.0 / CONTROL_HZ)
                return

            sim_ms = race_status['sim_boot_time_ms']
            start_ms = race_status['race_start_boot_time_ms']

            if self._wait_start_anchor_ms is None:
                self._wait_start_anchor_ms = sim_ms
                print(f'Waiting for race start (anchor sim_ms={sim_ms})...', flush=True)

            race_is_fresh = start_ms > 0 and start_ms >= self._wait_start_anchor_ms
            if not (race_is_fresh and sim_ms >= start_ms):
                self._log_action("waiting", 0.0, 0.0, 0.0, 0.0)
                update_attitude_flight_control(self.sim_conn, self.system_boot_ms, 0.0, 0.0, 0.0, 0.0)
                time.sleep(1.0 / CONTROL_HZ)
                return

            print('Race started - flying.', flush=True)
            self.flight_start_time = time.time()

        elapsed = time.time() - self.flight_start_time

        if elapsed < HOVER_DURATION_S:
            self._log_action("hover", 0.0, 0.0, 0.0, HOVER_THRUST)
            update_attitude_flight_control(self.sim_conn, self.system_boot_ms, 0.0, 0.0, 0.0, HOVER_THRUST)
        else:
            roll_rate, pitch_rate, yaw_rate, thrust = self._vision_steering_correction()
            self._log_action(self._last_action_phase, roll_rate, pitch_rate, yaw_rate, thrust)
            update_attitude_flight_control(self.sim_conn, self.system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust)

        time.sleep(1.0 / CONTROL_HZ)

    def _log_action(
        self,
        phase: str,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        thrust: float,
    ) -> None:
        replay_logger = self.shared_data.get("replay_logger") if hasattr(self, "shared_data") else None

        if replay_logger is None:
            return

        extra = {}
        gate = self.data.get('vision_gate_estimate')
        if gate is not None:
            extra["vision_gate"] = {
                "frame_id": gate.get("frame_id"),
                "cx_offset": gate.get("cx_offset"),
                "cy_offset": gate.get("cy_offset"),
                "bbox_w": gate.get("bbox_w"),
                "bbox_h": gate.get("bbox_h"),
                "area": gate.get("area"),
            }

        replay_logger.log_action(
            phase=phase,
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            thrust=thrust,
            extra=extra or None,
        )

    # -------------------------------
    # Maintain a running roll/pitch/yaw estimate by integrating HIGHRES_IMU gyro rates each
    # tick (a small-angle approximation adequate for the short windows between vision
    # corrections on this course - not a substitute for a real AHRS). If real ATTITUDE or
    # ODOMETRY telemetry is actually arriving (the spec claims it's disabled, but that was
    # never confirmed - see mavlink_rx diagnostics), snap to it instead: it's ground truth
    # and has no integration drift.
    # -------------------------------
    def _update_attitude_estimate(self):
        now = time.time()
        dt = 0.0 if self._last_tick_time is None else min(now - self._last_tick_time, MAX_ESTIMATOR_DT_S)
        self._last_tick_time = now

        real_attitude = self._freshest_real_attitude(now)
        if real_attitude is not None:
            if not self._logged_real_attitude_source:
                self._logged_real_attitude_source = True
                print(
                    f"ATTITUDE_EST: real '{real_attitude['source']}' telemetry is live - "
                    f"using it as ground truth instead of the gyro-integrated estimate",
                    flush=True
                )
            self._est_roll = real_attitude['roll']
            self._est_pitch = real_attitude['pitch']
            self._est_yaw = real_attitude['yaw']
            return

        gyro = self.data.get('gyro')
        if gyro is None or dt <= 0.0:
            return
        self._est_roll += gyro['roll'] * dt
        self._est_pitch += gyro['pitch'] * dt
        self._est_yaw += gyro['yaw'] * dt

    def _freshest_real_attitude(self, now):
        candidates = []
        attitude = self.data.get('attitude')
        if attitude is not None and now - attitude['time'] <= ATTITUDE_TELEMETRY_MAX_AGE_S:
            candidates.append(attitude)
        odometry = self.data.get('odometry')
        if odometry is not None and 'roll' in odometry and now - odometry['time'] <= ATTITUDE_TELEMETRY_MAX_AGE_S:
            candidates.append(odometry)
        if not candidates:
            return None
        freshest = max(candidates, key=lambda c: c['time'])
        source = 'ATTITUDE' if freshest is attitude else 'ODOMETRY'
        return {'source': source, 'roll': freshest['roll'], 'pitch': freshest['pitch'], 'yaw': freshest['yaw']}

    # -------------------------------
    # Duty-cycled pitch hold - see the note on TARGET_CRUISE_PITCH_RAD for why this is pulsed
    # rather than a continuously-held nonzero rate (empirically, sustaining one runs the
    # vehicle into a forward-flip). At most PITCH_PULSE_ON_S of nonzero pitch_rate at a time,
    # then a forced PITCH_PULSE_COOLDOWN_S at zero before another pulse can start.
    # -------------------------------
    def _pitch_pulse_rate(self):
        now = time.time()

        if now < self._pitch_cooldown_until:
            return 0.0

        if self._pitch_pulsing:
            if now - self._pitch_pulse_started_at >= PITCH_PULSE_ON_S:
                self._pitch_pulsing = False
                self._pitch_cooldown_until = now + PITCH_PULSE_COOLDOWN_S
                return 0.0
        else:
            pitch_error = TARGET_CRUISE_PITCH_RAD - self._est_pitch
            if abs(pitch_error) <= PITCH_DEADBAND_RAD:
                return 0.0
            self._pitch_pulsing = True
            self._pitch_pulse_started_at = now

        pitch_error = max(-MAX_PITCH_ERROR_RAD, min(MAX_PITCH_ERROR_RAD, TARGET_CRUISE_PITCH_RAD - self._est_pitch))
        return K_P_PITCH_RATE_PER_RAD * pitch_error

    # -------------------------------
    # Angle-hold controller: pitch holds a constant cruise angle; yaw holds a target heading
    # set from the detected gate's bearing (converted from its pixel offset via the pinhole
    # camera model) whenever a fresh detection arrives; roll assists the same held heading
    # error to coordinate the turn. Because all three are P(D) control against a persisted
    # target/estimate rather than a raw per-frame reaction, the correction keeps being
    # commanded - and keeps converging - through brief vision dropouts instead of decaying
    # back to neutral.
    # -------------------------------
    def _vision_steering_correction(self):
        gate = self.data.get('vision_gate_estimate')
        should_log = time.time() - self._last_vision_log >= 0.5
        gyro = self.data.get('gyro') or {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        if max(abs(gyro['roll']), abs(gyro['pitch']), abs(gyro['yaw'])) > TUMBLE_GYRO_THRESHOLD_RADS:
            self._last_action_phase = "tumble_recovery"
            # Reset the estimator too - a stale/diverged estimate would otherwise keep pinning
            # the pitch-hold P term at saturation even once the tumble passes, since nothing
            # would shrink the error back down on its own.
            self._est_roll = 0.0
            self._est_pitch = 0.0
            self._est_yaw = 0.0
            self._target_yaw = 0.0
            self._prev_gate_frame_id = None
            self._pitch_pulsing = False
            self._pitch_pulse_started_at = 0.0
            self._pitch_cooldown_until = time.time() + PITCH_PULSE_COOLDOWN_S
            if should_log:
                self._last_vision_log = time.time()
                print(
                    f"TUMBLE_GUARD: gyro={gyro} exceeds {TUMBLE_GYRO_THRESHOLD_RADS} rad/s on "
                    f"an axis - backing off instead of fighting it blind",
                    flush=True
                )
            return 0.0, 0.0, 0.0, TUMBLE_RECOVERY_THRUST

        self._last_action_phase = "vision"
        if gate is not None and gate['frame_id'] != self._prev_gate_frame_id:
            self._prev_gate_frame_id = gate['frame_id']
            # gate right of center (cx_offset > 0) -> positive bearing -> point the nose
            # further right (per euler_to_quat's "clockwise from above" yaw convention).
            bearing_rad = math.atan2(gate['cx_offset'], CAMERA_FX_PX)
            self._target_yaw = self._est_yaw + bearing_rad

        heading_error = _wrap_to_pi(self._target_yaw - self._est_yaw)

        if gate is not None:
            gain_scale = max(MIN_ROLL_YAW_GAIN_SCALE, min(1.0, ROLL_YAW_FULL_GAIN_AREA_PX2 / max(gate['area'], 1.0)))
        else:
            # No current detection to taper against - stay at full authority so a turn
            # already in progress keeps converging on the last known target heading instead
            # of being throttled back just because the gate isn't visible this instant.
            gain_scale = 1.0

        yaw_rate = gain_scale * (K_P_YAW_RATE_PER_RAD * heading_error - K_D_YAW_RATE_PER_RATE * gyro['yaw'])
        roll_rate = gain_scale * (K_P_ROLL_RATE_PER_RAD * heading_error - K_D_ROLL_RATE_PER_RATE * gyro['roll'])
        pitch_rate = self._pitch_pulse_rate()

        yaw_rate = max(-MAX_YAW_RATE_RADS, min(MAX_YAW_RATE_RADS, yaw_rate))
        roll_rate = max(-MAX_ROLL_RATE_RADS, min(MAX_ROLL_RATE_RADS, roll_rate))
        pitch_rate = max(-MAX_PITCH_RATE_RADS, min(MAX_PITCH_RATE_RADS, pitch_rate))

        # Ease off cruise speed as the commanded turn sharpens, same as a human pilot backing
        # off the throttle before a tight gate to buy time to complete the correction, then
        # getting back on the power once lined up.
        turn_mag = max(abs(roll_rate), abs(yaw_rate))
        ease_span = TURN_THRUST_EASE_FULL_RADS - TURN_THRUST_EASE_START_RADS
        turn_ease_scale = max(0.0, min(1.0, (turn_mag - TURN_THRUST_EASE_START_RADS) / ease_span))
        turn_thrust_ease = MAX_TURN_THRUST_EASE * turn_ease_scale

        if gate is not None:
            cy_for_thrust = gate['cy_offset']
            if gate['area'] >= FAR_GATE_CLIMB_AREA_PX2:
                cy_for_thrust = max(cy_for_thrust, 0.0)

            thrust = (
                CRUISE_THRUST
                - K_P_THRUST_PER_PX * cy_for_thrust
                - K_AREA_THRUST_DESCENT * gate['area']
                - turn_thrust_ease
            )
        else:
            thrust = CRUISE_THRUST - turn_thrust_ease
        thrust = max(MIN_CRUISE_THRUST, min(MAX_CRUISE_THRUST, thrust))

        # Single shared timer/print so all three lines always describe the same tick.
        if should_log:
            self._last_vision_log = time.time()
            print(f"VISION_GATE: {gate}", flush=True)
            print(
                f"ATTITUDE_EST: roll={self._est_roll:.3f} pitch={self._est_pitch:.3f} "
                f"yaw={self._est_yaw:.3f} target_yaw={self._target_yaw:.3f} "
                f"heading_error={heading_error:.3f}",
                flush=True
            )
            print(
                f"CONTROLLER_OUT: gain_scale={gain_scale:.3f} roll_rate={roll_rate:.3f} "
                f"pitch_rate={pitch_rate:.3f} yaw_rate={yaw_rate:.3f} thrust={thrust:.3f} "
                f"turn_ease={turn_thrust_ease:.3f}",
                flush=True
            )

        return roll_rate, pitch_rate, yaw_rate, thrust

    # -------------------------------
    # Ask the sim what flight modes it supports (MAVLink Standard Modes service)
    # -------------------------------
    def request_available_modes(self):
        AVAILABLE_MODES_MSG_ID = 435
        CURRENT_MODE_MSG_ID = 436
        for msg_id in (AVAILABLE_MODES_MSG_ID, CURRENT_MODE_MSG_ID):
            self.sim_conn.mav.command_long_send(
                self.sim_conn.target_system,
                self.sim_conn.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                msg_id,
                0, 0, 0, 0, 0, 0
            )

    # -------------------------------
    # Request ArduCopter's numeric GUIDED mode via COMMAND_LONG
    # -------------------------------
    def request_guided_mode(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            ARDUCOPTER_CUSTOM_MODE_GUIDED,
            0, 0, 0, 0, 0
        )

    # -------------------------------
    # Arm the drone
    # -------------------------------
    def arm(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

    # -------------------------------
    # Reset sim
    # -------------------------------
    def send_sim_reset_command(self):
        self.sim_conn.mav.command_long_send(
            self.sim_conn.target_system,
            self.sim_conn.target_component,
            MAVLINK_CMD_SIM_RESET,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0
        )
