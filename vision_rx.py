import socket
import struct
import threading
import time

import cv2
import numpy as np

# Modify these properties if you want to run the server remotely for example
SIM_SERVER_UDP_IP = "0.0.0.0"
SIM_SERVER_UDP_PORT = 5600

# Camera intrinsics per spec 3.8 - used to define image center for pixel-offset error.
IMG_CENTER_X, IMG_CENTER_Y = 320.0, 180.0

# Gate colour is red, which wraps around the HSV hue circle (0-180 in OpenCV) - two ranges
# are combined to cover both ends of the wrap.
LOWER_RED_1 = np.array([0, 120, 50])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 50])
UPPER_RED_2 = np.array([180, 255, 255])

# Minimum contour area (px^2) to be considered a real gate detection, not noise.
MIN_CONTOUR_AREA = 800

# Once the red frame is clipped by the image edge, its bounding-box center is no longer a
# reliable bearing to a gate. This happens right as gate 1 passes overhead; using that sliver
# as a fresh target can steer the drone away from the next gate.
EDGE_CLIP_MARGIN_PX = 2

class VisionRX:

    def __init__(self, data):
        self.data = data
        self.shared_data = data
        self.thread = threading.Thread(
            target=self._vision_loop,
            daemon=False
        )
        self.is_running = True
        self._last_diag_print = 0.0
        self.thread.start()

    def get_thread_for_join(self):
        self.is_running = False
        return self.thread

    def _vision_loop(self):
        header_format = "<IHHIIQ"
        header_sz = struct.calcsize(header_format)
        frames = {}  # frame_id -> received associated frame data

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((SIM_SERVER_UDP_IP, SIM_SERVER_UDP_PORT))
        sock.settimeout(1.0)
        print("Listening for camera frames...")

        while self.is_running:
            try:
                packet, addr = sock.recvfrom(65536)  # max UDP size
            except socket.timeout:
                continue

            header = packet[:header_sz]
            payload = packet[header_sz:]

            # frame_id - identifier for this vision frame
            # chunk_id - identifier for this chunk packet of data of this frame
            # total_chunks - total number of chunk packets that make up this frame
            # jpeg_size - full size of jpeg data
            # payload_size - size of this packet
            # sim_time_ns - frame's epoch timestamp in ns on the server
            frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = struct.unpack(header_format, header)

            if frame_id not in frames:
                frames[frame_id] = {
                    "chunks": {},
                    "total": total_chunks,
                    "size": jpeg_size,
                    "time": sim_time_ns
                }

            frames[frame_id]["chunks"][chunk_id] = payload

            # Check if frame is complete
            if len(frames[frame_id]["chunks"]) == total_chunks:
                jpeg_bytes = bytearray()

                frame_complete = True
                for i in range(total_chunks):
                    if i not in frames[frame_id]["chunks"]:
                        print('Missing packet %s in frame %s' % (i, frame_id,))
                        frame_complete = False
                        continue
                    jpeg_bytes.extend(frames[frame_id]["chunks"][i])

                if not frame_complete:
                    del frames[frame_id]
                    continue

                replay_logger = self.shared_data.get("replay_logger")
                if replay_logger is not None:
                    replay_logger.log_frame_jpeg(
                        frame_id=frame_id,
                        jpeg_bytes=jpeg_bytes,
                        sim_time_ns=sim_time_ns,
                    )

                img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if image is not None:
                    self.process_frame(frame_id, image)
                else:
                    print(f"Failed to decode frame: {frame_id}")

                del frames[frame_id]

    def process_frame(self, frame_id, img):
        """
        Detect the red gate frame in the FPV image and write its pixel offset from
        image center into shared_data['vision_gate_estimate'] (None if no gate seen).

        Stored convention:
          cx_offset > 0  ->  gate is to the RIGHT of image center
          cy_offset > 0  ->  gate is BELOW image center (image Y grows downward)
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

        # Diagnostic: distinguish "no red pixels at all" from "red seen but filtered out as
        # too small" - a prior test showed vision_gate_estimate staying None for entire
        # flights with no way to tell which of these was happening.
        now = time.time()
        if now - self._last_diag_print >= 0.5:
            self._last_diag_print = now
            red_pixel_count = int(cv2.countNonZero(mask))
            largest_raw_area = max((cv2.contourArea(c) for c in contours), default=0.0)
            print(
                f"VISION_DIAG: red_pixels={red_pixel_count} raw_contours={len(contours)} "
                f"largest_raw_area={largest_raw_area:.1f} (threshold={MIN_CONTOUR_AREA})",
                flush=True
            )

        if not valid:
            self.data['vision_gate_estimate'] = None
            return

        best = max(valid, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(best)
        img_h, img_w = img.shape[:2]

        if (
            bx <= EDGE_CLIP_MARGIN_PX
            or by <= EDGE_CLIP_MARGIN_PX
            or bx + bw >= img_w - EDGE_CLIP_MARGIN_PX
            or by + bh >= img_h - EDGE_CLIP_MARGIN_PX
        ):
            self.data['vision_gate_estimate'] = None
            return

        # Bounding-box center rather than contour centroid - the gate is a hollow frame,
        # so the centroid of its contour falls inside the empty middle anyway for a
        # rectangular outline, but the bounding box center is simpler and just as accurate.
        true_cx = bx + bw / 2.0
        true_cy = by + bh / 2.0

        self.data['vision_gate_estimate'] = {
            'cx_offset': true_cx - IMG_CENTER_X,
            'cy_offset': true_cy - IMG_CENTER_Y,
            'bbox_w': bw,
            'bbox_h': bh,
            'area': cv2.contourArea(best),
            'frame_id': frame_id,
        }
