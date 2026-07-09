from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Optional


class ReplayLogger:
    def __init__(
        self,
        run_dir: Path,
        frame_queue_size: int = 300,
        save_every_n_frames: int = 1,
    ):
        self.run_dir = Path(run_dir)
        self.frames_dir = self.run_dir / "frames"
        self.save_every_n_frames = max(1, save_every_n_frames)

        self.frames_seen = 0
        self.frames_enqueued = 0
        self.frames_dropped = 0
        self.frames_written = 0

        self._closed = False
        self._lock = threading.Lock()
        self._frame_queue = queue.Queue(maxsize=frame_queue_size)

        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self._actions_file = (self.run_dir / "actions.jsonl").open("a", encoding="utf-8")
        self._events_file = (self.run_dir / "events.jsonl").open("a", encoding="utf-8")

        self._writer_thread = threading.Thread(
            target=self._frame_writer_loop,
            name="ReplayFrameWriter",
            daemon=True,
        )
        self._writer_thread.start()
        self.log_event("replay_logger_started", {"run_dir": str(self.run_dir)})

    def log_frame_jpeg(
        self,
        frame_id: int,
        jpeg_bytes: bytes,
        sim_time_ns: Optional[int] = None,
    ) -> None:
        with self._lock:
            if self._closed:
                return
            self.frames_seen += 1
            frames_seen = self.frames_seen

        if frames_seen % self.save_every_n_frames != 0:
            return

        frame_path = self.frames_dir / f"{frame_id:08d}.jpg"
        item = (frame_path, bytes(jpeg_bytes), sim_time_ns)

        try:
            self._frame_queue.put_nowait(item)
        except queue.Full:
            with self._lock:
                self.frames_dropped += 1
            return

        with self._lock:
            self.frames_enqueued += 1

    def log_action(
        self,
        phase: str,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        thrust: float,
        extra: Optional[dict] = None,
    ) -> None:
        record = {
            "time": time.time(),
            "phase": phase,
            "roll_rate": roll_rate,
            "pitch_rate": pitch_rate,
            "yaw_rate": yaw_rate,
            "thrust": thrust,
        }
        if extra is not None:
            record["extra"] = extra
        self._write_jsonl(self._actions_file, record)

    def log_event(self, event_type: str, data: Optional[dict] = None) -> None:
        record = {
            "time": time.time(),
            "event_type": event_type,
            "data": data or {},
        }
        self._write_jsonl(self._events_file, record)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True

        self.log_event("replay_logger_closed", self._counters())

        try:
            self._frame_queue.put(None, timeout=1.0)
        except queue.Full:
            pass

        self._writer_thread.join(timeout=2.0)

        with self._lock:
            self._actions_file.close()
            self._events_file.close()

    def _frame_writer_loop(self) -> None:
        while True:
            item = self._frame_queue.get()
            try:
                if item is None:
                    return
                frame_path, jpeg_bytes, _sim_time_ns = item
                with frame_path.open("wb") as frame_file:
                    frame_file.write(jpeg_bytes)
                with self._lock:
                    self.frames_written += 1
            finally:
                self._frame_queue.task_done()

    def _write_jsonl(self, jsonl_file, record: dict) -> None:
        with self._lock:
            if self._closed and record.get("event_type") != "replay_logger_closed":
                return
            jsonl_file.write(json.dumps(record, separators=(",", ":")) + "\n")
            jsonl_file.flush()

    def _counters(self) -> dict:
        with self._lock:
            return {
                "frames_seen": self.frames_seen,
                "frames_enqueued": self.frames_enqueued,
                "frames_dropped": self.frames_dropped,
                "frames_written": self.frames_written,
                "queued_frames": self._frame_queue.qsize(),
            }
