from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def make_replay(run_dir: Path, fps: int = 30, output_name: str = "replay.mp4") -> Path:
    frames_dir = run_dir / "frames"

    if not frames_dir.exists():
        raise FileNotFoundError(f"Frames directory not found: {frames_dir}")

    frame_paths = sorted(frames_dir.glob("*.jpg"))

    if not frame_paths:
        raise FileNotFoundError(f"No .jpg frames found in: {frames_dir}")

    first = cv2.imread(str(frame_paths[0]))

    if first is None:
        raise RuntimeError(f"Could not read first frame: {frame_paths[0]}")

    height, width = first.shape[:2]
    output_path = run_dir / output_name

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )

    if not writer.isOpened():
        raise RuntimeError(f"Could not open VideoWriter for: {output_path}")

    written = 0

    for frame_path in frame_paths:
        frame = cv2.imread(str(frame_path))

        if frame is None:
            print(f"[WARN] Skipping unreadable frame: {frame_path}")
            continue

        frame_height, frame_width = frame.shape[:2]

        if frame_width != width or frame_height != height:
            frame = cv2.resize(frame, (width, height))

        writer.write(frame)
        written += 1

    writer.release()

    print(f"[REPLAY] Created: {output_path}")
    print(f"[REPLAY] Frames written: {written}")
    print(f"[REPLAY] FPS: {fps}")

    return output_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="Run folder containing frames/")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--output", default="replay.mp4")

    args = parser.parse_args()

    make_replay(
        run_dir=Path(args.run_dir),
        fps=args.fps,
        output_name=args.output,
    )


if __name__ == "__main__":
    main()
