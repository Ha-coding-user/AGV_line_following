import argparse
import csv
import json
import math
import queue
import threading
import time
import uuid
from pathlib import Path

class DataCollector:
    def __init__(self,
                 output="data",
                 hz=10,
                 scenario="test",
                 metadata=None,
                 queue_size=32,
                 jpeg_quality=95):
        
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("hz must be finite and positive")
        
        if queue_size < 1 or not 1 <= jpeg_quality <= 100:
            raise ValueError("Invalid queue_size/jpeg_quality")
        
        self.period = 1.0 / hz
        self.scenario = scenario
        self.quality = jpeg_quality
        self.path = Path(output) / (time.strftime("run_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
        
        (self.path / "images").mkdir(parents=True, exist_ok=True)
        
        self.lock = threading.RLock()
        self.jobs = queue.Queue(maxsize=queue_size)
        self.error = None
        self.closed = False
        self.last_frame = None
        self.last_seen = None
        self.saved = 0
        self.dropped = 0
        self.index = 0
        
        # Unknown until console explicitly reports a successfully applied command.
        self.command = None
        self.command_id = 0
        self.commands_file = open(str(self.path / "commands.csv"), "w", newline="")
        self.commands = csv.writer(self.commands_file)
        self.commands.writerow(["command_id", "applied_mono_s", "wall_time_s", "left_command", "right_command"])
        self.commands_file.flush()
        
        config = dict(metadata or {})
        config.update(hz=hz,
                      scenario=scenario,
                      jpeg_quality=jpeg_quality,
                      frame_time_semantics="host receipt/callback time, NOT sensor exposure time",
                      command_semantics="software write completed, NOT measured motor motion",
                      color_order="BGR",
                      clock="time.monotonic seconds on this host")
        
        (self.path / "metadata.json").write_text(json.dumps(config, indent=2, ensure_ascii=False), encoding="utf-8")
        self.worker = threading.Thread(target=self.write_loop, daemon=True)
        self.worker.start()
        
    def check(self):
        if self.error is not None:
            raise RuntimeError("Dataset writer failed: {}".format(self.error)) from self.error
        
    def record_command(self, left, right):
        left, right = float(left), float(right)
        
        if not all(math.isfinite(x) and -1 <= x <= 1 for x in (left, right)):
            raise ValueError("Commands must be finite and within [-1, 1]")
        
        with self.lock:
            self.check()
            
            if self.closed:
                raise RuntimeError("Collector is closed")
            
            stamp = time.monotonic()
            
            self.command_id += 1
            self.commands.writerow([self.command_id, stamp, time.time(), left, right])
            self.commands_file.flush()
            self.command = (self.command_id, stamp, left, right)
            
    def add_frame(self, bgr):
        """Call once per NEW frame immediately after read()/in camera callback.
        Returns True if queued, False if rate-limited or queue full.
        Do not repeatedly submit a cached camera.value.
        """
        with self.lock:
            self.check()
            
            if self.closed:
                raise RuntimeError("Collector is closed")
            
            now, wall = time.monotonic(), time.time()
            
            if self.last_seen is not None and now < self.last_seen:
                raise RuntimeError("Monotonic clock moved backwards")
            
            self.last_seen = now
            if self.last_frame is not None and now - self.last_frame < self.period:
                return False
            
            self.last_frame = now
            frame_id = self.index
            self.index += 1
            cmd = self.command
            
            # Copy prevents camera buffer reuse from changing queued images.
            item = (frame_id, now, wall, cmd, bgr.copy())
            
            try:
                self.jobs.put_nowait(item)
            except queue.Full:
                self.dropped += 1
                return False
            
            return True
        
    def _write_loop(self):
        try:
            import cv2
            with open (str(self.path / "records.csv"), "w", newline="") as f:
                rows = csv.writer(f)
                rows.writerow(["frame_id", "image", "frame_received_mono_s", "wall_time_s",
                               "command_valid", "command_id", "command_applied_mono_s",
                               "left_command", "right_command", "command_age_s", "scenario"])
                f.flush()
                
                while True:
                    job = self.jobs.get()
                    
                    if job is None:
                        break
                    
                    index, stamp, wall, cmd, frame = job
                    name = "images/{:08d}.jpg".format(index)
                    
                    if not cv2.imwrite(str(self.path / name), frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.quality]):
                        raise IOError("Could not write " + name)
                    
                    labels = [False, "", "", "", "", ""] if cmd is None else [
                        True, cmd[0], cmd[1], cmd[2], cmd[3], stamp - cmd[1]]
                    
                    rows.writerow([index, name, stamp, wall] + labels + [self.scenario])
                    f.flush()
                    
                    self.saved += 1
                    
        except BaseException as exc:
            self.error = exc
            
    def close(self):
        with self.lock:
            if self.closed:
                self.check()
                return
            
        # Do not deadlock on a full queue if the writer has failed.
        while self.worker.is_alive():
            try:
                self.jobs.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
            
        self.worker.join()
        self.commands_file.close()
        
        summary = dict(saved=self.saved,
                       dropped_queue_full=self.dropped,
                       attempted_samples=self.index,
                       error=str(self.error) if self.error else None)
        (self.path / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.check()
        
def main():
    p = argparse.ArgumentParser(description="Camera-only recording test; does not control motors")
    p.add_argument("--camera", choices=["usb", "csi"], default="usb")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--width", type=int, default=224)
    p.add_argument("--height", type=int, default=224)
    p.add_argument("--camera-fps", type=int, default=30)
    p.add_argument("--flip", type=int, default=0)
    p.add_argument("--hz", type=float, default=10)
    p.add_argument("--seconds", type=float, default=30)
    p.add_argument("--output", default="data")
    p.add_argument("--scenario", default="camera_test")
    p.add_argument("--preview", action="store_true")
    a = p.parse_args()
    if a.seconds <= 0 or not math.isfinite(a.seconds):
        p.error("seconds must be finite and positive")
    import cv2
    if a.camera == "csi":
        pipeline = (
            "nvarguscamerasrc sensor-id={index} ! video/x-raw(memory:NVMM), "
            "width=(int)1280, height=(int)720, format=(string)NV12, "
            "framerate=(fraction){fps}/1 ! nvvidconv flip-method={flip} ! "
            "video/x-raw, width=(int){w}, height=(int){h}, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink max-buffers=1 drop=true sync=false"
        ).format(index=a.index, fps=a.camera_fps, flip=a.flip, w=a.width, h=a.height)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(a.index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        cap.set(cv2.CAP_PROP_FPS, a.camera_fps)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("Camera open failed. Check camera ownership, backend and supported mode.")
    collector = None
    try:
        collector = DataCollector(a.output, a.hz, a.scenario, vars(a))
        print("Saving to:", collector.path, flush=True)
        print("Camera test: motor labels remain UNKNOWN. Ctrl+C to stop.", flush=True)
        start = time.monotonic()
        while time.monotonic() - start < a.seconds:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera read failed")
            collector.add_frame(frame)
            if a.preview:
                cv2.imshow("Collection", frame)
                if cv2.waitKey(1) & 0xff == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if a.preview:
            cv2.destroyAllWindows()
        if collector is not None:
            collector.close()
            print("Saved: {}, dropped: {}".format(collector.saved, collector.dropped))
        
        
if __name__ == "__main__":
    main()