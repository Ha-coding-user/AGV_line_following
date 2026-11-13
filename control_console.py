"""Linux USB gamepad teleoperation. Requires collector.py beside this file.
Python 3.6+. No pygame required. Inspect mapping before enabling motors.
"""
import argparse
import math
import os
import select
import struct
import threading
import time

from collector import DataCollector


class Joystick:
    def __init__(self, device):
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = {}
        self.buttons = {}
        self.buffer = b""
        self.button_events = []

    def poll(self):
        self.button_events = []

        # 한 번에 처리할 이벤트 수를 제한해 제어 루프가 멈추지 않게 함
        for _ in range(256):
            if not select.select([self.fd], [], [], 0)[0]:
                break

            try:
                chunk = os.read(self.fd, 8)
            except BlockingIOError:
                break

            if not chunk:
                raise RuntimeError("Gamepad disconnected")

            self.buffer += chunk

            while len(self.buffer) >= 8:
                packet, self.buffer = self.buffer[:8], self.buffer[8:]

                _, value, kind, number = struct.unpack(
                    "<IhBB", packet
                )

                initial = bool(kind & 0x80)
                kind &= 0x7f

                if kind == 2:
                    self.axes[number] = max(
                        -1.0, value / 32767.0
                    )

                elif kind == 1:
                    self.buttons[number] = bool(value)

                    if not initial:
                        self.button_events.append(
                            (number, bool(value))
                        )

    def close(self):
        os.close(self.fd)


def deadzone(value, threshold):
    """스틱 중앙 부근의 작은 입력 제거."""
    if abs(value) <= threshold:
        return 0.0

    return math.copysign(
        (abs(value) - threshold) / (1 - threshold),
        value,
    )


def mix(throttle, steering, speed, turn):
    """전후진·회전 입력을 좌우 모터 명령으로 변환."""
    left = throttle * speed + steering * turn
    right = throttle * speed - steering * turn

    scale = max(1.0, abs(left), abs(right))

    return left / scale, right / scale


def write_motors(robot, left, right):
    # 기존 JetTank가 다른 모터 API를 사용한다면 이 부분 수정
    robot.set_motors(left, right)


def main():
    p = argparse.ArgumentParser()

    p.add_argument("--device", default="/dev/input/js0")

    p.add_argument(
        "--inspect",
        action="store_true",
        help="Print mapping; no camera/motors",
    )
    p.add_argument(
        "--drive",
        action="store_true",
        help="Enable real motors; default is dry-run",
    )

    p.add_argument("--throttle-axis", type=int, default=1)
    p.add_argument("--steer-axis", type=int, default=0)

    p.add_argument(
        "--throttle-sign",
        type=int,
        choices=[-1, 1],
        default=-1,
    )
    p.add_argument(
        "--steer-sign",
        type=int,
        choices=[-1, 1],
        default=1,
    )

    p.add_argument(
        "--deadman-button",
        type=int,
        default=0,
        help="Hold to move",
    )
    p.add_argument("--quit-button", type=int, default=7)

    p.add_argument("--speed", type=float, default=0.15)
    p.add_argument("--turn-speed", type=float, default=0.12)
    p.add_argument("--deadzone", type=float, default=0.12)
    p.add_argument("--camera-timeout", type=float, default=1.0)

    p.add_argument("--hz", type=float, default=10)
    p.add_argument("--output", default="data")
    p.add_argument(
        "--scenario",
        default="manual_line_following",
    )

    a = p.parse_args()

    for name in ("speed", "turn_speed"):
        value = getattr(a, name)

        if not math.isfinite(value) or not 0 < value <= 1:
            p.error(name + " must be within (0, 1]")

    if not 0 <= a.deadzone < 1 or not 0 < a.camera_timeout <= 10:
        p.error("Invalid deadzone/camera-timeout")

    if not math.isfinite(a.hz) or a.hz <= 0:
        p.error("hz must be finite and positive")

    if min(
        a.throttle_axis,
        a.steer_axis,
        a.deadman_button,
        a.quit_button,
    ) < 0:
        p.error("Axis/button indices must be nonnegative")

    if a.deadman_button == a.quit_button:
        p.error("Deadman and quit buttons must differ")

    js = Joystick(a.device)

    robot = None
    camera = None
    collector = None
    callback = None

    frame_lock = threading.RLock()

    state = {
        "last_frame": None,
        "error": None,
        "closing": False,
    }

    current = None
    released = False
    last_report = 0.0

    try:
        # ----------------------------------------------------
        # 게임패드 축·버튼 확인 모드
        # 카메라와 모터는 초기화하지 않음
        # ----------------------------------------------------
        if a.inspect:
            print(
                "Move sticks and press buttons. Ctrl+C to exit.",
                flush=True,
            )

            while True:
                js.poll()

                now = time.monotonic()

                if now - last_report >= 0.25:
                    print(
                        "axes={} buttons={}".format(
                            js.axes, js.buttons
                        ),
                        flush=True,
                    )
                    last_report = now

                time.sleep(0.01)

        # ----------------------------------------------------
        # 카메라·로봇 초기화
        # ----------------------------------------------------
        from jetbot import Camera

        if a.drive:
            from jetbot import Robot

            robot = Robot()
            robot.stop()

        metadata = vars(a).copy()
        metadata.update(
            command_source="gamepad",
            motor_labels=(
                "actual software writes"
                if a.drive
                else "unknown (dry-run)"
            ),
        )

        collector = DataCollector(
            a.output,
            a.hz,
            a.scenario,
            metadata,
        )

        def apply(left, right):
            nonlocal current

            if current == (left, right):
                return

            if robot is not None:
                with collector.lock:
                    write_motors(robot, left, right)
                    collector.record_command(left, right)

            current = (left, right)

        # 실제 주행 모드에서는 초기 정지 명령과 라벨 기록
        apply(0.0, 0.0)

        # ----------------------------------------------------
        # 새 카메라 프레임 수신
        # ----------------------------------------------------
        def on_frame(change):
            with frame_lock:
                if state["closing"]:
                    return

                try:
                    frame = change["new"]

                    if frame is None:
                        return

                    # JetBot Camera의 BGR 이미지
                    collector.add_frame(frame)

                    state["last_frame"] = time.monotonic()

                except Exception as exc:
                    state["error"] = exc

        callback = on_frame

        camera = Camera.instance()
        camera.observe(callback, names="value")

        started = time.monotonic()

        print(
            "Saving to: {}".format(collector.path.resolve()),
            flush=True,
        )
        print(
            "MODE: {}".format(
                "REAL MOTORS"
                if a.drive
                else "DRY-RUN (unknown motor labels)"
            ),
            flush=True,
        )
        print(
            "Release deadman once, then hold it to move. "
            "Quit button or Ctrl+C exits.",
            flush=True,
        )

        # ----------------------------------------------------
        # 메인 제어 루프
        # ----------------------------------------------------
        while True:
            # 장치 읽기 오류가 나면 finally에서 정지 시도
            js.poll()
            collector.check()

            with frame_lock:
                frame_time = state["last_frame"]
                error = state["error"]

            if error is not None:
                raise RuntimeError(
                    "Camera callback failed"
                ) from error

            quit_pressed = any(
                number == a.quit_button and pressed
                for number, pressed in js.button_events
            )

            if quit_pressed:
                break

            now = time.monotonic()

            held = js.buttons.get(
                a.deadman_button, False
            )

            # 시작 후 주행 허용 버튼을 한 번 놓아야 활성화
            if (
                a.deadman_button in js.buttons
                and not held
            ):
                released = True

            fresh = (
                frame_time is not None
                and now - frame_time <= a.camera_timeout
            )

            ready = (
                a.throttle_axis in js.axes
                and a.steer_axis in js.axes
            )

            throttle = deadzone(
                js.axes.get(a.throttle_axis, 0)
                * a.throttle_sign,
                a.deadzone,
            )

            steering = deadzone(
                js.axes.get(a.steer_axis, 0)
                * a.steer_sign,
                a.deadzone,
            )

            left, right = mix(
                throttle,
                steering,
                a.speed,
                a.turn_speed,
            )

            # 버튼을 누르고 있고 카메라가 정상일 때만 이동
            if not (released and held and fresh and ready):
                left, right = 0.0, 0.0

            apply(left, right)

            if (
                frame_time is None
                and now - started > 10
            ):
                raise RuntimeError(
                    "No new camera frame for 10 seconds"
                )

            if now - last_report >= 1:
                print(
                    "L={:+.3f} R={:+.3f} "
                    "camera={} hold={} saved={} dropped={}".format(
                        left,
                        right,
                        fresh,
                        held,
                        collector.saved,
                        collector.dropped,
                    ),
                    flush=True,
                )
                last_report = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping...", flush=True)

    finally:
        # ----------------------------------------------------
        # 모터 정지를 먼저 시도
        # ----------------------------------------------------
        try:
            if robot is not None:
                if collector is not None:
                    with collector.lock:
                        robot.stop()

                        try:
                            collector.record_command(
                                0.0, 0.0
                            )
                        except Exception as exc:
                            print(
                                "Stop label could not be saved:",
                                exc,
                                flush=True,
                            )
                else:
                    robot.stop()

        finally:
            with frame_lock:
                state["closing"] = True

            try:
                if camera is not None:
                    try:
                        if callback is not None:
                            camera.unobserve(
                                callback,
                                names="value",
                            )
                    finally:
                        camera.stop()

            finally:
                try:
                    if collector is not None:
                        collector.close()

                        print(
                            "Dataset:",
                            collector.path.resolve(),
                            flush=True,
                        )

                finally:
                    js.close()


if __name__ == "__main__":
    main()