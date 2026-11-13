#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JETANK: D-pad driving, A recording toggle, B quit. Python 3.6+."""
import argparse
import array
import math
import os
import select
import struct
import threading
import time
from fcntl import ioctl

from collector import DataCollector

AXIS_NAMES = {0x00: 'x', 0x01: 'y', 0x02: 'z', 0x03: 'rx',
              0x04: 'ry', 0x05: 'rz', 0x10: 'lr', 0x11: 'fb'}
BUTTON_NAMES = {0x130: 'a', 0x131: 'b', 0x133: 'x', 0x134: 'y',
                0x220: 'dpad_up', 0x221: 'dpad_down',
                0x222: 'dpad_left', 0x223: 'dpad_right',
                0x2c0: 'dpad_left', 0x2c1: 'dpad_right',
                0x2c2: 'dpad_up', 0x2c3: 'dpad_down'}


class Gamepad:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            name = array.array('B', [0] * 128)
            ioctl(self.fd, 0x80006a13 + (len(name) << 16), name)
            self.name = name.tobytes().split(b'\0')[0].decode('utf-8', 'replace')
            count = array.array('B', [0])
            ioctl(self.fd, 0x80016a11, count)
            na = count[0]
            ioctl(self.fd, 0x80016a12, count)
            nb = count[0]
            axes = array.array('B', [0] * 64)
            ioctl(self.fd, 0x80406a32, axes)
            buttons = array.array('H', [0] * 512)
            ioctl(self.fd, 0x84006a34, buttons)
            self.axis_names = [AXIS_NAMES.get(v, 'axis_0x{:02x}'.format(v)) for v in axes[:na]]
            self.button_names = [BUTTON_NAMES.get(v, 'button_0x{:03x}'.format(v)) for v in buttons[:nb]]
        except Exception:
            os.close(self.fd)
            raise
        self.axes = {}
        self.buttons = {}
        self.pressed = []
        self.buffer = b''

    def poll(self):
        self.pressed = []
        for _ in range(256):
            if not select.select([self.fd], [], [], 0)[0]:
                break
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                break
            if not data:
                raise RuntimeError('Gamepad disconnected')
            self.buffer += data
            while len(self.buffer) >= 8:
                packet, self.buffer = self.buffer[:8], self.buffer[8:]
                _, value, kind, number = struct.unpack('<IhBB', packet)
                initial = bool(kind & 0x80)
                kind &= 0x7f
                if kind == 2:
                    self.axes[number] = max(-1.0, value / 32767.0)
                elif kind == 1:
                    old = self.buttons.get(number, False)
                    self.buttons[number] = bool(value)
                    if value and not old and not initial:
                        self.pressed.append(number)

    def button_index(self, name):
        return self.button_names.index(name) if name in self.button_names else None

    def direction(self, h_axis, v_axis):
        if h_axis is not None and v_axis is not None:
            h = self.axes.get(h_axis, 0)
            v = self.axes.get(v_axis, 0)
        else:
            def held(name):
                n = self.button_index(name)
                return int(self.buttons.get(n, False)) if n is not None else 0
            h = held('dpad_right') - held('dpad_left')
            v = held('dpad_down') - held('dpad_up')
        return (1 if h > .5 else -1 if h < -.5 else 0,
                1 if v > .5 else -1 if v < -.5 else 0)

    def close(self):
        os.close(self.fd)


def motor_commands(h, v, speed, turn):
    forward = -v * speed
    left, right = forward + h * turn, forward - h * turn
    # Keep the largest motor command at speed, including diagonals.
    factor = max(1.0, max(abs(left), abs(right)) / speed)
    return left / factor, right / factor


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--device', default='/dev/input/js0')
    p.add_argument('--inspect', action='store_true')
    p.add_argument('--drive', action='store_true')
    p.add_argument('--record-button', type=int)
    p.add_argument('--quit-button', type=int)
    p.add_argument('--dpad-x-axis', type=int)
    p.add_argument('--dpad-y-axis', type=int)
    p.add_argument('--speed', type=float, default=.20)
    p.add_argument('--turn-speed', type=float, default=.15)
    p.add_argument('--hz', type=float, default=10)
    p.add_argument('--width', type=int, default=300)
    p.add_argument('--height', type=int, default=300)
    p.add_argument('--output', default='data')
    p.add_argument('--scenario', default='manual_line_following')
    a = p.parse_args()
    if not all(math.isfinite(x) and 0 < x <= 1 for x in (a.speed, a.turn_speed)):
        p.error('speed/turn-speed must be within (0, 1]')
    if not math.isfinite(a.hz) or a.hz <= 0 or min(a.width, a.height) <= 0:
        p.error('Invalid hz/width/height')

    pad = Gamepad(a.device)
    robot = camera = callback = collector = None
    lock = threading.RLock()
    state = {'last': None, 'error': None, 'closing': False}
    current = None
    try:
        print('Device:', pad.name, flush=True)
        print('Axes:', list(enumerate(pad.axis_names)), flush=True)
        print('Buttons:', list(enumerate(pad.button_names)), flush=True)
        if a.inspect:
            report = 0
            while True:
                pad.poll()
                if time.monotonic() - report >= .25:
                    print('axes={} buttons={}'.format(pad.axes, pad.buttons), flush=True)
                    report = time.monotonic()
                time.sleep(.01)

        rec = a.record_button if a.record_button is not None else pad.button_index('a')
        quit_btn = a.quit_button if a.quit_button is not None else pad.button_index('b')
        if rec is None or quit_btn is None:
            raise RuntimeError('A/B mapping missing. Inspect then set --record-button / --quit-button.')
        if rec == quit_btn or not all(0 <= n < len(pad.button_names) for n in (rec, quit_btn)):
            raise ValueError('Invalid recording/quit button indices')
        hx = a.dpad_x_axis
        vy = a.dpad_y_axis
        if hx is None and vy is None and 'lr' in pad.axis_names and 'fb' in pad.axis_names:
            hx, vy = pad.axis_names.index('lr'), pad.axis_names.index('fb')
        if (hx is None) != (vy is None):
            raise ValueError('Set both D-pad axes together')
        if hx is not None:
            if hx == vy or not all(0 <= n < len(pad.axis_names) for n in (hx, vy)):
                raise ValueError('Invalid D-pad axis indices')
        elif not all(n in pad.button_names for n in ('dpad_up', 'dpad_down', 'dpad_left', 'dpad_right')):
            raise RuntimeError('No D-pad mapping. Inspect and set both --dpad-x-axis / --dpad-y-axis.')
        print('Record button={} Quit button={} D-pad axes={}'.format(rec, quit_btn, (hx, vy)), flush=True)

        from jetbot import Camera
        if a.drive:
            from jetbot import Robot
            robot = Robot()
            robot.stop()

        def apply(left, right, force=False):
            nonlocal current
            with lock:
                if not force and current == (left, right):
                    return
                if robot is not None:
                    if collector is not None:
                        with collector.lock:
                            robot.left_motor.value = left
                            robot.right_motor.value = right
                            collector.record_command(left, right)
                    else:
                        robot.left_motor.value = left
                        robot.right_motor.value = right
                current = (left, right)

        def on_frame(change):
            with lock:
                if state['closing']:
                    return
                try:
                    frame = change['new']
                    if frame is None:
                        return
                    state['last'] = time.monotonic()
                    if collector is not None:
                        collector.add_frame(frame)
                except Exception as exc:
                    state['error'] = exc

        callback = on_frame
        camera = Camera.instance(width=a.width, height=a.height)
        camera.observe(callback, names='value')
        apply(0, 0, force=True)
        print('MODE: {} | Recording OFF'.format('REAL MOTORS' if a.drive else 'DRY-RUN'), flush=True)
        print('Center D-pad once. A: record toggle, B: quit, Ctrl+C: quit.', flush=True)
        centered = False
        started = time.monotonic()
        report = 0
        while True:
            pad.poll()
            with lock:
                if state['error'] is not None:
                    raise RuntimeError('Camera/collector error') from state['error']
                if collector is not None:
                    collector.check()
                last_frame = state['last']
            if quit_btn in pad.pressed:
                break
            now = time.monotonic()
            if last_frame is None and now - started > 10:
                raise RuntimeError('No new camera frames')
            fresh = last_frame is not None and now - last_frame < 1.0
            h, v = pad.direction(hx, vy)
            if h == 0 and v == 0:
                centered = True
            if not fresh:
                centered = False
            if rec in pad.pressed:
                with lock:
                    apply(0, 0, force=True)
                    centered = False
                    if collector is None:
                        if not fresh:
                            print('Camera not ready; recording remains OFF', flush=True)
                        else:
                            metadata = vars(a).copy()
                            metadata.update(gamepad=pad.name, axis_names=pad.axis_names,
                                            button_names=pad.button_names,
                                            mode='drive' if a.drive else 'dry-run')
                            collector = DataCollector(a.output, a.hz, a.scenario, metadata)
                            apply(0, 0, force=True)
                            print('Recording ON:', collector.path.resolve(), flush=True)
                    else:
                        old, collector = collector, None
                        old.close()
                        print('Recording OFF:', old.path.resolve(), flush=True)
            left, right = motor_commands(h, v, a.speed, a.turn_speed)
            if not fresh or not centered:
                left, right = 0, 0
            apply(left, right)
            if now - report >= 1:
                print('D-pad=({}, {}) L={:+.2f} R={:+.2f} recording={} saved={}'.format(
                    h, v, left, right, collector is not None, collector.saved if collector else 0), flush=True)
                report = now
            time.sleep(.01)
    except KeyboardInterrupt:
        print('Stopping...', flush=True)
    finally:
        # Stop before any potentially slow file/camera cleanup.
        try:
            with lock:
                if robot is not None:
                    robot.stop()
                    if collector is not None:
                        try:
                            collector.record_command(0, 0)
                        except Exception as exc:
                            print('Stop label failed:', exc, flush=True)
        finally:
            with lock:
                state['closing'] = True
            try:
                if camera is not None:
                    try:
                        if callback is not None:
                            camera.unobserve(callback, names='value')
                    finally:
                        camera.stop()
            finally:
                try:
                    if collector is not None:
                        collector.close()
                        print('Saved:', collector.path.resolve(), flush=True)
                finally:
                    pad.close()


if __name__ == '__main__':
    main()
