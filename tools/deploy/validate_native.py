#!/usr/bin/env python3
"""Bench-run the native app through a transparent UART tap, retaining wire evidence.

Run as root with the old service stopped and the aircraft disarmed, props removed.
The tap generates no MAVLink messages; only the native application transmits.
"""
import argparse
import collections
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import termios
import time
import tty

from pymavlink.dialects.v20 import ardupilotmega as mav


def run(binary, configuration, output, seconds):
    output.mkdir(parents=True, exist_ok=False)
    config = json.loads(configuration.read_text())
    serial = os.open(config["device"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    master = slave = -1
    process = None
    counters = collections.Counter()
    lags = []
    resets = set()
    accepted = False
    alignment_requested = False
    failure = None
    wire_rows = []
    wire_bytes = 0
    max_poll_gap_ms = 0
    parsers = {direction: mav.MAVLink(None) for direction in ("fc", "native")}
    for parser in parsers.values():
        parser.robust_parsing = True
    pending = {"fc": bytearray(), "native": bytearray()}
    try:
        fcntl.ioctl(serial, termios.TIOCEXCL)
        tty.setraw(serial)
        attributes = termios.tcgetattr(serial)
        attributes[4] = attributes[5] = getattr(termios, "B" + str(config["baud"]))
        termios.tcsetattr(serial, termios.TCSANOW, attributes)
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        config["device"] = os.ttyname(slave)
        config["recording_dir"] = str(output / "recordings")
        # Share the established counter location, after stopping its previous owner.
        config_path = output / "flight.json"
        config_path.write_text(json.dumps(config, indent=2) + "\n")
        deadline = time.monotonic() + seconds
        environment = dict(os.environ, SUDO_USER="luke")
        with (output / "application.log").open("w") as log:
            process = subprocess.Popen([str(binary), str(config_path)], stdout=log, stderr=log,
                                       env=environment, start_new_session=True)
            previous_poll = time.monotonic_ns()
            while time.monotonic() < deadline and process.poll() is None:
                poll_time = time.monotonic_ns()
                max_poll_gap_ms = max(max_poll_gap_ms, (poll_time - previous_poll) / 1e6)
                previous_poll = poll_time
                readable, writable, _ = select.select(
                    [serial, master], [fd for direction, fd in (("fc", serial), ("native", master))
                                       if pending[direction]], [], .02)
                for fd in readable:
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        continue
                    if not data:
                        raise RuntimeError("UART/PTY stream ended")
                    direction = "fc" if fd == serial else "native"
                    destination = "native" if fd == serial else "fc"
                    pending[destination].extend(data)
                    if len(pending[destination]) > 65536:
                        raise RuntimeError("UART tap backlog exceeded 64 KiB")
                    for message in parsers[direction].parse_buffer(data) or []:
                        kind = message.get_type()
                        counters[direction + ":" + kind] += 1
                        row = message.to_dict()
                        row.update(direction=direction, host_ns=time.monotonic_ns(),
                                   system=message.get_srcSystem(), component=message.get_srcComponent())
                        # Disk flushes must not delay the UART forwarding loop.
                        encoded = json.dumps(row, default=lambda value: list(value)) + "\n"
                        wire_bytes += len(encoded)
                        if wire_bytes > 32 * 1024 * 1024:
                            raise RuntimeError("wire evidence exceeded 32 MiB memory limit")
                        wire_rows.append(encoded)
                        if direction == "fc" and kind == "HEARTBEAT" and message.get_srcComponent() == 1:
                            if message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                                raise RuntimeError("controller armed during bench validation")
                        if (direction == "native" and kind == "COMMAND_LONG" and
                                message.command == 218 and message.param1 == 80 and
                                message.param2 == 2):
                            alignment_requested = True
                        if (direction == "fc" and kind == "COMMAND_ACK" and
                                message.get_srcSystem() == 1 and message.get_srcComponent() == 1 and
                                message.command == 218 and alignment_requested and
                                message.target_system in (0, 1) and
                                message.target_component in (0, 197)):
                            accepted |= message.result == mav.MAV_RESULT_ACCEPTED
                        if direction == "native" and kind == "VISION_POSITION_ESTIMATE":
                            lags.append((row["host_ns"] - message.usec * 1000) / 1e6)
                            resets.add(message.reset_counter)
                for fd in writable:
                    destination = "fc" if fd == serial else "native"
                    try:
                        written = os.write(fd, pending[destination])
                        del pending[destination][:written]
                    except BlockingIOError:
                        pass
            if process.poll() is not None:
                failure = f"native application exited early: {process.returncode}"
            process.send_signal(signal.SIGTERM) if process.poll() is None else None
            process.wait(timeout=15)
    except Exception as error:
        failure = str(error)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                failure = "native application required forced termination"
        for fd in (master, slave, serial):
            if fd >= 0:
                os.close(fd)
    lags.sort()
    (output / "wire.jsonl").write_text("".join(wire_rows))
    summary = dict(binary=str(binary), error=failure,
                   exit_code=None if process is None else process.returncode,
                   max_poll_gap_ms=max_poll_gap_ms,
                   counts=dict(counters), alignment_accepted=accepted, reset_counters=sorted(resets),
                   poses=len(lags), lag_p95_ms=lags[int(.95 * (len(lags) - 1))] if lags else None,
                   lag_max_ms=max(lags) if lags else None)
    summary["passed"] = (failure is None and process.returncode == 0 and accepted and
                         len(lags) >= 100 and min(lags) >= 0 and
                         summary["lag_max_ms"] < config["max_lag"] * 1000)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=90)
    args = parser.parse_args()
    raise SystemExit(run(args.binary, args.config, args.output, args.seconds))
