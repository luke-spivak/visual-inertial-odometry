"""Exercise the native executable's signals with a virtual UART and no sensors."""
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time


def check(executable, configuration):
    for stop_signal in (signal.SIGTERM, signal.SIGINT):
        with tempfile.TemporaryDirectory(prefix="vio-native-main-") as directory:
            root = Path(directory)
            master, slave = pty.openpty()
            config = json.loads(Path(configuration).read_text())
            config.update(device=os.ttyname(slave), estimate_dir=str(root / "runtime"),
                          recording_dir=str(root / "recordings"),
                          estimator_config=str(Path(configuration).resolve().with_name("estimator_config.yaml")))
            path = root / "flight.json"
            path.write_text(json.dumps(config))
            os.close(slave)
            with (root / "output.log").open("w+") as log:
                process = subprocess.Popen([executable, str(path)], stdout=log, stderr=log)
                try:
                    deadline = time.monotonic() + 15
                    ready = False
                    while time.monotonic() < deadline and process.poll() is None:
                        if select.select([master], [], [], 0.05)[0]:
                            try:
                                ready = bool(os.read(master, 1024))
                            except OSError:  # PTY slave is not open yet.
                                time.sleep(0.01)
                            if ready:
                                break
                    if not ready:
                        log.seek(0)
                        raise AssertionError("native application did not reach heartbeat wait:\n" + log.read())
                    # A second native owner must fail before it touches the UART.
                    duplicate = subprocess.run([executable, str(path)], capture_output=True, timeout=5)
                    assert duplicate.returncode != 0
                    assert b"another native application" in duplicate.stderr
                    process.send_signal(stop_signal)
                    assert process.wait(timeout=5) == 0
                    assert (root / "runtime/reset_counter").read_text().strip() == "0"
                    assert not (root / "recordings").exists(), "capture started without a controller"
                    with (root / "runtime/application.lock").open() as lock:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait()
                    os.close(master)
    print("Native application SIGINT/SIGTERM and singleton checks passed")


if __name__ == "__main__":
    check(sys.argv[1], sys.argv[2])
