"""Small operational CLIs; persistent aircraft settings live in flight.json."""
import argparse
from dataclasses import asdict

from flight_config import DEFAULT_CONFIG, load_config


def parse_options(command, user_home, argv=None):
    descriptions = {
        "flight": "Supervise onboard VIO using the aircraft configuration.",
        "capture": "Capture one bench VIO session using the aircraft configuration.",
        "bridge": "Inspect or stream estimates using the aircraft configuration.",
    }
    parser = argparse.ArgumentParser(description=descriptions[command])
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="flight JSON configuration (default: src/config/flight.json)")
    if command == "capture":
        parser.add_argument("out", help="output prefix, e.g. ~/vio/bench")
        parser.add_argument("--secs", type=int, default=0, help="duration; 0 = until Ctrl-C")
        parser.add_argument("--no-record", action="store_true", help="estimate without raw recording")
        # Private supervisor-to-capture plumbing keeps live poses on tmpfs.
        parser.add_argument("--est", help=argparse.SUPPRESS)
    elif command == "bridge":
        parser.add_argument("est", nargs="?", help="estimate file")
        parser.add_argument("--dry-run", action="store_true", help="print only; open no link")
        parser.add_argument("--expect-accel", action="store_true", help="show the configured level IMU reading")
    options = parser.parse_args(argv)
    if command == "capture" and options.secs < 0:
        parser.error("--secs must not be negative")
    if command == "bridge" and not options.est and not options.expect_accel:
        parser.error("give the estimate file, or --expect-accel")
    try:
        config, path = load_config(options.config, user_home)
    except (OSError, ValueError) as exc:
        parser.error(f"invalid flight configuration: {exc}")
    del options.config
    return argparse.Namespace(**asdict(config), **vars(options), flight_config=path)
