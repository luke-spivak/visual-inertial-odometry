"""CLI definitions and shared option groups for onboard commands.

Runtime modules consume parsed options; defaults preserve the aircraft settings.
"""
import argparse
import os


def _parser(description):
    return argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawDescriptionHelpFormatter)


def _exposure_options(parser):
    group = parser.add_argument_group("camera exposure")
    group.add_argument("--shutter", type=int, help="fixed exposure in microseconds")
    group.add_argument("--gain", type=float, help="analogue gain with --shutter; default 1")
    group.add_argument("--exposure-sweep", action="store_true",
                       help="select the brightest exposure below the clipping limit")


def _link_options(parser):
    group = parser.add_argument_group("flight-controller connection")
    group.add_argument("--device", default="/dev/ttyAMA0",
                       help="serial device or pymavlink URL (e.g. udpout:HOST:14550)")
    group.add_argument("--baud", type=int, default=230400,
                       help="must match SERIAL3_BAUD on the FC (230 = 230400)")


def _mount_options(parser, *, flight):
    group = parser.add_argument_group("camera mounting")
    group.add_argument("--tilt-deg", type=float, default=15.0,
                       help="camera pitch below horizontal, degrees")
    # Preserve the existing interfaces: flight defaults upside down, diagnostics
    # default upright. Do not silently invert a deployed mounting convention.
    if flight:
        group.add_argument("--upright", action="store_true",
                           help="override the flight default of an upside-down camera")
    else:
        group.add_argument("--upside-down", action="store_true",
                           help="camera image is upside down; check with --expect-accel")


def capture_parser(user_home, description=None):
    parser = _parser(description)
    parser.add_argument("out", help="output prefix, e.g. ~/vio/walk1")
    parser.add_argument("--secs", type=int, default=0, help="run length; 0 = until Ctrl-C")
    sensors = parser.add_argument_group("sensor capture")
    sensors.add_argument("--fps", type=float, default=20)
    # A 4 ms cap preserves indoor SNR; the 1 ms calibration exposure was too dark.
    sensors.add_argument("--max-shutter", type=int, default=4000, help="AE shutter cap, microseconds")
    sensors.add_argument("--watermark", type=int, default=8, help="IMU FIFO watermark")
    # Motor vibration is around 176–195 Hz; useful motion is below about 20 Hz.
    sensors.add_argument("--imu-lpf", type=float, default=50.0, help="IMU low-pass cutoff, Hz; 0 = off")
    _exposure_options(parser)
    recording = parser.add_argument_group("recording")
    recording.add_argument("--no-record", action="store_true")
    recording.add_argument("--est", help="estimate path; default <out>.est.txt; supervisor uses tmpfs")
    estimator = parser.add_argument_group("OpenVINS runtime")
    estimator.add_argument("--bin", default=f"{user_home}/vio_live/vio_live")
    estimator.add_argument("--config", default=f"{user_home}/vio_live/config/estimator_config.yaml")
    estimator.add_argument("--verbosity", default="WARNING", help="OpenVINS print level")
    return parser


def flight_parser(user_home, description=None):
    parser = _parser(description)
    recording = parser.add_argument_group("recording")
    recording.add_argument("--dir", default=os.path.join(user_home, "vio"), help="recording directory")
    recording.add_argument("--est-dir", default="/run/vio", help="tmpfs for live estimate files")
    recording.add_argument("--min-free-gb", type=float, default=20.0,
                           help="record only with this much free space")
    _link_options(parser)
    _mount_options(parser, flight=True)
    _exposure_options(parser)
    return parser


def bridge_parser(description=None):
    parser = _parser(description)
    parser.add_argument("est", nargs="?", help="estimate file, e.g. ~/vio/f1.est.txt")
    _link_options(parser)
    _mount_options(parser, flight=False)
    stream = parser.add_argument_group("estimate streaming")
    stream.add_argument("--no-velocity", action="store_true", help="send position only")
    stream.add_argument("--max-lag", type=float, default=0.5,
                        help="drop poses older than this many seconds; 0 disables (file replay)")
    diagnostics = parser.add_argument_group("diagnostics")
    diagnostics.add_argument("--dry-run", action="store_true", help="print only; open no link")
    diagnostics.add_argument("--expect-accel", action="store_true",
                             help="print expected IMU acceleration while level and still, then exit")
    return parser


def parse_capture_options(parser, argv=None):
    """Validate exposure options before opening hardware or creating files."""
    options = parser.parse_args(argv)
    if options.gain is not None and options.shutter is None:
        parser.error("--gain requires --shutter")
    if options.exposure_sweep and options.shutter is not None:
        parser.error("--exposure-sweep cannot be combined with --shutter")
    return options
