#!/usr/bin/env python3
"""MAVLink bridge skeleton for PX4 boat SITL and NVIDIA Isaac Sim.

The script is intentionally usable as a standalone smoke test: it connects to
PX4's simulator_mavlink TCP endpoint, sends simple HIL sensor/GPS data, and
prints actuator commands. In Isaac Sim, feed BoatState from a physics callback
and apply BoatActuatorCommand to the boat thrusters.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

try:
    from pymavlink import mavutil
except ImportError as exc:
    raise SystemExit("pymavlink is required: pip3 install --user pymavlink") from exc


MAV_COMP_ID_SIMULATOR = 250
HIL_SENSOR_UPDATED_XACC = 1 << 0
HIL_SENSOR_UPDATED_YACC = 1 << 1
HIL_SENSOR_UPDATED_ZACC = 1 << 2
HIL_SENSOR_UPDATED_XGYRO = 1 << 3
HIL_SENSOR_UPDATED_YGYRO = 1 << 4
HIL_SENSOR_UPDATED_ZGYRO = 1 << 5
HIL_SENSOR_UPDATED_XMAG = 1 << 6
HIL_SENSOR_UPDATED_YMAG = 1 << 7
HIL_SENSOR_UPDATED_ZMAG = 1 << 8
HIL_SENSOR_UPDATED_ABS_PRESSURE = 1 << 9
HIL_SENSOR_UPDATED_DIFF_PRESSURE = 1 << 10
HIL_SENSOR_UPDATED_PRESSURE_ALT = 1 << 11
HIL_SENSOR_UPDATED_TEMPERATURE = 1 << 12


@dataclass
class BoatState:
    timestamp_us: int
    lat_deg: float = 47.397742
    lon_deg: float = 8.545594
    alt_m: float = 488.0
    vel_n_m_s: float = 0.0
    vel_e_m_s: float = 0.0
    vel_d_m_s: float = 0.0
    heading_rad: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    xacc_m_s2: float = 0.0
    yacc_m_s2: float = 0.0
    zacc_m_s2: float = -9.80665


@dataclass
class BoatActuatorCommand:
    timestamp_us: int = 0
    armed: bool = False
    right: float = 0.0
    left: float = 0.0


def now_us() -> int:
    return time.time_ns() // 1000


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def open_px4_connection(host: str, port: int, listen: bool):
    prefix = "tcpin" if listen else "tcp"

    return mavutil.mavlink_connection(
        f"{prefix}:{host}:{port}",
        source_system=1,
        source_component=MAV_COMP_ID_SIMULATOR,
        autoreconnect=True,
    )


def send_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_hil_sensor(master, state: BoatState) -> None:
    fields_updated = (
        HIL_SENSOR_UPDATED_XACC
        | HIL_SENSOR_UPDATED_YACC
        | HIL_SENSOR_UPDATED_ZACC
        | HIL_SENSOR_UPDATED_XGYRO
        | HIL_SENSOR_UPDATED_YGYRO
        | HIL_SENSOR_UPDATED_ZGYRO
        | HIL_SENSOR_UPDATED_XMAG
        | HIL_SENSOR_UPDATED_YMAG
        | HIL_SENSOR_UPDATED_ZMAG
        | HIL_SENSOR_UPDATED_ABS_PRESSURE
        | HIL_SENSOR_UPDATED_DIFF_PRESSURE
        | HIL_SENSOR_UPDATED_PRESSURE_ALT
        | HIL_SENSOR_UPDATED_TEMPERATURE
    )

    master.mav.hil_sensor_send(
        state.timestamp_us,
        state.xacc_m_s2,
        state.yacc_m_s2,
        state.zacc_m_s2,
        state.roll_rate_rad_s,
        state.pitch_rate_rad_s,
        state.yaw_rate_rad_s,
        0.21523,
        0.01073,
        0.42741,
        1013.25,
        0.0,
        state.alt_m,
        20.0,
        fields_updated,
    )


def send_hil_gps(master, state: BoatState) -> None:
    horizontal_speed = math.hypot(state.vel_n_m_s, state.vel_e_m_s)
    heading_cdeg = int(math.degrees(state.heading_rad) * 100.0) % 36000

    master.mav.hil_gps_send(
        state.timestamp_us,
        3,
        int(state.lat_deg * 1e7),
        int(state.lon_deg * 1e7),
        int(state.alt_m * 1000.0),
        100,
        150,
        int(horizontal_speed * 100.0),
        int(state.vel_n_m_s * 100.0),
        int(state.vel_e_m_s * 100.0),
        int(state.vel_d_m_s * 100.0),
        heading_cdeg,
        12,
    )


def receive_actuators(master, command: BoatActuatorCommand) -> BoatActuatorCommand:
    while True:
        msg = master.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)

        if msg is None:
            return command

        controls = list(msg.controls)
        armed = bool(msg.mode & 128)
        command = BoatActuatorCommand(
            timestamp_us=msg.time_usec,
            armed=armed,
            right=clamp(float(controls[0]) if len(controls) > 0 else 0.0, -1.0, 1.0),
            left=clamp(float(controls[1]) if len(controls) > 1 else 0.0, -1.0, 1.0),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="PX4 simulator MAVLink TCP host")
    parser.add_argument("--port", type=int, default=4560, help="PX4 simulator MAVLink TCP port")
    parser.add_argument(
        "--connect",
        action="store_true",
        help="connect to PX4 as a TCP client instead of listening for PX4",
    )
    parser.add_argument("--sensor-rate", type=float, default=250.0, help="HIL_SENSOR rate in Hz")
    parser.add_argument("--gps-rate", type=float, default=5.0, help="HIL_GPS rate in Hz")
    parser.add_argument("--print-rate", type=float, default=2.0, help="Actuator print rate in Hz")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    master = open_px4_connection(args.host, args.port, listen=not args.connect)
    command = BoatActuatorCommand()

    sensor_period = 1.0 / args.sensor_rate
    gps_period = 1.0 / args.gps_rate
    print_period = 1.0 / args.print_rate if args.print_rate > 0.0 else 0.0

    next_sensor = 0.0
    next_gps = 0.0
    next_heartbeat = 0.0
    next_print = 0.0

    mode = "connecting to" if args.connect else "listening for"
    print(f"{mode} PX4 simulator MAVLink at tcp:{args.host}:{args.port}")

    while True:
        now = time.monotonic()
        state = BoatState(timestamp_us=now_us())
        command = receive_actuators(master, command)

        if now >= next_heartbeat:
            send_heartbeat(master)
            next_heartbeat = now + 1.0

        if now >= next_sensor:
            send_hil_sensor(master, state)
            next_sensor = now + sensor_period

        if now >= next_gps:
            send_hil_gps(master, state)
            next_gps = now + gps_period

        if print_period > 0.0 and now >= next_print:
            print(
                "actuator armed={} right={:.3f} left={:.3f}".format(
                    command.armed, command.right, command.left
                )
            )
            next_print = now + print_period

        time.sleep(0.001)


if __name__ == "__main__":
    main()
