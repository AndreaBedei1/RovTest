"""Read-only MAVLink heartbeat probe.

This script opens a pymavlink endpoint and waits for HEARTBEAT messages only.
It does not send MAVLink commands, parameter writes, actuator requests, arm,
disarm, mode changes, or RC overrides.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from pymavlink import mavutil


def _heartbeat_summary(msg: Any) -> str:
    payload = msg.to_dict()
    mav_type = payload.get("type", "")
    autopilot = payload.get("autopilot", "")
    base_mode = payload.get("base_mode", "")
    custom_mode = payload.get("custom_mode", "")
    system_status = payload.get("system_status", "")
    return (
        f"src_system={msg.get_srcSystem()} "
        f"src_component={msg.get_srcComponent()} "
        f"type={mav_type} "
        f"autopilot={autopilot} "
        f"base_mode={base_mode} "
        f"custom_mode={custom_mode} "
        f"system_status={system_status}"
    )


def probe(endpoint: str, timeout_s: float, source_system: int, source_component: int) -> int:
    print(f"endpoint={endpoint}")
    print(f"timeout_s={timeout_s}")
    master = None
    try:
        master = mavutil.mavlink_connection(
            endpoint,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=False,
        )
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        fallback = None
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is None:
                continue
            if getattr(msg, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
                print(f"heartbeat_gcs_ignored: {_heartbeat_summary(msg)}")
                continue
            if fallback is None:
                fallback = msg
            src_comp = msg.get_srcComponent()
            src_autopilot = getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
            print(f"heartbeat_seen: {_heartbeat_summary(msg)}")
            if (
                src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                or src_autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                print("result=AUTOPILOT_HEARTBEAT")
                return 0

        if fallback is not None:
            print(f"result=NON_GCS_HEARTBEAT_ONLY: {_heartbeat_summary(fallback)}")
            return 2
        print("result=NO_HEARTBEAT")
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"result=ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 3
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only MAVLink HEARTBEAT probe.")
    parser.add_argument("endpoint", help="pymavlink endpoint, e.g. udpin:0.0.0.0:14550")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for heartbeat.")
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=0)
    args = parser.parse_args()
    return probe(args.endpoint, args.timeout, args.source_system, args.source_component)


if __name__ == "__main__":
    raise SystemExit(main())
