#!/usr/bin/env python3
"""
mavsdk_arm.py

Replaces the fixed-delay `arm_and_offboard()` bash function.

Root cause it fixes:
    EKF2 yaw/heading takes a variable amount of time to converge after
    boot (magnetometer init, gyro bias estimation, etc). The old bash
    approach fired the arm+offboard service calls at a hardcoded
    `sleep 40`, which raced EKF2 convergence and failed with
    "no heading reference" / success=False, result=1.

Fix:
    Poll PX4's own `health` telemetry (via MAVSDK) until it reports
    `is_armable`, which PX4 only sets true once EKF2 has a valid
    global position AND a valid heading. Only then do we arm and
    switch to OFFBOARD. We also stream setpoints *before* arming
    (PX4 OFFBOARD requires a setpoint stream already running or the
    mode switch is rejected), and keep streaming after to prevent
    the offboard failsafe from tripping while navigators spin up.

Usage:
    python3 mavsdk_arm.py --port 14550 --uav_id 0
    python3 mavsdk_arm.py --port 14560 --uav_id 1

Notes on ports:
    MAVSDK connects on its own UDP endpoint, separate from the MAVROS
    GCS-side connection. This does NOT conflict with MAVROS because
    PX4 (via mavlink_router / multiple mavlink instances) can serve
    the same telemetry stream to more than one GCS-type endpoint
    simultaneously, as already confirmed in earlier testing.
"""

import argparse
import asyncio
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ---- Tunables -------------------------------------------------------------

HEALTH_POLL_INTERVAL_S = 1.0
HEALTH_TIMEOUT_S = 90.0          # give up if EKF never converges (was silent-hang before)
SETPOINT_RATE_HZ = 20.0
PRE_ARM_SETPOINT_COUNT = 10      # PX4 wants a short setpoint stream before accepting OFFBOARD
POST_ARM_STREAM_S = None        # None = stream forever (script stays alive as a background service)


def log(uav_id: int, msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] [uav{uav_id}] {msg}", flush=True)


async def wait_until_armable(drone: System, uav_id: int) -> bool:
    """
    Poll health until PX4 itself reports is_armable == True.
    This is the actual fix: it removes the fixed sleep() race against
    EKF2 convergence and instead waits for the specific precondition
    ("no heading reference") that was blocking arming.
    """
    start = time.monotonic()
    last_state = None

    async for health in drone.telemetry.health():
        elapsed = time.monotonic() - start

        state = (
            health.is_global_position_ok,
            health.is_home_position_ok,
            health.is_armable,
        )
        if state != last_state:
            log(
                uav_id,
                f"health: global_pos_ok={health.is_global_position_ok} "
                f"home_ok={health.is_home_position_ok} "
                f"armable={health.is_armable} "
                f"(t={elapsed:.1f}s)",
            )
            last_state = state

        if health.is_armable:
            log(uav_id, f"is_armable=True after {elapsed:.1f}s — proceeding to arm")
            return True

        if elapsed > HEALTH_TIMEOUT_S:
            log(uav_id, f"TIMEOUT after {HEALTH_TIMEOUT_S}s waiting for is_armable — aborting")
            return False

    return False


async def stream_zero_setpoints(drone: System, uav_id: int, count: int) -> None:
    """Prime the OFFBOARD setpoint stream. PX4 rejects the OFFBOARD mode
    switch if it hasn't been receiving setpoints already, so we send a
    handful of zero-velocity setpoints first."""
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    for _ in range(count):
        await drone.offboard.set_velocity_body(zero)
        await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)
    log(uav_id, f"streamed {count} priming setpoints")


async def arm_and_offboard(drone: System, uav_id: int) -> bool:
    log(uav_id, "arming...")
    try:
        await drone.action.arm()
    except Exception as e:
        log(uav_id, f"arm() failed: {e}")
        return False
    log(uav_id, "armed OK")

    log(uav_id, "switching to OFFBOARD...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        log(uav_id, f"OFFBOARD start failed: {e._result.result}")
        return False
    log(uav_id, "OFFBOARD active")
    return True


async def keep_stream_alive(drone: System, uav_id: int, duration_s):
    """Keep sending setpoints so PX4's offboard failsafe doesn't trip
    while the real navigator node (APF/VFH+/RRT*) spins up and takes
    over publishing setpoints on its own topic/interface."""
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    start = time.monotonic()
    while duration_s is None or (time.monotonic() - start) < duration_s:
        await drone.offboard.set_velocity_body(zero)
        await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)


async def main(port: int, uav_id: int, system_address: str) -> int:
    drone = System(port=port)
    log(uav_id, f"connecting via {system_address} (mavsdk port {port})...")
    await drone.connect(system_address=system_address)

    log(uav_id, "waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            log(uav_id, "connected")
            break

    ok = await wait_until_armable(drone, uav_id)
    if not ok:
        log(uav_id, "giving up: PX4 never reported is_armable")
        return 1

    await stream_zero_setpoints(drone, uav_id, PRE_ARM_SETPOINT_COUNT)

    ok = await arm_and_offboard(drone, uav_id)
    if not ok:
        return 1

    log(uav_id, "entering keep-alive setpoint stream (Ctrl+C / SIGTERM to stop)")
    await keep_stream_alive(drone, uav_id, POST_ARM_STREAM_S)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True, help="MAVSDK local UDP port, e.g. 14550")
    parser.add_argument("--uav_id", type=int, required=True, help="UAV index for logging (0, 1, ...)")
    parser.add_argument(
        "--system_address",
        type=str,
        default=None,
        help="Override MAVSDK system_address. Defaults to udp://:<port>",
    )
    args = parser.parse_args()

    system_address = args.system_address or f"udp://:{args.port}"

    try:
        exit_code = asyncio.run(main(args.port, args.uav_id, system_address))
    except KeyboardInterrupt:
        exit_code = 0
    sys.exit(exit_code)