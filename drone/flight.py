#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy

from drone import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State


def log(msg):
    rospy.loginfo(f"[FLIGHT] {msg}")
    print(f"[FLIGHT] {msg}")


class Flight:
    def __init__(self):
        rospy.init_node("takeoff_wait_land_verbose", anonymous=True)

        self.state = None
        rospy.Subscriber("/mavros/state", State, self._cb_state)

        log("waiting for services...")

        # drone services (Clover/drone stack)
        rospy.wait_for_service("get_telemetry")
        rospy.wait_for_service("navigate")
        rospy.wait_for_service("land")

        self.get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy("navigate", srv.Navigate)
        self.land = rospy.ServiceProxy("land", Trigger)

        # mavros services
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        log("services are ready")

    def _cb_state(self, s: State):
        self.state = s

    def wait_state(self, timeout=10.0):
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < timeout:
            if self.state is not None and self.state.connected:
                return True
            rospy.sleep(0.05)
        return False

    def wait_mode(self, mode: str, timeout=10.0):
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < timeout:
            if self.state and self.state.mode == mode:
                return True
            rospy.sleep(0.05)
        return False

    def wait_armed(self, armed: bool, timeout=10.0):
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < timeout:
            if self.state and self.state.armed == armed:
                return True
            rospy.sleep(0.05)
        return False

    def set_mode_verbose(self, mode: str, timeout=10.0):
        if not self.state:
            raise RuntimeError("No mavros/state yet")

        if self.state.mode == mode:
            log(f"mode already {mode}")
            return

        log(f"setting mode -> {mode}")
        res = self.set_mode(0, mode)
        log(f"set_mode result: mode_sent={res.mode_sent}")

        if not self.wait_mode(mode, timeout):
            cur = self.state.mode if self.state else "unknown"
            raise RuntimeError(f"Mode switch timed out: want={mode}, current={cur}")

        log(f"✅ mode is now {mode}")

    def arm_verbose(self, timeout=10.0):
        if not self.state:
            raise RuntimeError("No mavros/state yet")

        if self.state.armed:
            log("already armed")
            return

        log("arming...")
        res = self.arming(True)
        log(f"arming ack: success={res.success} result={res.result}")

        if not self.wait_armed(True, timeout):
            raise RuntimeError("Arming timed out (armed flag did not become True)")

        log("✅ armed=True")

    def ensure_guided_nogps_then_arm(self):
        """
        Важно для ArduPilot indoor:
        1) GUIDED_NOGPS ДО arm
        2) LAND не armable -> сначала уходим из LAND
        """
        if not self.wait_state(10.0):
            raise RuntimeError("No FCU connection (/mavros/state not connected)")

        log(f"mavros: connected={self.state.connected} mode={self.state.mode} armed={self.state.armed}")

        # Если вдруг мы в LAND — это не armable
        if self.state.mode == "LAND":
            log("FCU is in LAND (not armable). Switching to GUIDED_NOGPS first...")

        # Ставим GUIDED_NOGPS ДО arm (как ты требуешь)
        self.set_mode_verbose("GUIDED", timeout=10.0)

        # Армим
        self.arm_verbose(timeout=10.0)

    def navigate_wait_verbose(self, x=0.0, y=0.0, z=0.0,
                             speed=0.5, frame_id="body",
                             yaw=float("nan"),
                             tolerance=0.2,
                             timeout=30.0,
                             auto_arm=False):
        """
        auto_arm=False — потому что мы уже сделали GUIDED_NOGPS -> arm заранее.
        """
        log(f"navigate(): x={x} y={y} z={z} speed={speed} frame={frame_id} auto_arm={auto_arm}")

        res = self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        if not res.success:
            log(f"❌ navigate rejected: {res.message}")
            raise RuntimeError(res.message)

        log("navigate accepted; waiting for target...")

        t0 = time.time()
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id="navigate_target")
            dist = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)

            log(f"target error: x={telem.x:.2f} y={telem.y:.2f} z={telem.z:.2f} | dist={dist:.2f} m")

            if dist < tolerance:
                log(f"✅ target reached in {time.time() - t0:.1f} s")
                return

            if (time.time() - t0) > timeout:
                raise RuntimeError(f"Navigate timed out after {timeout:.1f} s")

            rospy.sleep(0.2)

    def land_wait_verbose(self, timeout=60.0):
        log("sending LAND command via drone/land ...")
        self.land()

        t0 = time.time()
        while not rospy.is_shutdown():
            telem = self.get_telemetry()
            log(f"landing... armed={telem.armed} mode={telem.mode} z={telem.z:.2f}")

            if not telem.armed:
                log("✅ landed and disarmed")
                return

            if (time.time() - t0) > timeout:
                raise RuntimeError(f"Land timed out after {timeout:.1f} s")

            rospy.sleep(0.3)


def main():
    f = Flight()

    # 1) GUIDED_NOGPS -> ARM (до navigate)
    log("==== PREPARE (GUIDED_NOGPS -> ARM) ====")
    f.ensure_guided_nogps_then_arm()

    # 2) TAKEOFF: 1 m вверх в body
    log("==== TAKEOFF to 1.0 m ====")
    f.navigate_wait_verbose(z=+1.0, speed=0.5, frame_id="body", auto_arm=False, timeout=30.0)

    # 3) HOLD
    hold_sec = 5
    log(f"==== HOLD ({hold_sec} sec) ====")
    for i in range(hold_sec):
        telem = f.get_telemetry()
        log(f"hold {i+1}/{hold_sec}: x={telem.x:.2f} y={telem.y:.2f} z={telem.z:.2f} armed={telem.armed} mode={telem.mode}")
        rospy.sleep(1.0)

    # 4) LAND
    log("==== LAND ====")
    f.land_wait_verbose(timeout=90.0)

    log("mission complete ✅")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log(f"💥 exception: {e}")
        raise
