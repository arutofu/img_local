#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandHome, CommandLong

def log(msg: str):
    rospy.loginfo(f"[AP] {msg}")
    print(f"[AP] {msg}")

class APCloverLike:
    def __init__(self):
        rospy.init_node("ap_clover_like", anonymous=True)

        self.state = None
        self.pose = None
        self.statustext = deque(maxlen=50)

        rospy.Subscriber("/mavros/state", State, self._cb_state)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._cb_pose)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self._cb_statustext)

        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=20)

        log("waiting for mavros services...")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/cmd/takeoff")

        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        # optional services
        self.land_srv = None
        self.set_home_srv = None
        self.cmd_long_srv = None

        try:
            rospy.wait_for_service("/mavros/cmd/land", timeout=1.0)
            self.land_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        except Exception:
            pass

        try:
            rospy.wait_for_service("/mavros/cmd/set_home", timeout=1.0)
            self.set_home_srv = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
        except Exception:
            pass

        try:
            rospy.wait_for_service("/mavros/cmd/command", timeout=1.0)
            self.cmd_long_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        except Exception:
            pass

        log("ready.")

    def _cb_state(self, s: State):
        self.state = s

    def _cb_pose(self, p: PoseStamped):
        self.pose = p

    def _cb_statustext(self, st: StatusText):
        self.statustext.append((time.time(), st.severity, st.text))

    def dump_statustext(self, n=12):
        if not self.statustext:
            log("no STATUSTEXT yet")
            return
        log(f"last {min(n,len(self.statustext))} STATUSTEXT (newest last):")
        for ts, sev, txt in list(self.statustext)[-n:]:
            log(f"  [sev={sev}] {txt}")

    def wait_connected(self, timeout=10.0):
        t0 = time.time()
        while not rospy.is_shutdown() and time.time()-t0 < timeout:
            if self.state and self.state.connected:
                return True
            rospy.sleep(0.05)
        return False

    def wait_pose(self, timeout=10.0):
        t0 = time.time()
        while not rospy.is_shutdown() and time.time()-t0 < timeout:
            if self.pose:
                return True
            rospy.sleep(0.05)
        return False

    def set_mode_checked(self, mode: str, timeout=5.0):
        if self.state and self.state.mode == mode:
            log(f"mode already {mode}")
            return True
        log(f"setting mode -> {mode}")
        res = self.set_mode(0, mode)
        log(f"set_mode: mode_sent={res.mode_sent}")
        t0 = time.time()
        while not rospy.is_shutdown() and time.time()-t0 < timeout:
            if self.state and self.state.mode == mode:
                log(f"✅ mode is now {mode}")
                return True
            rospy.sleep(0.05)
        self.dump_statustext()
        return False

    def arm_checked(self, timeout=8.0):
        if self.state and self.state.armed:
            log("already armed")
            return True
        log("arming...")
        res = self.arming(True)
        log(f"arming ack: success={res.success} result={res.result}")
        if not res.success:
            self.dump_statustext()
            return False
        t0 = time.time()
        while not rospy.is_shutdown() and time.time()-t0 < timeout:
            if self.state and self.state.armed:
                log("✅ armed=True")
                return True
            rospy.sleep(0.05)
        self.dump_statustext()
        return False

    def try_set_home_current(self):
        if not self.set_home_srv:
            log("set_home service not available - skip")
            return False
        try:
            # current_gps=True: автопилот попробует выставить home по текущей оценке
            res = self.set_home_srv(True, 0.0, 0.0, 0.0, 0.0)
            log(f"set_home: success={res.success} result={res.result}")
            return res.success
        except Exception as e:
            log(f"set_home exception: {e}")
            return False

    def takeoff(self, alt=1.0):
        log(f"TAKEOFF to {alt:.2f}m (cmd/takeoff)")
        try:
            # lat/lon/yaw лучше не фиксировать (NaN), чтобы не улетать в (0,0)
            res = self.takeoff_srv(
                min_pitch=0.0,
                yaw=math.nan,
                latitude=math.nan,
                longitude=math.nan,
                altitude=float(alt),
            )
            log(f"takeoff ack: success={res.success} result={res.result}")
            if not res.success:
                self.dump_statustext()
            return res.success
        except Exception as e:
            log(f"takeoff exception: {e}")
            self.dump_statustext()
            return False

    def send_body_velocity(self, vx, vy, vz, duration_s, hz=10):
        """
        vx,vy,vz в NED относительно корпуса:
        +vx вперёд, +vy вправо, +vz вниз. (вверх = отрицательный vz)
        Сообщение надо переотправлять регулярно, иначе Copter остановится через ~3 сек. :contentReference[oaicite:8]{index=8}
        """
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        r = rospy.Rate(hz)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time()-t0 < duration_s:
            msg.header.stamp = rospy.Time.now()
            self.sp_pub.publish(msg)
            r.sleep()

        # stop
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        for _ in range(int(hz*0.5)):
            msg.header.stamp = rospy.Time.now()
            self.sp_pub.publish(msg)
            r.sleep()

    def condition_yaw(self, deg, relative=True, yaw_speed=0.0, cw=True):
        if not self.cmd_long_srv:
            log("cmd/command (CommandLong) not available - skip yaw")
            return False
        # MAV_CMD_CONDITION_YAW = 115
        direction = 1.0 if cw else -1.0
        is_relative = 1.0 if relative else 0.0
        try:
            res = self.cmd_long_srv(
                broadcast=False,
                command=115,
                confirmation=0,
                param1=float(deg),
                param2=float(yaw_speed),
                param3=float(direction),
                param4=float(is_relative),
                param5=0.0, param6=0.0, param7=0.0
            )
            log(f"condition_yaw: success={res.success} result={res.result}")
            if not res.success:
                self.dump_statustext()
            return res.success
        except Exception as e:
            log(f"condition_yaw exception: {e}")
            return False

    def land(self):
        if self.land_srv:
            log("LAND via /mavros/cmd/land")
            try:
                res = self.land_srv(0.0, math.nan, math.nan, math.nan, 0.0)
                log(f"land ack: success={res.success} result={res.result}")
                if not res.success:
                    self.dump_statustext()
                return res.success
            except Exception as e:
                log(f"land exception: {e}")

        log("LAND via set_mode LAND")
        ok = self.set_mode_checked("LAND", timeout=5.0)
        if not ok:
            self.dump_statustext()
        return ok

def main():
    ap = APCloverLike()

    if not ap.wait_connected(10.0):
        raise RuntimeError("No FCU connection")
    if not ap.wait_pose(10.0):
        raise RuntimeError("No local_position/pose (EKF not publishing local pose?)")

    log(f"connected=True mode={ap.state.mode} armed={ap.state.armed}")

    # GUIDED -> ARM
    if not ap.set_mode_checked("GUIDED"):
        raise RuntimeError("Failed to set GUIDED")
    if not ap.arm_checked():
        raise RuntimeError("Arming failed (see STATUSTEXT)")

    # try set home (helpful for no-GPS setups)
    ap.try_set_home_current()

    # TAKEOFF (must be before movement setpoints in most cases) :contentReference[oaicite:9]{index=9}
    if not ap.takeoff(alt=1.0):
        raise RuntimeError("Takeoff rejected (see STATUSTEXT)")

    log("HOLD 3s")
    rospy.sleep(3.0)

    log("FORWARD 1m (0.3 m/s => ~3.3s)")
    ap.send_body_velocity(vx=0.3, vy=0.0, vz=0.0, duration_s=3.4, hz=10)

    log("YAW +90deg")
    ap.condition_yaw(90, relative=True)
    rospy.sleep(2.0)

    log("FORWARD 1m again")
    ap.send_body_velocity(vx=0.3, vy=0.0, vz=0.0, duration_s=3.4, hz=10)

    log("LAND")
    ap.land()

    log("done ✅")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log(f"💥 exception: {e}")
        raise
