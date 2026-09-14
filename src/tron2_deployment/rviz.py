"""Review a frozen pregrasp plan on a separate, local ROS visualization master."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.parse import urlsplit
import xmlrpc.client

import numpy as np


DEFAULT_MASTER = "http://127.0.0.1:11331"


def validate_master(uri, profile):
    parsed = urlsplit(uri)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1") or not parsed.port:
        raise ValueError("RViz requires an explicit localhost HTTP visualization master")
    if parsed.port == 11311:
        raise ValueError("use a separate visualization master port, for example 11331")
    camera_uri = profile.get("camera", {}).get("ros_master_uri")
    if camera_uri and parsed.netloc == urlsplit(camera_uri).netloc:
        raise ValueError("RViz must not publish into the camera/robot ROS master")
    return parsed.port


def sample_plan(plan, elapsed_s):
    """Use the executor's exact quintic segment interpolation and timestamps."""
    if plan.get("interpolation") != "quintic_stop":
        raise ValueError("unsupported plan interpolation")
    times = np.asarray(plan["times_s"], dtype=float)
    values = np.column_stack((plan["left_arm_qpos"], plan["right_arm_qpos"], plan["head_qpos"]))
    if times.ndim != 1 or len(times) < 2 or values.shape != (len(times), 16):
        raise ValueError("malformed synchronized plan arrays")
    if not np.isfinite(times).all() or not np.isfinite(values).all() or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError("invalid finite plan timestamps or joints")
    elapsed = float(elapsed_s)
    if not np.isfinite(elapsed):
        raise ValueError("elapsed time must be finite")
    if elapsed <= 0:
        return values[0].copy()
    if elapsed >= times[-1]:
        return values[-1].copy()
    index = int(np.searchsorted(times, elapsed, side="right")) - 1
    fraction = (elapsed - times[index]) / (times[index + 1] - times[index])
    weight = fraction**3 * (10 + fraction * (-15 + 6 * fraction))
    return values[index] + weight * (values[index + 1] - values[index])


def preview_data(profile, plan):
    from .config import profile_fingerprint
    from .kinematics import RobotModel
    from .planning import validate_plan_path
    if plan.get("profile_hash") != profile_fingerprint(profile):
        raise ValueError("plan does not match the current calibration and model profile")
    checks = validate_plan_path(profile, plan)
    names = (profile["robot"]["arm_joint_names"]["left"]
             + profile["robot"]["arm_joint_names"]["right"]
             + profile["robot"]["head_joint_names"])
    model = RobotModel(profile, plan["source"]["observation"])
    paths = {side: [] for side in plan["selected_sides"]}
    # Sample densely in plan time so curved FK paths between IK nodes are visible.
    duration = float(plan["times_s"][-1])
    for elapsed in np.linspace(0, duration, max(2, min(2000, int(duration * 20) + 1))):
        joints = sample_plan(plan, float(elapsed))
        model.set_state(joints[:14], joints[14:])
        poses = model.wrist_poses()
        for side in paths:
            paths[side].append(poses[side][:3])
    return {"joint_names": names, "duration_s": duration, "wrist_paths": paths,
            "checks": checks, "object_pose7": plan["source"]["observation"]["pose7"],
            "object_radius_m": profile["scene"]["object_radius_m"],
            "table_z_m": profile["scene"]["table_z_m"]}


class _Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1
        return connection


def _master_alive(uri):
    try:
        return xmlrpc.client.ServerProxy(uri, transport=_Transport()).getUri("/pregrasp_preview")[0] == 1
    except (OSError, xmlrpc.client.Error):
        return False


def _rviz_config(path, has_robot):
    robot = """    - Class: rviz/RobotModel
      Enabled: true
      Name: Robot
      Robot Description: /pregrasp_preview/robot_description
      Visual Enabled: true
      Collision Enabled: true
""" if has_robot else ""
    path.write_text("""Panels:
  - Class: rviz/Displays
Visualization Manager:
  Class: ""
  Global Options:
    Background Color: 239; 244; 245
    Fixed Frame: base_Link
    Frame Rate: 30
  Displays:
    - Class: rviz/Grid
      Enabled: true
      Name: Grid
      Plane: XY
      Cell Size: 0.1
      Plane Cell Count: 20
    - Class: rviz/MarkerArray
      Enabled: true
      Name: Observed object and pregrasp path
      Marker Topic: /pregrasp_preview/markers
""" + robot + """  Views:
    Current:
      Class: rviz/Orbit
      Distance: 2
      Pitch: 0.45
      Yaw: 0.6
      Focal Point:
        X: 0.3
        Y: 0
        Z: 0
""")


def publish(profile, plan, *, ros_master_uri=DEFAULT_MASTER, gui=True,
            start_master=True, loop=True, rate_hz=30):
    port = validate_master(ros_master_uri, profile)
    if not np.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("rate_hz must be positive")
    data = preview_data(profile, plan)
    urdf = profile.get("robot", {}).get("urdf")
    has_robot = bool(urdf and Path(urdf).is_file())
    if not has_robot and plan["source"]["state"].get("source") == "real":
        raise ValueError("real-plan RViz review requires robot.urdf matching the calibrated robot model")
    # ROS imports are deliberately delayed until this explicit CLI operation.
    os.environ["ROS_MASTER_URI"] = ros_master_uri
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    try:
        import rospy
        from geometry_msgs.msg import Point
        from sensor_msgs.msg import JointState
        from visualization_msgs.msg import Marker, MarkerArray
    except ImportError as error:
        raise RuntimeError("source the ROS 1 environment before running the RViz publisher") from error
    from .geometry import pose_matrix
    from .kinematics import RobotModel
    children = []
    directory = tempfile.TemporaryDirectory(prefix="tron2-rviz-")
    try:
        if not _master_alive(ros_master_uri):
            if not start_master:
                raise RuntimeError(f"visualization ROS master is not running: {ros_master_uri}")
            children.append(subprocess.Popen(["roscore", "-p", str(port)], env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            deadline = time.monotonic() + 10
            while not _master_alive(ros_master_uri):
                if time.monotonic() >= deadline or children[-1].poll() is not None:
                    raise RuntimeError("could not start the separate visualization master")
                time.sleep(.1)
        rospy.init_node("pregrasp_preview", anonymous=True)
        if has_robot:
            rospy.set_param("/pregrasp_preview/robot_description", Path(urdf).read_text())
            children.append(subprocess.Popen(["rosrun", "robot_state_publisher", "robot_state_publisher", "__ns:=/pregrasp_preview", "__name:=model"], env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        else:
            print("MOCK preview: robot.urdf is missing; displaying observed-object/wrist markers and publishing planned joints only.", flush=True)
        joints_pub = rospy.Publisher("/pregrasp_preview/joint_states", JointState, queue_size=2)
        markers_pub = rospy.Publisher("/pregrasp_preview/markers", MarkerArray, queue_size=1, latch=True)
        if gui:
            config = Path(directory.name) / "pregrasp.rviz"
            _rviz_config(config, has_robot)
            children.append(subprocess.Popen(["rviz", "-d", str(config)], env=os.environ.copy()))
        model = RobotModel(profile, plan["source"]["observation"])
        marker_id = 0
        static = []

        def new_marker(kind, namespace, color):
            nonlocal marker_id
            item = Marker(); item.header.frame_id = "base_Link"; item.ns = namespace
            item.id = marker_id; marker_id += 1; item.type = kind; item.action = Marker.ADD
            item.pose.orientation.w = 1; item.color.r, item.color.g, item.color.b, item.color.a = color
            return item

        def axes(pose, namespace, length=.06):
            transform = pose_matrix(pose)
            for axis, color in enumerate(((.9,.2,.2,1),(.2,.7,.3,1),(.2,.4,.9,1))):
                arrow = new_marker(Marker.ARROW, namespace, color)
                begin = transform[:3,3]; end = begin + length * transform[:3,axis]
                arrow.points = [Point(*begin.tolist()), Point(*end.tolist())]
                arrow.scale.x=.004; arrow.scale.y=.009; arrow.scale.z=.012; static.append(arrow)

        obj = new_marker(Marker.SPHERE, "object_collision_envelope", (.18,.55,.65,.25))
        obj.pose.position = Point(*data["object_pose7"][:3]); obj.scale.x=obj.scale.y=obj.scale.z=2*data["object_radius_m"]; static.append(obj)
        axes(data["object_pose7"], "object_axes")
        table = new_marker(Marker.CUBE, "table", (.65,.7,.72,.35)); table.pose.position.z=data["table_z_m"]-.005
        table.scale.x=table.scale.y=3; table.scale.z=.01; static.append(table)
        for side, points in data["wrist_paths"].items():
            color = (.1,.6,.5,1) if side=="left" else (.6,.3,.75,1)
            line = new_marker(Marker.LINE_STRIP, side+"_wrist_path", color); line.scale.x=.004
            line.points=[Point(*point) for point in points]; static.append(line)
            axes(plan["targets"][side]["wrist_pregrasp_pose7_base"], side+"_wrist_goal")
            axes(plan["targets"][side]["tcp_pregrasp_pose7_base"], side+"_tcp_stop", .04)
        label = new_marker(Marker.TEXT_VIEW_FACING, "reviewed_plan", (.2,.3,.35,1)); label.pose.position = Point(*data["object_pose7"][:3]); label.pose.position.z += .2
        label.scale.z=.025; label.text=f"PREGRASP / {plan['plan_id'][:12]} / {data['duration_s']:.2f}s"; static.append(label)
        start = time.monotonic(); rate = rospy.Rate(rate_hz)
        print(f"RViz review plan {plan['plan_id']} on {ros_master_uri}; Ctrl-C closes this preview.", flush=True)
        while not rospy.is_shutdown():
            elapsed = time.monotonic()-start
            if loop:
                elapsed %= data["duration_s"]+1.0
            joints = sample_plan(plan, min(elapsed, data["duration_s"]))
            message = JointState(); message.header.stamp=rospy.Time.now(); message.name=data["joint_names"]; message.position=joints.tolist(); joints_pub.publish(message)
            model.set_state(joints[:14], joints[14:]); poses=model.wrist_poses(); moving=[]
            for offset,side in enumerate(data["wrist_paths"]):
                sphere = Marker(); sphere.header=message.header; sphere.header.frame_id="base_Link"; sphere.ns="current_wrist"; sphere.id=offset; sphere.type=Marker.SPHERE; sphere.action=Marker.ADD; sphere.pose.orientation.w=1
                sphere.pose.position=Point(*poses[side][:3]); sphere.scale.x=sphere.scale.y=sphere.scale.z=.018; sphere.color.r=.9; sphere.color.g=.4; sphere.color.b=.1; sphere.color.a=1; moving.append(sphere)
            for item in static:
                item.header.stamp=message.header.stamp
            markers_pub.publish(MarkerArray(markers=static+moving)); rate.sleep()
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired: child.kill(); child.wait()
        directory.cleanup()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",required=True)
    parser.add_argument("--plan",required=True)
    parser.add_argument("--ros-master-uri",default=DEFAULT_MASTER)
    parser.add_argument("--no-gui",action="store_true")
    parser.add_argument("--no-start-master",action="store_true")
    parser.add_argument("--once",action="store_true",help="play once and hold the final pregrasp pose")
    parser.add_argument("--rate-hz",type=float,default=30)
    args=parser.parse_args(argv)
    from .config import load_profile
    profile=load_profile(args.profile); plan=json.loads(Path(args.plan).read_text())
    publish(profile,plan,ros_master_uri=args.ros_master_uri,gui=not args.no_gui,
            start_master=not args.no_start_master,loop=not args.once,rate_hz=args.rate_hz)


if __name__=="__main__":
    main()
