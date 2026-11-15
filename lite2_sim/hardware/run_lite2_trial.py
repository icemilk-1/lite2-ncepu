"""Run the supervised 0.5 m trial and restore manual mode on exit."""
import argparse
from datetime import datetime
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import signal

sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from vendor_protocol import velocity_packets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="trial_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--phase", choices=("pulse", "mission"), default="pulse")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--pulse-duration", type=float, choices=(0.2, 2.0), default=0.2)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    if args.phase == "mission":
        import json
        if args.calibration is None:
            parser.error("--calibration must reference the accepted pulse.json")
        calibration = json.loads(args.calibration.read_text())
        if (calibration.get("error") or not calibration.get("feedback_seen")
                or not calibration.get("feedback_zero")
                or calibration.get("stopping_position_span_m", 1.0) >= 0.015):
            parser.error("Calibration pulse has not passed")
    rospy.init_node("airace_trial_supervisor", anonymous=True)
    last_bridge = [0.0]

    def execution(message):
        import json
        report = json.loads(message.data)
        last_bridge[0] = report["received_monotonic"]

    rospy.Subscriber("/airace/execution", String, execution, queue_size=1)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ("192.168.1.120", 43893)
    child = None
    recorder = None
    record_log = (output / "rosbag.log").open("w")
    process_log = (output / "mission.log").open("w")
    try:
        topics = ["/camera/aligned_depth_to_color/image_raw", "/camera/color/camera_info",
                  "/airace/raw_packet", "/airace/state", "/airace/cmd_vel",
                  "/airace/depth_received", "/airace/execution", "/tf", "/tf_static"]
        recorder = subprocess.Popen(["/opt/ros/melodic/lib/rosbag/record", "-O",
                                     str(output / "inputs.bag")] + topics,
                                    stdout=record_log, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError("ROS bag recorder failed to start")
        initial_execution = rospy.wait_for_message("/airace/execution", String, timeout=3)
        execution(initial_execution)
        rospy.wait_for_service("/airace/prepare", timeout=5)
        response = rospy.ServiceProxy("/airace/prepare", Trigger)()
        if not response.success:
            raise RuntimeError(response.message)
        child = subprocess.Popen([sys.executable, "-u",
                                  str(Path(__file__).with_name("lite2_live.py")),
                                  args.phase, "--output", str(output),
                                  "--pulse-duration", str(args.pulse_duration)],
                                 stdout=process_log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 45
        while child.poll() is None:
            if (rospy.is_shutdown() or time.monotonic() > deadline
                    or time.monotonic() - last_bridge[0] > 0.3
                    or recorder.poll() is not None):
                raise RuntimeError("Supervisor lost the execution bridge, recorder or task deadline")
            time.sleep(0.02)
        return child.returncode
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
        for _ in range(5):
            for packet in velocity_packets(0.0, 0.0):
                sender.sendto(packet, address)
            time.sleep(0.02)
        sender.sendto(struct.pack("<3i", 0x21010C02, 0, 0), address)
        sender.sendto(struct.pack("<3i", 0x21010D05, 0, 0), address)
        try:
            rospy.wait_for_service("/airace/disarm", timeout=1)
            rospy.ServiceProxy("/airace/disarm", Trigger)()
        except (rospy.ROSException, rospy.ServiceException):
            pass
        if child is not None and child.poll() is None:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait()
        record_log.close()
        process_log.close()
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
