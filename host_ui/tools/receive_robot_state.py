"""Receive factory-forwarded UDP state and publish ROS telemetry.

Run on the perception host with Python 3 and the ROS1 environment sourced.
The protocol directory supplies the deployed vendor_protocol.py decoder.
"""
import argparse
import json
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol-dir", required=True)
    args = parser.parse_args()
    sys.path.insert(0, args.protocol_dir)
    sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
    from vendor_protocol import decode_state, decode_command
    import rospy
    from std_msgs.msg import String

    rospy.init_node("airace_state_receiver", anonymous=False)
    states = rospy.Publisher("/airace/state", String, queue_size=2)
    raw = rospy.Publisher("/airace/raw_packet", String, queue_size=128)
    feedback = {"joystick": [0.0, 0.0, 0.0], "goal_velocity": [0.0, 0.0, 0.0]}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 43898))
        receiver.settimeout(0.2)
        while not rospy.is_shutdown():
            try:
                packet, address = receiver.recvfrom(4096)
            except socket.timeout:
                continue
            if address[0] != "127.0.0.1":
                continue
            stamp = time.monotonic()
            raw.publish(String(data=json.dumps({"received_monotonic": stamp,
                                               "packet_hex": packet.hex()})))
            command = decode_command(packet)
            if command is not None:
                feedback.update(command, command_received=stamp)
                continue
            state = decode_state(packet)
            if state is not None:
                state.update(feedback, received=stamp)
                states.publish(String(data=json.dumps(state)))


if __name__ == "__main__":
    main()
