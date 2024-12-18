"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import redis
import rclpy

import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky.protocols import Readable

from bluesky_ros.ophyd_ros import ActionMovable
from pdf_beamtime_interfaces.action import FidPoseControlMsg
from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class PickPlaceRedisClient(ActionMovable, Readable):
    """Construct a class to build the client message."""

    # action_type = FidPoseControlMsg
    action_type = PickPlaceControlMsg
    parent = None
    current_sample = None

    def construct_goal_mesage(self, *args, **kwargs):
        """Populate the goal msg."""
        goal_msg = PickPlaceControlMsg.Goal()
        # goal_msg = FidPoseControlMsg.Goal()
        # goal_msg.sample_id = kwargs.get("sample_id")
        # self.current_sample =  goal_msg.sample_id
        # goal_msg.sample_return = kwargs.get("sample_return")
        # goal_msg.inbeam_approach = [x / 180 * math.pi for x in kwargs.get("inbeam_approach")]
        # goal_msg.inbeam = [x / 180 * math.pi for x in kwargs.get("inbeam")]
        goal_msg.pickup_approach = [x / 180 * math.pi for x in kwargs.get("pickup_approach")]
        goal_msg.pickup = [x / 180 * math.pi for x in kwargs.get("pickup")]
        goal_msg.place_approach = [x / 180 * math.pi for x in kwargs.get("place_approach")]
        goal_msg.place = [x / 180 * math.pi for x in kwargs.get("place")]
        return goal_msg

    def feedback_callback(self, feedback_msg):
        """Handle regular feedback and print the completion percentage."""
        feedback: PickPlaceControlMsg.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Percentage completed: {0} %".format(math.ceil(feedback.status * 100))
        )

    def get_result_callback(self, future):
        """Handle results at the end of a state transition."""
        return super().get_result_callback(future)
    
    def read(self):
        return dict(sample_id={'value': self.current_sample})
    
    def describe(self):
        return dict(sample_id={"source": "REDIS", "dtype":"number", "shape":[]})

    

def plan(node, goal_structure):
    """Planning sequence goes here."""
    yield from bps.mv(node, goal_structure)


def main(args=None):
    """Python main."""
    rclpy.init(args=args)

    # Change the sample name to represent the correct sample to be picked.
    sample_name = "sample_3"

    # Read sample ID from the redis server
    redis_client = redis.Redis(host="192.168.56.1", port=6379, db=0)
    tag_key = redis_client.hget("sample_name_index", sample_name).decode("utf-8")
    tag_id = int(redis_client.hget(tag_key, "id"))

    node = PickPlaceRedisClient(node_name="pdf_beamtime_fidpose_client", action_client_name="pdf_beamtime_action_server")

    goal_structure = {
        "inbeam_approach": [55.10, -51.78, 124.84, -73.16, 52.24, 180.0],
        "inbeam": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
        "sample_id": tag_id,
        "sample_return": False,
        "place_approach": [55.10, -51.78, 124.84, -73.16, 52.24, 180.0],
        "place": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
        "pickup_approach": [241.41, -59.73, 130.19, -68.36, 99.66, 180.0],
        "pickup": [238.27, -50.99, 106.60, -53.53, 96.54, 180.0]
    }

    RE = RunEngine({})
    RE(plan(node, goal_structure))


if __name__ == "__main__":
    main()
