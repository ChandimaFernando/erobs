"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import numpy as np
import redis
import rclpy
import time
import bluesky.preprocessors as bpp
import bluesky.plan_stubs as bps
from bluesky import RunEngine
from bluesky_adaptive.per_event import adaptive_plan, recommender_factory
from bluesky.protocols import Readable
from ophyd.sim import ABDetector

from bluesky_ros.ophyd_ros import ActionMovable
from pdf_beamtime_interfaces.action import FidPoseControlMsg
# from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class PickPlaceRedisClient(ActionMovable, Readable):
    """Construct a class to build the client message."""

    action_type = FidPoseControlMsg
    # action_type = PickPlaceControlMsg
    parent = None
    current_sample = None

    def construct_goal_mesage(self, *args, **kwargs):
        """Populate the goal msg."""
        # goal_msg = PickPlaceControlMsg.Goal()
        goal_msg = FidPoseControlMsg.Goal()
        goal_msg.sample_id = kwargs.get("sample_id")
        self.current_sample = goal_msg.sample_id
        goal_msg.sample_return = kwargs.get("sample_return")
        goal_msg.inbeam_approach = [x / 180 * math.pi for x in kwargs.get("inbeam_approach")]
        goal_msg.inbeam = [x / 180 * math.pi for x in kwargs.get("inbeam")]
        # goal_msg.pickup_approach = [x / 180 * math.pi for x in kwargs.get("pickup_approach")]
        # goal_msg.pickup = [x / 180 * math.pi for x in kwargs.get("pickup")]
        # goal_msg.place_approach = [x / 180 * math.pi for x in kwargs.get("place_approach")]
        # goal_msg.place = [x / 180 * math.pi for x in kwargs.get("place")]
        self.current_sample = kwargs.get("sample_id")
        return goal_msg

    def feedback_callback(self, feedback_msg):
        """Handle regular feedback and print the completion percentage."""
        feedback: FidPoseControlMsg.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Percentage completed: {math.ceil(feedback.status * 100)} %"
        )

    def get_result_callback(self, future):
        """Handle results at the end of a state transition."""
        return super().get_result_callback(future)

    def read(self):
        return dict(sample_id={'value': self.current_sample, "timestamp": time.time()})

    def describe(self):
        return dict(sample_id={"source": "REDIS", "dtype": "number", "shape": []})

    @property
    def name(self):
        # This is the Readback Value Key associated with this motor
        return "sample_id"


class Agent:
    def __init__(self):
        self.loaded = False
        self.current_sample = None
        self.current_idx = 2
        self.climbing = True
        self.counter = 1

    def next_in_seq(self):
        print(f"experiment counter: {self.counter}")
        if self.climbing:
            self.current_idx += 1
            self.counter += 1
            if self.current_idx == 6:
                self.climbing = False
        else:
            self.current_idx -= 1
            self.counter += 1
            if self.current_idx == 2:
                self.climbing = True

    def tell(self, x, y):
        self.current_sample = f"sample_{x[0]}"

    def tell_many(self, xs, ys):
        for x, y in zip(xs, ys):
            self.tell(x, y)

    def _create_msg(self, sample_name):
        # Read sample ID from the redis server 
        tag_key = redis_client.hget("sample_name_index", sample_name).decode("utf-8")
        tag_id = int(redis_client.hget(tag_key, "id"))
        print(f"current tag id : {tag_id}")
        goal_structure = {
            "inbeam_approach":  [55.10, -58.25, 124.84, -66.72, 52.19, 180.0],
            "inbeam": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
            "sample_id": tag_id,
            "sample_return": self.loaded,
            "place_approach": [55.10, -51.78, 124.84, -73.16, 52.24, 180.0],
            "place": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
            "pickup_approach": [241.41, -59.73, 130.19, -68.36, 99.66, 180.0],
            "pickup": [238.27, -50.99, 106.60, -53.53, 96.54, 180.0]
        }
        return goal_structure

    def _get_starting_msg(self):
        msg = self._create_msg("sample_2")
        self.loaded = True # Artificially set self.loaded True because no `ask` on first sync adaptive loop
        return msg

    def ask(self, batch_size=1):
        if self.loaded:
            msg = self._create_msg(self.current_sample)
            self.loaded = False
        else:
            self.next_in_seq()
            # next_sample = f"sample_{int(np.random.randint(2, 7))}"
            next_sample = f"sample_{self.current_idx}"
            msg = self._create_msg(next_sample)
            self.loaded = True
        # print(msg)
        return [msg]



@bpp.run_decorator()
def test_plan(node, sample_name):
    """Planning sequence goes here."""
    # Read sample ID from the redis server 
    tag_key = redis_client.hget("sample_name_index", sample_name).decode("utf-8")
    tag_id = int(redis_client.hget(tag_key, "id"))
  
    goal_structure = {
        "inbeam_approach": [55.10, -58.25, 124.84, -66.72, 52.19, 180.0],
        "inbeam": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
        "sample_id": tag_id,
        "sample_return": False,
        "place_approach": [55.10, -51.78, 124.84, -73.16, 52.24, 180.0],
        "place": [63.85, -47.04, 98.27, -51.31, 61.00, 180.0],
        "pickup_approach": [241.41, -59.73, 130.19, -68.36, 99.66, 180.0],
        "pickup": [238.27, -50.99, 106.60, -53.53, 96.54, 180.0]
    }

    yield from bps.mv(node, goal_structure)

rclpy.init()
redis_client = redis.Redis(host="192.168.56.1", port=6379, db=0)
pick_place_client_node = PickPlaceRedisClient(node_name="pdf_beamtime_fidpose_client", action_client_name="pdf_beamtime_fidpose_action_server")
pick_place_client_node.kind = "hinted"
ab_det = ABDetector(name="ab_det")
RE = RunEngine({})
agent = Agent()
recommender, queue = recommender_factory(agent, independent_keys=["sample_id"], dependent_keys=["ab_det_a"], max_count=999)
plan = adaptive_plan([ab_det],
                     {pick_place_client_node: agent._get_starting_msg()},
                     to_recommender=recommender,
                     from_recommender=queue)



def initilaize_pickplace_redis():
    """Python main."""
    #  Change the sample name to represent the correct sample to be picked.
    sample_name = "sample_4"
    RE(test_plan(node=pick_place_client_node, sample_name=sample_name))
