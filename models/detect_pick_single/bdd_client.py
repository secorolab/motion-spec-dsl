#!/usr/bin/env python3
"""Send a Behaviour goal and stream the run's exported events until the result arrives.

Usage: python3 bdd_client.py [action_name] [events_channel]
"""

import sys
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from bdd_ros2_interfaces.action import Behaviour
from bdd_ros2_interfaces.msg import Event


class BddClient(Node):
    def __init__(self, action_name, events_channel):
        super().__init__('bdd_client')
        self.result = None
        self.uuid = list(uuid.uuid4().bytes)
        self.create_subscription(
            Event, events_channel,
            lambda m: print(f'[event] {m.uri}  ctx={bytes(m.scenario_context_id.uuid).hex()[:8]}'),
            10)
        self._client = ActionClient(self, Behaviour, action_name)

    def send(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            sys.exit('no action server')
        goal = Behaviour.Goal()
        goal.scenario_context_id.uuid = self.uuid
        print(f'[goal] sending ctx={bytes(self.uuid).hex()[:8]}')
        self._client.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            sys.exit('goal rejected')
        print('[goal] accepted')
        handle.get_result_async().add_done_callback(self._done)

    def _done(self, future):
        result = future.result()
        trinary = {0: 'UNKNOWN', 1: 'TRUE', 2: 'FALSE'}.get(result.result.result.trinary.value)
        print(f'[result] status={result.status} trinary={trinary}')
        self.result = result


def main():
    action_name = sys.argv[1] if len(sys.argv) > 1 else 'pick_place'
    events_channel = sys.argv[2] if len(sys.argv) > 2 else '/bdd/events'
    rclpy.init()
    node = BddClient(action_name, events_channel)
    node.send()
    while rclpy.ok() and node.result is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
