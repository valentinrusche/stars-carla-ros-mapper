#!/usr/bin/env python
#
# Copyright (c) 2024 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future


class AsyncServiceClient(Node):

    def __init__(self, node_name: str, message_type, topic_name: str, callback_group, timeout_sec: float = 10.0) -> None:
        # Create a client
        super().__init__(node_name)
        self.timeout: float = timeout_sec
        self.client: Client = self.create_client(srv_type=message_type, srv_name=topic_name, callback_group=callback_group)

        # Check if the a service is available
        while not self.client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().info(message="Waiting for available service.")

        self.message_type = message_type

        self.request = self.message_type.Request()

    def send_request(self):
        self.response: Future = self.client.call_async(request=self.request)
        rclpy.spin_until_future_complete(node=self, future=self.response)
        return self.response.result()