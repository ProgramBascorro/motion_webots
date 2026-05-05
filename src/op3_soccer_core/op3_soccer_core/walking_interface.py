import copy
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from op3_walking_module_msgs.msg import WalkingParam
from op3_walking_module_msgs.srv import GetWalkingParam


class WalkingInterface:
    def __init__(self, node: Node,
                 enable_topic: str = "/robotis/enable_ctrl_module",
                 walking_command_topic: str = "/robotis/walking/command",
                 walking_param_topic: str = "/robotis/walking/set_params",
                 action_topic: str = "/robotis/action/page_num",
                 walking_get_param_srv: str = "/robotis/walking/get_params",
                 action_module_name: str = "action_module",
                 walking_module_name: str = "walking_module",
                 base_module_name: str = "base_module") -> None:
        self._node = node
        self._enable_pub = node.create_publisher(String, enable_topic, 10)
        self._walking_cmd_pub = node.create_publisher(String, walking_command_topic, 10)
        self._walking_param_pub = node.create_publisher(WalkingParam, walking_param_topic, 10)
        self._action_pub = node.create_publisher(Int32, action_topic, 10)
        self._param_client = node.create_client(GetWalkingParam, walking_get_param_srv)

        self._action_module_name = action_module_name
        self._walking_module_name = walking_module_name
        self._base_module_name = base_module_name

        self._baseline_params: Optional[WalkingParam] = None
        self._last_fetch_time = 0.0
        self._fetch_retry_sec = 1.0

    def tick(self, now: float) -> None:
        if self._baseline_params is not None:
            return
        if now - self._last_fetch_time < self._fetch_retry_sec:
            return
        if not self._param_client.service_is_ready():
            return
        self._last_fetch_time = now
        request = GetWalkingParam.Request()
        request.get_param = True
        future = self._param_client.call_async(request)
        future.add_done_callback(self._handle_baseline_response)

    def _handle_baseline_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._node.get_logger().warn(f"Failed to get walking params: {exc}")
            return
        if response:
            self._baseline_params = response.parameters
            self._node.get_logger().info("Loaded baseline walking parameters")

    @property
    def baseline_ready(self) -> bool:
        return self._baseline_params is not None

    def enable_module(self, module_name: str) -> None:
        self._enable_pub.publish(String(data=module_name))

    def enable_base(self) -> None:
        self.enable_module(self._base_module_name)

    def enable_action(self) -> None:
        self.enable_module(self._action_module_name)

    def enable_walking(self) -> None:
        self.enable_module(self._walking_module_name)

    def start_walking(self) -> None:
        self._walking_cmd_pub.publish(String(data="start"))

    def stop_walking(self) -> None:
        self._walking_cmd_pub.publish(String(data="stop"))

    def set_walk(self, x: float, y: float, yaw: float,
                 max_x: float, max_y: float, max_yaw: float) -> None:
        if self._baseline_params is None:
            return
        params = copy.copy(self._baseline_params)
        params.x_move_amplitude = float(max(-max_x, min(max_x, x)))
        params.y_move_amplitude = float(max(-max_y, min(max_y, y)))
        params.angle_move_amplitude = float(max(-max_yaw, min(max_yaw, yaw)))
        self._walking_param_pub.publish(params)

    def kick(self, page: int) -> None:
        if page < 0:
            return
        self.enable_action()
        self._action_pub.publish(Int32(data=page))
