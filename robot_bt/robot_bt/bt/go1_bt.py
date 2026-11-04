"""Behaviour tree for the Unitree Go1 (hand gesture control).

The Go1 is driven by the unitree_ros driver (https://github.com/snt-arg/unitree_ros),
which listens to geometry_msgs/Twist on /cmd_vel and std_msgs/Empty on
/stand_up and /stand_down. Connection handling and low-battery shutdown are
done by the driver itself, so this tree only handles plugin selection.

Plugin selection is done with the keyboard of the control station
(published on /key_pressed):
    - `h` : hand gesture control
    - `m` / `s` : manual control (hand control disabled, robot stopped)

This is how the tree looks like:

[-] Go1BT
    --> RemoteOperator
    [o] Plugins
        {-} HandGesturesControl
            --> CanRunHandGestures
            --> HandGesturesPlugin
            --> Go1GesturesInterpreter
        --> Idle (manual mode)
"""

import py_trees
from rclpy.node import Node

from robot_bt.behaviours.shared.actions import PluginClient, RemoteOperator
from robot_bt.behaviours.shared.conditions.can_run_plugin import CanRunPlugin
from robot_bt.behaviours.go1.actions import Go1GesturesInterpreterAction


class Go1BT(py_trees.composites.Sequence):
    def __init__(self, node: Node):
        super().__init__("Go1BT", memory=False)
        self.node = node
        self.build_tree()

    def setup(self):  # type: ignore
        self.plugins_blackboard = py_trees.blackboard.Client(name="PluginsBlackboard")
        self.plugins_blackboard.register_key(
            "selected_plugin", access=py_trees.common.Access.WRITE
        )
        # Start in manual mode: press `h` in the control station to enable hand control
        self.plugins_blackboard.selected_plugin = ""

    def build_tree(self):
        remote_operator = RemoteOperator("RemoteOperator", self.node)

        plugins = py_trees.composites.Selector(
            "Plugins",
            memory=False,
            children=[
                py_trees.composites.Sequence(
                    "HandGesturesControl",
                    memory=False,
                    children=[
                        CanRunPlugin("CanRunHandGestures", "landmark_detector_node"),
                        PluginClient(
                            "HandGesturesPlugin", "landmark_detector_node", self.node
                        ),
                        Go1GesturesInterpreterAction(
                            "Go1GesturesInterpreter", self.node
                        ),
                    ],
                ),
                py_trees.behaviours.Success("Idle"),
            ],
        )

        self.add_children([remote_operator, plugins])


def bootstrap(ros_node: Node) -> py_trees.behaviour.Behaviour:
    return Go1BT(ros_node)
