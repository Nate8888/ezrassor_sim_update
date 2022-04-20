"""Execute a ROS node using the arms_driver module.

This node reads messages from a custom topic (external), parses them into
the value that Gazebo expects, and then publishes these messages
to Gazebo (internal). For the arms driver, these messages are
Float64 values that represent the velocity that should be applied to
raising or lowering the arms.

The abstraction done by this node is necessary to be able to switch
between hardware and software; a similarly written hardware node could
listen to the same istructions topic and command actuators, instead of the sim.
"""
import std_msgs.msg
import rclpy

NODE = "arms_driver"
FRONT_ARMS_EXTERNAL_TOPIC = "front_arm_instructions"
FRONT_ARMS_INTERNAL_TOPIC = "arm_front_velocity_controller/command"
BACK_ARMS_EXTERNAL_TOPIC = "back_arm_instructions"
BACK_ARMS_INTERNAL_TOPIC = "arm_back_velocity_controller/command"

QUEUE_SIZE = 10
MAX_ARM_SPEED = 0.75

# Dictionary values set after publishers get created in main()
publishers = {}


def handle_front_arm_movements(data):
    """Move the front arm of the robot per
    the commands encoded in the instruction.
    """
    publishers[FRONT_ARMS_INTERNAL_TOPIC].publish(data.data * MAX_ARM_SPEED)


def handle_back_arm_movements(data):
    """Move the front arm of the robot per
    the commands encoded in the instruction.
    """
    publishers[BACK_ARMS_INTERNAL_TOPIC].publish(data.data * MAX_ARM_SPEED)


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers to Gazebo velocity managers.
        publishers[FRONT_ARMS_INTERNAL_TOPIC] = node.create_publisher(
            std_msgs.msg.Float64, FRONT_ARMS_INTERNAL_TOPIC, QUEUE_SIZE
        )
        publishers[BACK_ARMS_INTERNAL_TOPIC] = node.create_publisher(
            std_msgs.msg.Float64, BACK_ARMS_INTERNAL_TOPIC, QUEUE_SIZE
        )

        # Create subscriptions to listen for specific robot actions from users.
        node.create_subscription(
            std_msgs.msg.Float64,
            FRONT_ARMS_EXTERNAL_TOPIC,
            handle_front_arm_movements,
            QUEUE_SIZE,
        )
        node.create_subscription(
            std_msgs.msg.Float64,
            BACK_ARMS_EXTERNAL_TOPIC,
            handle_front_arm_movements,
            QUEUE_SIZE,
        )

        node.get_logger().info("arms_driver node created successfully")

        # Spin!
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
