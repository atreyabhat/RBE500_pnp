import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import os
import ast
import time
from ros2_grasping.action import Attacher


class ATTACHERclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('Attacher_client')
        self._action_client = ActionClient(self, Attacher, 'Attacher')
        # 2. Wait for ATTACHER server to be available:
        print("Waiting for ATTACHER action server to be available...")
        self._action_client.wait_for_server()
        print("Attacher ACTION SERVER detected.")
    
    def send_goal(self, object, endeffector):
        # 1. Assign variables:
        goal_msg = Attacher.Goal()
        goal_msg.object = object
        goal_msg.endeffector = endeffector
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)


# 12. DETACHER - Publisher:
class DetacherPUB(Node):
    
    def __init__(self):
        # Declare NODE:
        super().__init__("ros2_PUBLISHER")
        # Declare PUBLISHER:
        self.publisher_ = self.create_publisher(String, "ros2_Detach", 5) #(msgType, TopicName, QueueSize)

    def publish_detach(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)


class PicknPlace(Node):
    def __init__(self):
        super().__init__('pnp_execute_node')
        self.get_logger().info('Pick and place node started')

        self.attach_client = ATTACHERclient()
        self.detach_client = DetacherPUB()

    def execute_action(self, trigger):
        if trigger['action'] == 'Attach':
            self.get_logger().info(str(trigger['value']))

            obj = trigger['value']['object']
            ee = trigger['value']['endeffector']
            
            self.attach_client.send_goal(obj, ee)
            rclpy.spin_once(self.attach_client)

            self.get_logger().info("Object ATTACHED successfully.")

        elif trigger['action'] == 'Detach':
            self.get_logger().info(str(trigger['value']))

            obj = trigger['value']['object']
            
            msg = String()
            msg.data = "True"

            t_end = time.time() + 1
            while time.time() < t_end:
                self.detach_client.publish_detach(msg.data)  # Publish repeatedly for a second

            self.get_logger().info("Object DETACHED successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = PicknPlace()
    
    # Example trigger format to execute actions
    trigger = {
        'action': 'Attach',
        'value': {
            'object': 'red_box',
            'endeffector': 'gripper_left_link'
        }
    }

    detech_trigger = {
        'action': 'Detach',
        'value': {
            'object': 'red_box'
        }
    }
    
    node.execute_action(trigger)
    # time.sleep(3)
    # node.execute_action(detech_trigger)
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
