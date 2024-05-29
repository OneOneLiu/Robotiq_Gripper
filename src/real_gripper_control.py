#!/usr/bin/env python3
import rospy
import argparse
import tkinter as tk
from tkinter import ttk

from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

def parse_args():
    parser = argparse.ArgumentParser(description="Control RViz and Real Robotiq Gripper")
    parser.add_argument("--enable_rviz", type=int, default=1, help="Enable RViz visualization (default: True)")
    args, unknown = parser.parse_known_args() 

    return args

class GripperControl:
    def __init__(self, enable_rviz=True):
        """Initialize the application with a slider to control the gripper."""
        # Initialize ROS node
        rospy.init_node('robotiq_gripper_controller', anonymous=True)
        
        # Initialize ROS publisher
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        self.enable_rviz = enable_rviz
        if self.enable_rviz:
            self.rviz_publisher = rospy.Publisher('/gripper/joint_states', JointState, queue_size=10)

        # Subscribe to the real gripper's input to reflect its state in RViz
        rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.rviz_callback)

        # Send reset and activate command
        self.reset_gripper()
        self.activate_gripper()
    
    def reset_gripper(self):
        """Reset the gripper."""
        reset_command = outputMsg.Robotiq2FGripper_robot_output(rACT=0)
        self.pub.publish(reset_command)
        rospy.sleep(1)  # Ensure the command has time to be processed

    def activate_gripper(self):
        """Activate the gripper."""
        activate_command = outputMsg.Robotiq2FGripper_robot_output(rACT=1, rGTO=1, rSP=255, rFR=150)
        self.pub.publish(activate_command)
        rospy.sleep(1)  # Ensure the command has time to be processed

    def control_gripper(self, val):
        """Control the gripper position."""
        command = outputMsg.Robotiq2FGripper_robot_output(rACT=1, rGTO=1, rSP=255, rFR=150, rPR=int(float(val)))
        self.pub.publish(command)

    def rviz_callback(self, data):
        """Reflect the gripper state in RViz."""
        # Calculate the corresponding RViz position in the range of 0 to 0.8
        position = (data.gPO / 255.0) * 0.8
        
        if self.enable_rviz:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = [
                'robotiq_85_left_knuckle_joint'
            ]
            # Set the position for the joints
            joint_state.position = [position]
            self.rviz_publisher.publish(joint_state)

def main():
    # Parse command-line arguments
    args = parse_args()

    # Initialize the gripper control application
    gripper = GripperControl(enable_rviz=args.enable_rviz)

    # Demo: create slider to control the gripper
    master = tk.Tk()
    master.title("Robotiq Gripper Control")
    slider = ttk.Scale(master, from_=0, to=255, orient="horizontal", command=gripper.control_gripper)
    slider.pack(fill=tk.X, expand=True)
    master.mainloop()

if __name__ == '__main__':
    main()