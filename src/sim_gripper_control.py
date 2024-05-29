#!/usr/bin/env python3
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk

class GripperControl:
    def __init__(self, master):
        self.master = master
        self.master.title("Gripper Control")

        # ROS initialization
        rospy.init_node('gripper_control_node', anonymous=True)
        self.publisher = rospy.Publisher('/gripper_controller/follow_joint_trajectory/goal', 
                                         FollowJointTrajectoryActionGoal, queue_size=10)

        # GUI setup
        ttk.Label(master, text="Gripper Position:").pack()
        self.slider = ttk.Scale(master, from_=0.0, to=0.8, orient='horizontal', 
                                length=300,  # Make the slider longer
                                command=self.control_gripper)
        self.slider.pack(fill=tk.X, expand=True)  # Add some padding around the slider

    def control_gripper(self, value):
        # Prepare the message
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"] 
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.time_from_start = rospy.Duration(1.0)
        msg.goal.trajectory.points.append(point)
        
        # Publish the message
        self.publisher.publish(msg)
        rospy.loginfo("Sent gripper command: %f", float(value))

def main():
    root = tk.Tk()
    app = GripperControl(root)
    root.mainloop()

if __name__ == '__main__':
    main()