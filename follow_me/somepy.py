#!/usr/bin/env python
# This code is run after the following line is executed
#  roslaunch px4 multi_uav_mavros_sitl.launch


import rospy
import numpy as np
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import ActuatorControl
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

class Follow_People:

	def __init__(self):
		#Set parameters
		self.min_capture_dist = 5  #meters
		rospy.init_node('follow_people_node', anonymous=True)
		self.r = rospy.Rate(10) # Hz
		#Get target locations
		self.uav_cmd_vel = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel_unstamped',
									   Twist, queue_size=1)
		self.uav_actuator = rospy.Publisher('/uav1/mavros/actuator_control',
											ActuatorControl, queue_size=1)
		self.uav_subscriber = rospy.Subscriber('/uav1/mavros/local_position/pose',
										PoseStamped, self.uav_state_cb)
		self.uav_set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
		self.uav_arming = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)

		
	def uav_state_cb(self, msg):
		self.uav_pose = msg.pose

		
	def takeoff(self):
		takeoff_twist = Twist()
		takeoff_twist.linear.x = 0
		takeoff_twist.linear.y = 0
		takeoff_twist.linear.z = 10
		for jj in range(70):
			self.uav_cmd_vel.publish(takeoff_twist)
			self.uav_set_mode(base_mode=0, custom_mode="OFFBOARD")
			self.uav_arming(True)
			self.r.sleep()


	def pitch_camera(self):
		pitch_actuator = ActuatorControl()
		pitch_actuator.group_mix = 2
		pitch_actuator.controls[1] = -0.7
		import pdb; pdb.set_trace()
		for jj in range(20):
			self.uav_actuator.publish(pitch_actuator)
			self.uav_set_mode(base_mode=0, custom_mode="OFFBOARD")
			self.uav_arming(True)
			self.r.sleep()
			

	def move_to_target(self):
		control_twist = Twist()
		control_twist.linear.z = 0
		pointing_vec = self.return_pointing_vector()
		while np.linalg.norm(pointing_vec) > self.min_capture_dist:
			control_twist.linear.x = pointing_vec[0]
			control_twist.linear.y = pointing_vec[1]
			self.uav_cmd_vel.publish(control_twist)
			self.uav_set_mode(base_mode=0, custom_mode="OFFBOARD")
			self.uav_arming(True)
			self.r.sleep()
			pointing_vec = self.return_pointing_vector()

			
	def hover(self):
		hover_twist = Twist()
		hover_twist.linear.x = 0
		hover_twist.linear.y = 0
		hover_twist.linear.z = 0
		while not rospy.is_shutdown():
			self.uav_cmd_vel.publish(hover_twist)
			self.uav_set_mode(base_mode=0, custom_mode="OFFBOARD")
			self.uav_arming(True)
			self.r.sleep()

			
	def return_pointing_vector(self):
		t_x=target_state.pose.position.x
		t_y=target_state.pose.position.y
		d_x=self.uav_pose.position.x
		d_y=self.uav_pose.position.y
		return np.asarray([t_x-d_x,t_y-d_y])
	

class Move_Target:
	def __init__(self):
		pass
	def update_target(self):
		target_state.pose.position.x += -.02
		set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		resp = set_state(target_state)

if __name__=='__main__':
	target_state = ModelState()
	target_state.model_name = 'person_walking'
 	target_state.pose.position.x = -5.09057
	target_state.pose.position.y = -1.93779
	target_state.pose.position.z = 0
	target_state.pose.orientation.x = 0
	target_state.pose.orientation.y = 0
	target_state.pose.orientation.z = -0.6768
	target_state.pose.orientation.w = 0.73617

	follow_people = Follow_People()
	move_target = Move_Target()
	follow_people.takeoff()
	follow_people.move_to_target()
	for _ in range(1000):
		move_target.update_target()
		follow_people.move_to_target()
	# follow_people.pitch_camera()
	follow_people.hover()