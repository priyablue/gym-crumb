import gym
import rospy
from gym import error, spaces, utils
from gym.utils import seeding
from gazebo_msgs.srv import GetModelState, GetJointProperties
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from math import radians
import numpy as np

sign = lambda a: (a>0) - (a<0)
module = lambda a: a*sign(a)


class CrumbEnv(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		rospy.init_node('arm_start')
		self.arm = [rospy.Publisher('arm_'+str(i+1)+'_joint/command', Float64, queue_size=10) for i in range(5)]
		self.gripper = rospy.Publisher('gripper_1_joint/command', Float64, queue_size=10)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.resetworld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		self.box_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		self.unpause()
		self.resetworld()
		self.pick(gripper_time = 5.0)

	def _reset(self):
		for i in range(5):
			self.arm[i].publish(0.0)
			rospy.sleep(0.5)
		self.gripper.publish(2.0)
		rospy.sleep(5.0)
		self.resetworld()
		self.pick(gripper_time = 5.0)
		_, state = self.get_state()
		return self.binarizer(state)
		
	def _seed(self, seed = None):
		print('12')

	def _step(self, action):
		"""action = (joint, step)"""
		box_h, _ = self.get_state()
		self.arm[action[0]].publish(action[1])
		rospy.sleep(2.0)
		reward = 0
		new_h, state = self.get_state() 
		reward = (new_h - box_h)//0.1
		if reward < 0:
			reward = reward * 2
		if reward == 0:
			reward = - 1
		done = False
		r = module(state[1]) + module(state[2] - radians(90)) + module(state[3])
		if (r < radians(22.5)):
			done = True
			reward = 10 		
		return self.binarizer(state), reward, done

	def get_state(self):
		return self.box_state('little_box_0', 'link').pose.position.z, [self.joint_state('arm_'+str(i+1)+'_joint').position[0] for i in range(5)]

	def pick(self, gripper_time):
		self.gripper.publish(1.0)
		rospy.sleep(0.5)
		self.arm[3].publish(radians(-70))
		rospy.sleep(0.5)
		self.arm[2].publish(radians(55))
		rospy.sleep(0.5)
		self.arm[1].publish(radians(60))
		rospy.sleep(0.5)
		self.arm[0].publish(radians(0))
		rospy.sleep(0.5)
		self.gripper.publish(0.0)
		rospy.sleep(gripper_time)


	def binarizer(self, state):
		t = radians(180)/5
		bin_state = [state[i]//t for i in range(1, 4)]
		return bin_state 
		


	

