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
		self.pick()

	def _reset(self):
		for i in range(5):
			self.arm[i].publish(0.0)
			rospy.sleep(0.5)
		self.resetworld()
		self.pick()
		reward, state = self.get_state()
		return state, reward
		
	def _seed(self, seed = None):
		print('12')

	def _step(self, action):
		"""action = (joint, step)"""
		self.arm[action[0]].publish(action[1])
		rospy.sleep(0.5)
		reward, state = self.get_state()		
		return state, reward

	def get_state(self):
		return self.box_state('little_box_0', 'link').pose.position.z, [self.joint_state('arm_'+str(i+1)+'_joint').position for i in range(5)]

	def pick(self):
		self.gripper.publish(1.0)
		rospy.sleep(0.5)
		self.arm[3].publish(radians(-70))
		rospy.sleep(0.5)
		self.arm[2].publish(radians(55))
		rospy.sleep(0.5)
		self.arm[1].publish(radians(60))
		rospy.sleep(0.5)
		self.gripper.publish(0.0)
		rospy.sleep(5.0)
		


	

