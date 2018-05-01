import gym
import rospy
from gym import error, spaces, utils
from gym.utils import seeding
from gazebo_msgs.srv import GetModelState, GetJointProperties, GetLinkState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from math import radians
import numpy as np

sign = lambda a: 1 if a>0 else -1 if (a<0) else 0
module = lambda a: a*sign(a)
metric = lambda a1, a2: ((a1.x - a2.x)**2 + (a1.y - a2.y)**2 + (a1.z - a2.z)**2)**(1/2)


class CrumbPickEnv(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		rospy.init_node('arm_start')
		self.arm = [rospy.Publisher('arm_'+str(i+1)+'_joint/command', Float64, queue_size=10) for i in range(5)]
		self.gripper = rospy.Publisher('gripper_1_joint/command', Float64, queue_size=10)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.resetworld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		self.box_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		self.link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.aim = self.box_state('little_box_0', 'link').pose.position
		self.unpause()
		self.resetworld()

	def _reset(self):
		for i in range(5):
			self.arm[i].publish(0.0)
			rospy.sleep(0.5)
		self.gripper.publish(2.0)
		self.resetworld()
		_, state = self.get_state()
		return state
		
	def _seed(self, seed = None):
		print('12')

	def _step(self, action):
		"""action = (joint, step)"""
		gripper = self.link_state('gripper_1_link', '').link_state.pose.position
		r1 = metric(gripper, self.aim)//0.05
		self.arm[action[0]].publish(action[1])
		rospy.sleep(0.2)
		gripper = self.link_state('gripper_1_link', '').link_state.pose.position
		r2 = metric(gripper, self.aim)//0.05
		reward = 0
		_, state = self.get_state() 
		reward = r1 - r2
		done = False
		if reward < 0:
			reward = reward * 2
		if reward == 0:
			reward = -1
		if (r2 == 0):
			done = True
			reward = 100 		
		return state, reward, done

	def get_state(self):
		return self.box_state('little_box_0', 'link').pose.position.z, [self.joint_state('arm_'+str(i+1)+'_joint').position[0] for i in range(5)]





	

