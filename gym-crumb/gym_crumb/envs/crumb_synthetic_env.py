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

sign = lambda a: 1 if a>0 else -1 if (a<0) else 0
module = lambda a: a*sign(a)
metric = lambda a1, a2: ((a1[0] - a2[0])**2 + (a1[1] - a2[1])**2)**(1/2)

class CrumbSyntheticEnv(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		#read file, return state,
		self.state = np.radians(np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/state_f.txt')) #arm position
		self.aim = np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/aim_f.txt') #aim position 


	def _reset(self):
		#same as in init
		self.state = np.radians(np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/state_f.txt'))
		state = [self.state[i] for i in range(3)]
		return state
		
	def _seed(self, seed = None):
		print('12')

	def _render(self, mode='human', close=False):
		box_pose = self.get_box_pose()
		state = [self.state[i] for i in range(3)]
		return state, box_pose, metric(box_pose, self.aim) 

	def _step(self, action):
		"""action = (joint i, step)"""
		box_pose = self.get_box_pose()
		r1 = metric(self.aim, box_pose)//(10**(-1))
		self.state[action[0]] = action[1]
		box_pose = self.get_box_pose() 
		r2 = metric(self.aim, box_pose)//(10**(-1)) 		
		reward = 0
		done = False 		
		reward = r1 - r2
		if reward < 0:
			reward *= 2
		if reward == 0:
			reward = -5
		if (r2 == 0): 
			done = True
			reward = 1000
		state = [self.state[i] for i in range(3)] 		
		return state, reward, done
	
	def get_box_pose(self):
		arm_pose = np.zeros(3)
		box_pose = np.zeros(2)
		arm_pose[0] = self.state[0]
		for i in range(1,3):
			arm_pose[i] = self.state[i] + arm_pose[i-1] #to calculate box_pose we need angle relative to the horizon

		box_pose[0] = np.sum(np.sin(arm_pose))
		box_pose[1] = np.sum(np.cos(arm_pose))
		return box_pose

			
 		
		


	

