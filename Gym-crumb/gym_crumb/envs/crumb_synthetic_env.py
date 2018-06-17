import gym
from gym import error, spaces, utils
from gym.utils import seeding
from math import radians
import numpy as np

metric = lambda a1, a2: ((a1[0] - a2[0])**2 + (a1[1] - a2[1])**2)**(1/2)
sign = lambda a: 1 if a>0 else -1 if (a<0) else 0
arm_length = np.array([0.15000096, 0.14202964, 0.11450014])
def signs(vec_a):
    signs = np.zeros(len(vec_a))
    for i in range(len(vec_a)):
        signs[i] = sign(vec_a[i])
    return signs
        
def module(vec_a):
    return ((vec_a**2).sum(axis = 1)**(0.5)).reshape(3,1)

def angle(vec_a, vec_b):
    return np.arccos(((vec_a/module(vec_a))*(vec_b/module(vec_b))).sum(axis = 1))

def norm(vec_a):
    norm = np.zeros_like(vec_a)
    for i in range(len(vec_a)):
        norm[i][0] = -vec_a[i][1]
        norm[i][1] = vec_a[i][0]
    return norm

def state1(state, aim, synth = False):
    aim1 = aim
    #Угол к вертикали
    hor = np.zeros(3)
    hor[0] = state[0]
    hor[1] = radians(90)
    for i in range(1,3):
        hor[i] += -state[i] + hor[i-1]
        
    #Joints pose
    joint_x = np.zeros(4) #*a
    joint_y = np.zeros(4)
    for i in range(3):
        joint_x[i+1] = joint_x[i] + np.sin(hor[i])*arm_length[i]
        joint_y[i+1] = joint_y[i] + np.cos(hor[i])*arm_length[i]
    joint = np.array([joint_x,joint_y]).T

    
    #vectors
    vec1 = np.zeros((3,2))
    vec2 = np.zeros((3,2))
    vec3 = np.zeros((3,2))
    
    for i in range(3):
        vec1[i] = joint[3] - joint[i]
        vec2[i] = aim1 - joint[i]
    
    state1 = (angle(norm(vec1), vec2) - np.full(3, radians(90)))//0.01*0.01
    vec = tuple((vec1[0] - vec2[0])//0.01*0.01)
    vec += tuple((vec1[1] - vec2[1])//0.01*0.01)
    vec += tuple((vec1[2] - vec2[2])//0.01*0.01)
    return vec+(state1[0],state1[1], state[2])
    



class CrumbSyntheticEnv(gym.Env):
	metadata = {'render.modes': ['human']}

	def __init__(self):
		#read file, return state,
		self.state = np.radians(np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/state_f.txt')) #arm position
		self.aim = np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/aim_f.txt') #aim position 


	def _reset(self):
		self.state = np.radians(np.loadtxt('/home/airan/ISA/helloworld/gym-crumb/state_f.txt'))
		box_pose = self.get_box_pose()
		state = (metric(box_pose, self.aim),)
		state += state1([self.state[i] for i in range(3)], self.aim)
		state += (self.state[0], self.state[1], self.state[2])
		return state
		
	def _seed(self, seed = None):
		print('12')

	def _render(self, mode='human', close=False):
		box_pose = self.get_box_pose()
		state = (metric(box_pose, self.aim)//0.01*0.01,)
		state += state1([self.state[i] for i in range(3)], self.aim)
		state += (self.state[0]//0.01*0.01,self.state[1]//0.01*0.01, self.state[2]//0.01*0.01)
		return state

	def _step(self, action):
		"""action = (joint i, step)"""
		box_pose = self.get_box_pose()
		r1 = metric(self.aim, box_pose)//0.05
		self.state[action[0]] += action[1]
		self.state = self.state %(sign(self.state[action[0]])*2*np.pi)
		box_pose = self.get_box_pose() 
		r2 = metric(self.aim, box_pose)//0.05		
		reward = 0
		done = False 		
		reward = r1 - r2
		if reward < 0:
			reward *= 2
		if reward == 0:
			reward = -5
		if (r2 == 0): 
			done = True
			reward += 100
		state = (metric(box_pose, self.aim),)
		state += state1([self.state[i] for i in range(3)], self.aim)
		state += (self.state[0], self.state[1], self.state[2])
		return state, reward, done
	
	def get_box_pose(self):
		arm_pose = np.zeros(3)
		box_pose = []
		arm_pose[0] = self.state[0]
		arm_pose[1] = radians(90)		
		for i in range(1,3):
			arm_pose[i] += -self.state[i] + arm_pose[i-1] #углы к вертикали

		box_pose += (np.sum(np.sin(arm_pose)*arm_length),)
		box_pose += (np.sum(np.cos(arm_pose)*arm_length),)
		return box_pose

			
 		
		


	

