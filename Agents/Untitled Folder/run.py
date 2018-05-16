import numpy as np
import gym
import gym_crumb
from math import radians
from qlearning_pick import QLearningAgent
import util
from util import play_and_train

if __name__ == '__main__':
    env = gym.make("crumb-pick-v0")
    env.reset()
    agent = QLearningAgent(alpha=0.25,epsilon=0.8,discount=0.99)
    rewards = []
    for i in range(1000):
        rewards.append(play_and_train(env,agent))