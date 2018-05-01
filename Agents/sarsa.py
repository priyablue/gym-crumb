

import random,math
from math import radians

import numpy as np
from collections import defaultdict

class SarsaAgent():
  
  def __init__(self,alpha,epsilon,discount):

    self.Actions = ()
    for i in range(1,4):
	self.Actions += ((i, radians(90)), (i, radians(0)), (i, radians(-90)))
    self._qValues = defaultdict(lambda:defaultdict(lambda:0))
    self.alpha = alpha
    self.epsilon = epsilon
    self.discount = discount

  def getQValue(self, state, action):
    """
      Returns Q(state,action)
    """
    return self._qValues[state][action]

  def setQValue(self,state,action,value):
    """
      Sets the Qvalue for [state,action] to the given value
    """
    self._qValues[state][action] = value
   
  def getPolicy(self, state):
    """
      Compute the best action to take in a state. 
      
    """
    possibleActions = self.Actions
    
    best_action = None

    best_action = possibleActions[np.argmax([self.getQValue(state, a) for a in possibleActions])]
    return best_action

  def getAction(self, state):
    """
      Compute the action to take in the current state, including exploration.  

    """
    
    # Pick Action
    possibleActions = self.Actions
    action = None

    #agent parameters:
    epsilon = self.epsilon

    if np.random.random()<=epsilon:
    	return random.choice(possibleActions)
    else:
    	action = self.getPolicy(state)
    return action

  def update(self, state, action, nextState, nextAction, reward):

    #agent parameters
    gamma = self.discount
    learning_rate = self.alpha
    
    reference_qvalue = reward + gamma * self.getQValue(nextState, nextAction)
    updated_qvalue = (1-learning_rate) * self.getQValue(state,action) + learning_rate * reference_qvalue
    self.setQValue(state,action,updated_qvalue)



