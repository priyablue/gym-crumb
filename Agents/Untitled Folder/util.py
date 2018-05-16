import numpy as np
from math import radians
sign = lambda a: 1 if a>0 else -1 if (a<0) else 0
def signs(vec_a):
    signs = np.zeros(len(vec_a))
    for i in range(len(vec_a)):
        signs[i] = sign(vec_a[i])
    return signs
        
        
def module(vec_a):
    return (vec_a**2).sum()**(0.5)

def angle(vec_a, vec_b):
    return np.arccos((vec_a*vec_b).sum(axis = 1)/(module(vec_a) * module(vec_b)))
def norm(vec_a):
    norm = np.zeros_like(vec_a)
    for i in range(len(vec_a)):
        norm[i][0] = -vec_a[i][1]
        norm[i][1] = vec_a[i][0]
    return norm
        
def get_xz(joint, env):
    a = env.link_state(joint, '').link_state.pose.position
    return np.array([a.x, a.z])

def get_xy(joint, env):
    a = env.link_state(joint, '').link_state.pose.position
    return np.array([a.x, a.y])

def state1(aim, env):
    aim1 = np.array([aim.x, aim.z])
    aim1_gor = np.array([aim.x, aim.y])

    #Joints pose vert
    joint = np.zeros((6,2))
    joint[0] = get_xz('biceps_link', env)
    joint[1] = get_xz('forearm_link', env)
    joint[2] = get_xz('wrist_1_link', env)
    joint[3] = get_xz('gripper_1_link', env)
    
    #Joints pose gor
    joint[4] = get_xy('gripper_1_link', env)
    joint[5] = get_xy('biceps_link',env)
    
    #vectors
    vec1 = np.zeros((3,2))
    vec2 = np.zeros((3,2))
    vec3 = np.zeros((1,2))
    vec4 = np.zeros((1,2))
    
    vec3[0] = joint[4] - joint[5]
    vec4[0] = aim1_gor - joint[5]
    for i in range(3):
        vec1[i] = joint[3] - joint[i]
        vec2[i] = aim1 - joint[i]
    
    
    state1 = signs(angle(norm(vec1), vec2) - np.full(3, radians(90)))
    state2 = signs(angle(norm(vec3), vec4) - np.full(3, radians(90))) #вестор из 1, -1, 0
    return (tuple(state1), tuple(state2))

def play_and_train(env,agent,t_max=10**3):

    total_reward = 0.0
    s = env.reset()
    r = 0
    #env.step((3, radians(0)))
    for t in range(t_max):
        state = state1(env.aim, env)#state for agent, s for gazebo
        action = agent.getAction(state)#<get agent to pick action given state s>
        a = (action[0], action[1] + s[action[0]]) 
        next_s,r, done = env.step(a)
        next_state = state1(env.aim, env)
        agent.update(state, action, next_state, r)#<train (update) agent for state s>
        
        s = next_s
        total_reward +=r
        if done:
            agent.epsilon = agent.epsilon * 0.8
            print ('yyes')
            break
    return total_reward
