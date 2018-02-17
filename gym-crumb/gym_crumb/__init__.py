from gym.envs.registration import register

register(
    id='crumb-v0',
    entry_point='gym_crumb.envs:CrumbEnv',
)
