from gym.envs.registration import register

register(
    id='crumb-v0',
    entry_point='gym_crumb.envs:CrumbEnv',
)
register(
    id='crumb-synthetic-v0',
    entry_point='gym_crumb.envs:CrumbSyntheticEnv',
)
register(
    id='crumb-pick-v0',
    entry_point='gym_crumb.envs:CrumbPickEnv',
)

