from gym.envs.registration import register

#register(
#    id='unicycle-v0',
#    entry_point='gym_unicycle.envs:UnicycleEnv',
#)

#register(
#         id='unicycle-v1',
#         entry_point='gym_unicycle.envs:UnicycleSYSEnv',
#         )

register(
         id='unicycle-v2',
         entry_point='gym_unicycle.envs:UnicycleSYSEnv_v2',
         )
