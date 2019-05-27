import gym
import gym_unicycle
import time 
import numpy as np

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common import set_global_seeds
from stable_baselines import PPO2


#env = gym.make('unicycle-v0')
#env = DummyVecEnv([lambda: env])
model_name = 'reward_function_nr1_2'
env_id = "unicycle-v2"
num_cpu = 8  # Number of processes to use
# Create the vectorized environment

def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.
    
    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environment you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = gym.make(env_id)
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init





def train_ppo_constant_learningrate(steps, env, model, lr, tb_log_name, tb_log_dir, tb_log_file,model_name):
    
    
    if model is None:
        model = PPO2(MlpPolicy, env, verbose=1, learning_rate=lr, gamma=0.99, tensorboard_log=tb_log_dir)
    else:
        model = PPO2.load('models/new_base_model_with_noise3.pkl',env=env,learning_rate=lr)

    model.learn(total_timesteps=steps, tb_log_name=tb_log_name)
    model.save(model_name)



def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    #if (n_steps + 1) % 1000 == 0:
        # Evaluate policy performance
     #   x, y = ts2xy(load_results(log_dir), 'timesteps')
      #  if len(x) > 0:
       #     mean_reward = np.mean(y[-100:])
        #    print(x[-1], 'timesteps')
         #   print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))

            # New best model, you could save the agent here
          #  if mean_reward > best_mean_reward:
           #     best_mean_reward = mean_reward
            #    # Example for saving best model
             #   print("Saving new best model")
              #  _locals['self'].save(log_dir + 'best_model.pkl')
    #n_steps += 1
    return True
env = SubprocVecEnv([make_env(env_id, i) for i in range(num_cpu)])
train_ppo_constant_learningrate(steps=5000000, 
                                env=env, 
                                model=None, 
                                lr=10e-4, 
                                tb_log_name='first_run_lr=10e-4', 
                                tb_log_dir='tensorboard_logs', 
                                tb_log_file='testing_log_first_run',
                                model_name='models/' + model_name)



#delete model
#del model
#model = PPO2.load("ppo2_unicycle")

