from stable_baselines3 import PPO
from HexapodBulletEnv import HexapodBulletEnv
import math
import pybullet as p
import matplotlib.pyplot as plt

def train(env, exp_name, **kwargs):
    for n in range(0, 10):
        log_dir = "tmp/"
        model = PPO(
            policy = "MlpPolicy",
            env = env,
            verbose = True,
            seed = 32 * n,
            tensorboard_log = log_dir,
            **kwargs
        )
        print("[+] Starting training", n)
        #model = PPO.load("trained_models/" + "test" + "_" + str(n - 1), env = env, tensorboard_log = log_dir)
        model.learn(
            total_timesteps = 100000,
            log_interval = 1,
            tb_log_name = "PPO",
        )
        model.save("trained_models/" + exp_name + "_" + str(n))
        env.close()
        del env
        env = HexapodBulletEnv(client, time_step = 0.25, frameskip = 60, render = False,  max_velocity = 59*2*math.pi/60, max_torque = 1.50041745)
        del model

if __name__ == "__main__":
    client = p.connect(p.GUI)
    env = HexapodBulletEnv(client, time_step = 0.25, frameskip = 60, render = False,  max_velocity = 59*2*math.pi/60, max_torque = 1.50041745)
    train(
        env = env,
        exp_name = "test",
        gamma = 0.99,
        n_steps = 128,
        batch_size = 32,
        ent_coef = 0.01,
        learning_rate = 1e-3,
        clip_range = 0.2,
        device = "auto",
        _init_setup_model = True
    )
    client = p.connect(p.GUI)
    env = HexapodBulletEnv(client, time_step = 1, frameskip = 240, render = False,  max_velocity = 59*2*math.pi/60, max_torque = 1.50041745)
    model = PPO.load("trained_models/" + "test" + "_" + str(9))
    obs = env.reset()
    r = []
    t = []
    dt = 0
    for i in range(0, 100000):
        action, _ = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        if(done):
            env.reset()
        r.append(rewards)
        t.append(dt)
        dt += 1
        env.render()
        print(i)
    plt.plot(t, r)
    plt.axis([0, 10000, -60, 60])
    plt.show()
    plt.title('PPO')
    plt.xlabel('Time Steps')
    plt.ylabel('Reward')
    plt.savefig('PPO.png')