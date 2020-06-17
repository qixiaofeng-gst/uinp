def train_with_table():
    import gym
    import numpy as np
    env = gym.make('FrozenLake-v0')
    Q = np.zeros([env.observation_space.n, env.action_space.n])
    learning_rate = .8
    y = .95
    num_episodes = 2000
    reward_list = []
    for i in range(num_episodes):
        state = env.reset()
        reward_sum = 0
        d = False
        j = 0
        while j < 99:
            j += 1
            a = np.argmax(Q[state, :] + np.random.randn(1, env.action_space.n) * (1. / (i + 1)))
            next_state, reward, d, _ = env.step(a)
            Q[state, a] = Q[state, a] + learning_rate * (reward + y * np.max(Q[next_state, :]) - Q[state, a])
            reward_sum += reward
            state = next_state
            if True == d:
                break
            reward_list.append(reward_sum)
    print('Score over time:', sum(reward_list) / num_episodes)
    print('Final Q-Table values:')
    print(Q)


def train_with_network():
    # TODO required to completely refactored to tensorflow 2.
    import gym
    import numpy as np
    import tensorflow as tf
    import tensorflow.keras.backend as kb
    import matplotlib.pyplot as plt

    env = gym.make('FrozenLake-v0')
    kb.clear_session()
    input_state_one = kb.placeholder(shape = [1, 16], dtype = tf.float64)
    w = kb.variable(kb.random_uniform([16, 4], 0, 0.01, dtype = tf.float64), dtype = tf.float64)
    qout = tf.linalg.matmul(input_state_one, w)
    predict = kb.argmax(qout, 1)
    next_q = kb.placeholder(shape = [1, 4], dtype = tf.float64)
    loss = tf.math.reduce_sum(tf.math.square(next_q - qout))
    trainer = tf.keras.optimizers.SGD(learning_rate = 0.1)
    update_model = trainer.minimize(loss, None)
    init = kb.manual_variable_initialization()
    y = .99
    e = 0.1
    num_episodes = 2000
    j_list = []
    reward_list = []
    with tf.Session() as tfss:
        tfss.run(init)
        for i in range(num_episodes):
            state = env.reset()
            reward_sum = 0
            d = False
            j = 0
            while j < 99:
                j += 1
                action, all_q = tfss.run(
                    [predict, qout],
                    feed_dict = {'inputs1': np.identity(16)[state: state + 1]}
                )
                if np.random.rand(1) < e:
                    action[0] = env.action_space.sample()
                next_state, reward, d, _ = env.step(action[0])
                qnext = tfss.run(
                    qout,
                    feed_dict = {'inputs1': np.identity(16)[next_state: next_state + 1]}
                )
                max_qnext = np.max(qnext)
                target_q = all_q
                target_q[0, action[0]] = reward + y * max_qnext

                _, wnext = tfss.run(
                    [update_model, w],
                    feed_dict = {
                        'inputs1': np.identity(16)[state: state + 1],
                        next_q: target_q
                    }
                )
                reward_sum += reward
                state = next_state
                if True == d:
                    e = 1. / ((1 / 50) + 10)
                    break
            j_list.append(j)
            reward_list.append(reward_sum)
    print('Percent of successful episodes:', sum(reward_list) / num_episodes)
    plt.plot(reward_list)
    plt.plot(j_list)


if __name__ == '__main__':
    from tools.profiler import executeWithTimestamp

    executeWithTimestamp(train_with_table)
