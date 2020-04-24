def trainWithTable():
    import gym
    import numpy as np
    env = gym.make('FrozenLake-v0')
    Q = np.zeros([env.observation_space.n, env.action_space.n])
    learningRate = .8
    y = .95
    num_episodes = 2000
    rewardList = []
    for i in range(num_episodes):
        state = env.reset()
        rewardSum = 0
        d = False
        j = 0
        while j < 99:
            j += 1
            a = np.argmax(Q[state,:] + np.random.randn(1, env.action_space.n) * (1./(i + 1)))
            nextState, reward, d, _ = env.step(a)
            Q[state, a] = Q[state, a] + learningRate * (reward + y * np.max(Q[nextState,:]) - Q[state, a])
            rewardSum += reward
            state = nextState
            if d == True:
                break
            rewardList.append(rewardSum)
    print('Score over time:', sum(rewardList) / num_episodes)
    print('Final Q-Table values:')
    print(Q)

def trainWithNetwork():
    #TODO required to completely refactored to tensorflow 2.
    import gym
    import numpy as np
    import random
    import tensorflow as tf
    import tensorflow.keras.backend as kb
    import matplotlib.pyplot as plt

    env = gym.make('FrozenLake-v0')
    kb.clear_session()
    inputStateOne = kb.placeholder(shape=[1, 16], dtype=tf.float64)
    W = kb.variable(kb.random_uniform([16, 4], 0, 0.01, dtype=tf.float64), dtype=tf.float64)
    Qout = tf.linalg.matmul(inputStateOne, W)
    predict = kb.argmax(Qout, 1)
    nextQ = kb.placeholder(shape=[1, 4], dtype=tf.float64)
    loss = tf.math.reduce_sum(tf.math.square(nextQ - Qout))
    trainer = tf.keras.optimizers.SGD(learning_rate=0.1)
    updateModel = trainer.minimize(loss)
    init = kb.manual_variable_initialization()
    y = .99
    e = 0.1
    num_episodes = 2000
    jList = []
    rewardList = []
    with tf.Session() as tfss:
        tfss.run(init)
        for i in range(num_episodes):
            state = env.reset()
            rewardSum = 0
            d = False
            j = 0
            while j < 99:
                j += 1
                action, allQ = tfss.run(
                    [predict, Qout],
                    feed_dict={inputs1:np.identity(16)[state : state + 1]}
                )
                if np.random.rand(1) < e:
                    action[0] = env.action_space.sample()
                nextState, reward, d, _ = env.step(action[0])
                Qnext = tfss.run(
                    Qout,
                    feed_dict={inputs1: np.identity(16)[nextState : nextState + 1]}
                )
                maxQnext = np.max(Qnext)
                targetQ = allQ
                targetQ[0, action[0]] = reward + y * maxQnext

                _, Wnext = tfss.run(
                    [updateModel, W],
                    feed_dict={
                        inputs1:np.identity(16)[state : state + 1],
                        nextQ: targetQ
                    }
                )
                rewardSum += reward
                state = nextState
                if d == True:
                    e = 1. / ((1 / 50) + 10)
                    break
            jList.append(j)
            rewardList.append(rewardSum)
    print('Percent of successful episodes:', sum(rewardList) / num_episodes)
    plt.plot(rewardList)
    plt.plot(jList)

if __name__ == '__main__':
    from tools.profiler import executeWithTimestamp
    executeWithTimestamp(trainWithTable)
