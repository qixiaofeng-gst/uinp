#!/usr/bin/env python3
"""
Situation is suite for utilizing GPU:
1. Big network;
2. Big batch size.

Estimate largest batch size:
Max batch size = available GPU memory bytes / 4 / (size of tensors + trainable parameters)
"""


def play_mnist():
    import tensorflow as tf
    print('Using tensorflow', tf.__version__, '====>>> below personal code starts.')

    # with tf.device("/gpu:0"): # Use /cpu:0 or /gpu:0.
    mnist = tf.keras.datasets.mnist

    (x_train, y_train), (x_test, y_test) = mnist.load_data(path = "mnist.npz")
    x_train, x_test = x_train / 255.0, x_test / 255.0

    model = tf.keras.models.Sequential([
        tf.keras.layers.Flatten(input_shape = (28, 28), dtype = 'float64'),
        tf.keras.layers.Dense(256, activation = 'relu', dtype = 'float64'),
        tf.keras.layers.Dense(10, dtype = 'float64'),
    ])

    loss_fn = tf.keras.losses.SparseCategoricalCrossentropy(from_logits = True)
    model.compile(optimizer = 'adam', loss = loss_fn, metrics = ['accuracy'])

    from tools.weights_loader import WeightsLoader
    weights_loader = WeightsLoader(__file__)
    if weights_loader.has_weights():
        print('====>>> Loading weights.')
        weights_loader.load_weights(model)
        print('====>>> Loaded weights.')
    else:
        model.fit(x_train, y_train, batch_size = 256, epochs = 5)
    model.evaluate(x_test, y_test, verbose = 1)
    model.evaluate(x_train, y_train, verbose = 1)
    weights_loader.save_weights(model)


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(play_mnist)
