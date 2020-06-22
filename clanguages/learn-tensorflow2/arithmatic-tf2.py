import random

limit = 100


def create_data(quantity):
    x_data = []
    y_data = []
    for _ in range(quantity):
        xa, xb = float(random.randint(0, limit)), float(random.randint(0, limit))
        y = xa * xb / (limit * limit)
        x_data.append([xa, xb])
        y_data.append(y)
    return x_data, y_data


def fit_multiplication():
    import tensorflow as tf
    from tools.weights_loader import WeightsLoader
    # with tf.device("/gpu:0"): # Use /cpu:0 or /gpu:0.

    (x_train, y_train), (x_test, y_test) = create_data(1024 * 128), create_data(1024)

    model = tf.keras.models.Sequential([
        tf.keras.layers.InputLayer(input_shape = (2,), dtype = 'float64'),
        tf.keras.layers.Dense(256, activation = 'sigmoid', dtype = 'float64'),
        tf.keras.layers.Dense(1, activation = 'sigmoid', dtype = 'float64'),
    ])

    loss_fn = tf.keras.losses.MeanSquaredError()
    model.compile(optimizer = 'sgd', loss = loss_fn, metrics = ['accuracy'])

    weights_loader = WeightsLoader(__file__)
    model.fit(x_train, y_train, batch_size = 256, epochs = 5)
    model.evaluate(x_test, y_test, verbose = 1)
    weights_loader.save_weights(model)


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(fit_multiplication)
