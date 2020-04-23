"""
Situation is suite for utilizing GPU:
1. Big network;
2. Big batch size.

Estimate largest batch size:
Max batch size = available GPU memory bytes / 4 / (size of tensors + trainable parameters)
"""
def gen_getWeightFileName(fullSourceFilePath):
    def _function():
        import os
        dot = '.'
        currentFileName = os.path.basename(__file__)
        return currentFileName[:currentFileName.rindex(dot)] + dot + 'h5'
    return _function

getWeightFileName = gen_getWeightFileName(__file__)

def saveWeights(model):
    model.save_weights(getWeightFileName())

def loadWeights(model):
    model.load_weights(getWeightFileName())

def hasWeights():
    import os
    return os.path.exists(getWeightFileName())

if __name__ == '__main__':
    from datetime import datetime
    import tensorflow as tf
    startTimestamp = datetime.now().timestamp()
    print('Using tensorflow', tf.__version__, '====>>> below personal code starts.')

    #with tf.device("/gpu:0"): # Use /cpu:0 or /gpu:0.
    mnist = tf.keras.datasets.mnist

    (x_train, y_train), (x_test, y_test) = mnist.load_data(path="mnist.npz")
    x_train, x_test = x_train / 255.0, x_test / 255.0

    model = tf.keras.models.Sequential([
      tf.keras.layers.Flatten(input_shape=(28, 28), dtype='float64'),
      tf.keras.layers.Dense(256, activation='relu', dtype='float64'),
      #tf.keras.layers.Dropout(0.2, dtype='float64'),
      tf.keras.layers.Dense(10, dtype='float64')
    ])

    #predictions = model(x_train[:1]).numpy()
    #tf.nn.softmax(predictions).numpy()

    loss_fn = tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True)
    model.compile(optimizer='adam', loss=loss_fn, metrics=['accuracy'])

    if hasWeights():
        print('====>>> Loading weights.')
        loadWeights(model)
        print('====>>> Loaded weights.')
    else:
        model.fit(x_train, y_train, batch_size=256, epochs=5)
    model.evaluate(x_test, y_test, verbose=1)
    model.evaluate(x_train, y_train, verbose=1)
    saveWeights(model)
    print('Cost time: {:.6f}s'.format(datetime.now().timestamp() - startTimestamp))
