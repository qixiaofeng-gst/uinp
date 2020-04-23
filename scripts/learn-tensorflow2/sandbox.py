def printPublicMembers(target, onlyCount=True):
    def _empty(key):
        pass
    def _print(key):
        print(key)
    printMember = _empty if onlyCount else _print
    methodCount = 0
    for key in dir(target):
        if not key.startswith('_'):
            methodCount += 1
            printMember(key)
    print('{} has {} public members.'.format(target.__name__, methodCount))

if __name__ == '__main__':

    import os
    currentFileName = os.path.basename(__file__)
    print(currentFileName, '<<<<<<<', currentFileName[:currentFileName.rindex('.')])
    print(os.path.exists(__file__))

    arrayA = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    print(arrayA[1:2], arrayA[::2], arrayA[1:8:3])
    print(arrayA[0:-2], arrayA[0:-5])
    print(arrayA[0:3], arrayA[:3])
    print(arrayA[3:], arrayA[3:1])
    print(arrayA[-1:], arrayA[-1:0])

    import numpy as np
    help(np.ndarray)

    print('Belows are tensorflow relevant. ====>>>')
    import tensorflow as tf
    data = tf.keras.datasets.mnist.load_data(path="mnist.npz")
    print(type(data), len(data))
    train, test = data
    print(type(train), len(train), type(test), len(test))

    a, b = train
    c, d = test
    print('a', type(a), len(a))
    print('b', type(b), len(b))
    print('c', type(c), len(c))
    print('d', type(d), len(d))

    a_first = a[:1]
    print('a_first', type(a_first), a_first.shape)
    b_first = b[:1]
    print('b_first', type(b_first), b_first.shape, b_first)
    b_last = b[-1:]
    print('b_last', type(b_last), b_last)

    from tensorflow.keras import Model
    print(tf.keras.layers.__file__)
    printPublicMembers(Model)
    #print(Model.compile.__doc__)
    #printPublicMembers(tf.keras.layers, False)
    #print(tf.keras.layers.Layer.__doc__)
    #print(tf.keras.models.Sequential.__doc__)
    #print(tf.keras.layers.Dense.__doc__)
    #print(tf.keras.layers.Flatten.__doc__)
    #print(tf.keras.layers.Dropout.__doc__)
    #help("modules")
