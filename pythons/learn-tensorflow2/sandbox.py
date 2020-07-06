def print_public_members(target, only_count = True):
    def _empty(_):
        pass

    def _print(field_name):
        print(field_name)

    print_member = _empty if only_count else _print
    method_count = 0
    for key in dir(target):
        if not key.startswith('_'):
            method_count += 1
            print_member(key)
    print('{} has {} public members.'.format(target.__name__, method_count))


def play_with_primitive_array():
    array_a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    print(array_a[1:2], array_a[::2], array_a[1:8:3])
    print(array_a[0:-2], array_a[0:-5])
    print(array_a[0:3], array_a[:3])
    print(array_a[3:], array_a[3:1])
    print(array_a[-1:], array_a[-1:0])


def play_with_tensorflow():
    import numpy as np
    help(np.ndarray)

    print('Belows are tensorflow relevant. ====>>>')
    import tensorflow as tf
    data = tf.keras.datasets.mnist.load_data(path = "mnist.npz")
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
    print_public_members(Model)
    # print(Model.compile.__doc__)
    # printPublicMembers(tf.keras.layers, False)
    # print(tf.keras.layers.Layer.__doc__)
    # print(tf.keras.models.Sequential.__doc__)
    # print(tf.keras.layers.Dense.__doc__)
    # print(tf.keras.layers.Flatten.__doc__)
    # print(tf.keras.layers.Dropout.__doc__)
    # help("modules")


if __name__ == '__main__':
    import pybullet as bt

    help(bt)
