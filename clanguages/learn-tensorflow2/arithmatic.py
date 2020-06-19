"""
Properties of whole model:
    Optimizer: SGD, Momentum, Adam, Adadelta, Adabound. (See https://keras.io/api/optimizers/)
    Metrics: Accuracy, MeanSquaredError. (See https://keras.io/api/metrics/)
    Cost(Loss) function: BinaryCrossentropy, ConsineSimilarity, MeanSquaredError. (See https://keras.io/api/losses/)

Properties of a single layer:
    Activation function: sigmoid, tanh, relu. (See https://keras.io/api/layers/)

Hadamard Product(Element-size multiplication) could be done by np.multiply().
"""

import numpy as np
import random


def sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))


def sigmoid_prime(z):
    """Derivative of the sigmoid function."""
    y = sigmoid(z)
    return y * (1 - y)


def cost_derivative(output_activations, y):
    r"""
    Return the vector of partial derivatives
        TeX[\partial C_x / \partial a] for the output activations.
    """
    return output_activations - y


class Network:
    """
    Origin author is Michael Nielson.
    See http://neuralnetworksanddeeplearning.com/chap1.html
    """

    def __init__(self, layer_sizes):
        self._num_layers = len(layer_sizes)
        self._layer_sizes = layer_sizes

        default_rng = np.random.default_rng()
        self._biases = [
            default_rng.standard_normal(
                size = (neuron_quantity, 1),
                dtype = np.float32,
            )
            for neuron_quantity in
            layer_sizes[1:]
        ]
        self._weights = [
            default_rng.standard_normal(
                size = (output_quantity, input_quantity),
                dtype = np.float32,
            )
            for input_quantity, output_quantity in
            zip(layer_sizes[:-1], layer_sizes[1:])
        ]

    def train_with_sgd(self, training_data, epoch_quantity, minibatch_size, eta, test_data = None):
        """
        Train with Stochastic Gradient Descent.
        :param training_data: is a list of tuples (x, y) representing the
            training inputs and the desired outputs.
        :param epoch_quantity
        :param minibatch_size
        :param eta
        :param test_data: optional
        """
        test_length = len(test_data) if test_data else 0
        length = len(training_data)
        max_hit_count = 0
        for epoch_sn in range(epoch_quantity):
            random.shuffle(training_data)
            minibatches = [
                training_data[k:k + minibatch_size]
                for k in
                range(0, length, minibatch_size)
            ]
            for minibatch in minibatches:
                self.update_minibatch(minibatch, eta)
            if test_length > 0:
                hit_count = self.evaluate(test_data)
                if hit_count > max_hit_count:
                    max_hit_count = hit_count
                print('Epoch {}: {} / {}'.format(
                    epoch_sn,
                    hit_count,
                    test_length,
                ))
            else:
                print('Epoch {} complete'.format(epoch_sn))
        return max_hit_count

    def update_minibatch(self, minibatch, eta):
        """
        Update the network's weights and biases by applying gradient descent
        using backpropagation to a single minibatch.
        :param minibatch: is a list of tuples
        :param eta: is the learning rate
        """
        affinity_eta = eta / len(minibatch)
        nabla_bias = [np.zeros(b.shape) for b in self._biases]
        nabla_weight = [np.zeros(w.shape) for w in self._weights]
        for x, y in minibatch:
            delta_nabla_bias, delta_nabla_weight = self.back_propagation(x, y)
            nabla_bias = [nb + dnb for nb, dnb in zip(nabla_bias, delta_nabla_bias)]
            nabla_weight = [nw + dnw for nw, dnw in zip(nabla_weight, delta_nabla_weight)]
        self._weights = [
            w - affinity_eta * nw
            for w, nw in
            zip(self._weights, nabla_weight)
        ]
        self._biases = [
            b - affinity_eta * nb
            for b, nb in
            zip(self._biases, nabla_bias)
        ]

    def back_propagation(self, x, y):
        """
        Calculate the gradient for the cost function C_x.
        :param x: input
        :param y: expected output
        :return: (nabla_bais, nabla_weight) tuple
        """
        nabla_bias = [np.zeros(b.shape) for b in self._biases]
        nabla_weight = [np.zeros(w.shape) for w in self._weights]

        # Feed forward
        activation = x
        activations = [x]
        zs = []
        for bias, weight in zip(self._biases, self._weights):
            z = np.dot(weight, activation) + bias
            zs.append(z)
            activation = sigmoid(z)
            activations.append(activation)

        # Backword pass
        delta = cost_derivative(activations[-1], y) * sigmoid_prime(zs[-1])
        nabla_bias[-1] = delta
        nabla_weight[-1] = np.dot(delta, activations[-2].transpose())
        for layer in range(2, self._num_layers):
            z = zs[-layer]
            sp = sigmoid_prime(z)
            delta = np.dot(self._weights[-layer + 1].transpose(), delta) * sp
            nabla_bias[-layer] = delta
            nabla_weight[-layer] = np.dot(delta, activations[-layer - 1].transpose())
        return nabla_bias, nabla_weight

    def feed_forward(self, inputs):
        activation = inputs
        for baises, weights in zip(self._biases, self._weights):
            activation = sigmoid(np.dot(weights, activation) + baises)
        return activation

    def evaluate(self, test_data):
        test_results = [
            (self.feed_forward(x)[0, 0], y[0, 0])
            for (x, y) in
            test_data
        ]
        # print('>>>>>>>', test_results)
        return sum(int(np.isclose(x, y, atol = 1e-2)) for (x, y) in test_results)


limit = 35


def create_data(quantity):
    training_data = []
    for _ in range(quantity):
        xa, xb = float(random.randint(0, limit)), float(random.randint(0, limit))
        y = xa * xb / (limit * limit)
        training_data.append((np.array([[xa], [xb]]), np.array([[y]])))
    return training_data


def learn_dnn():
    max_hit_count = 0
    for _ in range(10):
        training_data = create_data(1000)
        test_data = create_data(100)
        net = Network([2, 128, 64, 1])
        mhc = net.train_with_sgd(training_data, 5, 100, 1, test_data)
        if mhc > max_hit_count:
            max_hit_count = mhc
    print('Max hit count:', max_hit_count)


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(learn_dnn)
