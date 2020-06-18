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
            default_rng.standard_normal(size = (neuron_quantity, 1))
            for neuron_quantity in
            layer_sizes[1:]
        ]
        self._weights = [
            default_rng.standard_normal(size = (output_quantity, input_quantity))
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
                print('Epoch {}: {} / {}'.format(
                    epoch_sn,
                    self.evaluate(test_data),
                    test_length,
                ))
            else:
                print('Epoch {} complete'.format(epoch_sn))

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
            delta_nabla_bias, delta_nabla_weight = self.backprop(x, y)
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

    def backprop(self, x, y):
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
        return sum(int(x == y) for (x, y) in test_results)


class Neuron:
    def __init__(self, input_quantity: int):
        self._weights = np.zeros(input_quantity, dtype = np.int8)
        self._bias = 0

    def __setitem__(self, key, value):
        self._weights[key] = value

    @property
    def b(self):
        return self._bias

    @b.setter
    def b(self, value):
        self._bias = value

    def calculate_z_for(self, input_values):
        if len(input_values) == len(self._weights):
            return np.sum(self._weights * input_values) + self._bias
        else:
            raise RuntimeError('Invalid input shape {}, expecting {}.'.format(
                len(input_values),
                len(self._weights),
            ))

    def calculate_for(self, input_values):
        raise NotImplementedError


class Perceptron(Neuron):
    """
    A perceptrons formed network is hard-to-control.
    Any weight change of a single perceptron may cause a big behavior change of the rest of the network.
    """

    def __init__(self, input_quantity: int):
        super(Perceptron, self).__init__(input_quantity)

    def calculate_for(self, input_values):
        return 1 if self.calculate_z_for(input_values) > 0 else 0


class SigmoidNeuron(Neuron):
    def __init__(self, input_quantity):
        super(SigmoidNeuron, self).__init__(input_quantity)

    def calculate_for(self, input_values):
        return 1 / (1 + np.exp(self.calculate_z_for(input_values)))


def lets_go():
    print('Yeah, let\'s go!')
    # Below perceptron is a presentation for NAND gate.
    p = Perceptron(2)
    p[0] = -2
    p[1] = -2
    p.b = 3
    print(
        '>>>>>>> [1, 1] -> {} '
        '&& [1, 0] -> {} '
        '&& [0, 1] -> {} '
        '&& [0, 0] -> {}'.format(
            p.calculate_for([1, 1]),
            p.calculate_for([1, 0]),
            p.calculate_for([0, 1]),
            p.calculate_for([0, 0]),
        )
    )


def learn_dnn():
    training_data = [
        (np.array([[.2], [.1]]), np.array([[.02]])),
        (np.array([[.3], [.1]]), np.array([[.03]])),
        (np.array([[.4], [.5]]), np.array([[.20]])),
        (np.array([[.7], [.5]]), np.array([[.35]])),
    ]
    test_data = [
        (np.array([[.3], [.4]]), np.array([[.12]])),
        (np.array([[.5], [.6]]), np.array([[.30]])),
    ]
    net = Network([2, 2, 1])
    net.train_with_sgd(training_data, 10, 2, 1, test_data)
    print('>>>>>>>', net.feed_forward([[.1], [.2]]))


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(lets_go)
    execute_with_timestamp(learn_dnn)
