import numpy as np


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


if __name__ == '__main__':
    from tools.profiler import execute_with_timestamp

    execute_with_timestamp(lets_go)
