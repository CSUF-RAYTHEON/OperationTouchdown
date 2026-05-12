import numpy as np
from .core import evaluate_bitwidth


def layer_str(self):
    layer_type = str(self.parameters.layer_type).split('.')[-1]
    layer_type = layer_type.replace("FullyConnected", "Fully.")
    layer_type = layer_type.replace("Convolutional", "Conv.")
    layer_type = layer_type.replace("Separable", "Sep.")
    return self.name + " (" + layer_type + ")"


def layer_repr(self):
    data = "<akida.Layer, type=" + str(self.parameters.layer_type)
    data += ", name=" + self.name
    data += ", input_dims=" + str(self.input_dims)
    data += ", output_dims=" + str(self.output_dims)
    if self.mapping is not None:
        data += ",nps=" + repr(self.mapping.nps)
    data += ">"
    return data


def layer_to_dict(self):
    """Provide a dict representation of the Layer

    Returns:
        dict: a Layer dictionary.
    """
    params = {name: getattr(self.parameters, name) for name in dir(self.parameters)}
    params["layer_type"] = self.parameters.layer_type.name
    variables = {}
    for name in self.variables.names:
        var = self.variables[name]
        variables[name] = {
            "shape": var.shape,
            "dtype": str(var.dtype),
            "bitwidth": evaluate_bitwidth(var),
            "data": var.tolist()
        }
    has_shapes = "input_channels" in dir(self.parameters) or len(self.inbounds) > 0
    return {
        "name": self.name,
        "parameters": params,
        "variables": variables,
        "inbounds": [layer.name for layer in self.inbounds],
        "input_shape": self.input_dims if has_shapes else None,
        "output_shape": self.output_dims if has_shapes else None
    }


def set_variable(self, name, values):
    """Set the value of a layer variable.

    Layer variables are named entities representing the weights or
    thresholds used during inference:

    * Weights variables are typically integer arrays of shape:

      (num_neurons, features/channels, y, x) col-major ordered ('F')

    or equivalently:

      (x, y, features/channels, num_neurons) row-major ('C').

    * Threshold variables are typically integer or float arrays of shape:
      (num_neurons).

    Args:
        name (str): the variable name.
        values (:obj:`numpy.ndarray`): a numpy.ndarray containing the variable values.

    """
    self.variables[name] = np.ascontiguousarray(values)


def get_variable(self, name):
    """Get the value of a layer variable.

    Layer variables are named entities representing the weights or
    thresholds used during inference:

    * Weights variables are typically integer arrays of shape:
      (x, y, features/channels, num_neurons) row-major ('C').
    * Threshold variables are typically integer or float arrays of shape:
      (num_neurons).

    Args:
        name (str): the variable name.

    Returns:
        :obj:`numpy.ndarray`: an array containing the variable.

    """
    return self.variables[name]


def get_variable_names(self):
    """Get the list of variable names for this layer.

    Returns:
        a list of variable names.

    """
    return self.variables.names


def get_learning_histogram(self):
    """Returns an histogram of learning percentages.

    Returns a list of learning percentages and the associated number of
    neurons.

    Returns:
        :obj:`numpy.ndarray`: a (n,2) numpy.ndarray containing the learning
        percentages and the number of neurons.

    """
    histogram = np.zeros((100, 2), dtype=np.uint32)
    num_neurons = self.get_variable("weights").shape[3]
    num_weights = np.count_nonzero(self.get_variable("weights")) / num_neurons
    for i in range(num_neurons):
        threshold_learn = self.get_variable("threshold_learning")[i]
        learn_percentage = int(100 * threshold_learn / num_weights)
        histogram[learn_percentage, 0] = learn_percentage
        histogram[learn_percentage, 1] += 1
    return histogram[histogram[:, 0] != 0, :]
