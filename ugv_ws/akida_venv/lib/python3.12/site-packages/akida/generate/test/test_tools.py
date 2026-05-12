import numpy as np
import akida


def get_cyclic_values(shape, bitwidth, duplicate_values, dtype):
    input_size = np.prod(shape)
    inputs = np.zeros((input_size), dtype=np.int32)
    max_value = 2**(bitwidth) - 1
    for i in range(input_size):
        inputs[i] = (i // duplicate_values) % (max_value + 1)
    if np.issubdtype(dtype, np.signedinteger):
        # If signed, correct max value and re-center on 0. Note that if shape
        # is small or duplicate_values is True, np.max(value) might not be
        # max_value.
        # If max / 2 is odd, the range is [-(max // 2) - 1, (max // 2)]
        inputs -= np.ceil(np.max(inputs) / 2).astype(np.int32)
        assert np.all((inputs >= -2**(bitwidth - 1)) & (inputs <= 2**(bitwidth - 1) - 1))
    return inputs.reshape(shape).astype(dtype)


def input_type_for_model(model):
    first_layer = model.get_layer(0)
    first_layer_idata = first_layer.parameters.layer_type == akida.LayerType.InputData
    # If input is InputData and its output is signed, input is signed
    if first_layer_idata and first_layer.output_signed:
        bitwidth = first_layer.input_bits
        if bitwidth > 8:
            dtype = np.int16
        else:
            dtype = np.int8
    else:
        dtype = np.uint8
    return dtype


def get_cyclic_input(model, n=1, duplicate_values=1):
    """
    Generate an input with cyclic values.

    Args:
        model (:obj:`akida.Model`): the model for which we want an input
        n (int, optional): number of frames or samples. Defaults to 1.
        duplicate_values (int, optional): duplication of each value. For
            example, a "1,2,3" pattern of values with a duplicate_values of 2
            will be "1,1,2,2,3,3". Defaults to 1, the values are not duplicated.
    Returns:
        :obj:`np.ndarray`: the cyclic inputs
    """
    assert duplicate_values >= 1, "duplicate_values must be strictly positive"
    input_shape = (n,) + tuple(model.input_shape)
    dtype = input_type_for_model(model)
    bitwidth = model.get_layer(0).input_bits
    return get_cyclic_values(input_shape,
                             bitwidth=bitwidth,
                             duplicate_values=duplicate_values,
                             dtype=dtype)


def set_cyclic_weights(model, bits=4, var_names=["weights", "weights_pw"]):
    for j in range(model.layer_count):
        layer = model.get_layer(j)
        var_list = layer.variables.names if var_names is None else var_names
        for var_name in var_list:
            if var_name in layer.variables.names:
                weights = layer.get_variable(var_name)
                if 'weights_bits' in dir(layer.parameters):
                    bitwidth = layer.parameters.weights_bits
                else:
                    bitwidth = bits
                # Generate a vector of cycling weights
                new_weights = np.zeros_like(weights).flatten()
                if bitwidth == 1:
                    for i in range(weights.size):
                        new_weights[i] = i % 2
                else:
                    if new_weights.size >= 2**(bitwidth - 1):
                        max_value = 2**(bitwidth - 1) - 1
                    else:
                        max_value = new_weights.size
                    n_values = 2 * max_value + 1
                    for i in range(weights.size):
                        # We set weights in sequence, each new sequence
                        # starting at an increasing value in the sequence
                        new_weights[i] = (i + i % n_values) % n_values - max_value
                # Weights are WHCF, but we want to cycle in each filter, so we
                # reshape the vector to FCHW to have weights grouped by filters
                new_weights = new_weights.reshape(np.flip(weights.shape))
                # ... then transpose to obtain the WHCF weights
                new_weights = np.transpose(new_weights)
                layer.set_variable(var_name, new_weights)


def set_constant_weights(model, value):
    for i in range(model.layer_count):
        layer = model.get_layer(i)
        for var_name in "weights", "weights_pw":
            if var_name in layer.variables.names:
                weights = layer.get_variable(var_name)
                weights[:, :, :, :] = value
                layer.set_variable(var_name, weights)
