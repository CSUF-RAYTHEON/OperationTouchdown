from akida.core import Layer, LayerType, LayerParams


class FullyConnected(Layer):
    """This represents a Dense or Linear neural layer.

    A standard Dense Layer in a network is converted to FullyConnected Layer
    on Akida.
    This layer optionally executes ReLU operation to the outputs of the Dense
    Layer.

    The FullyConnected layer accepts 1-bit, 2-bit or 4-bit input tensors.
    The FullyConnected can be configured with 1-bit, 2-bit or 4-bit weights.
    It multiplies the inputs by its internal unit weights, returning a 4D
    tensor of values whose first dimension is the number of samples and the
    last dimension represents the number of units.
    It can optionally apply a step-wise ReLU activation to its outputs.

    Args:
        units (int): number of units.
        name (str, optional): name of the layer. Defaults to empty string.
        weights_bits (int, optional): number of bits used to quantize weights.
             Defaults to 1.
        activation (bool, optional): enable or disable activation
            function. Defaults to True.
        act_bits (int, optional): number of bits used to quantize the neuron
            response. Defaults to 1.

    """

    def __init__(self,
                 units,
                 name="",
                 weights_bits=1,
                 activation=True,
                 act_bits=1):
        try:
            params = LayerParams(
                LayerType.FullyConnected, {
                    "units": units,
                    "weights_bits": weights_bits,
                    "activation": activation,
                    "act_bits": act_bits
                })

            # Call parent constructor to initialize C++ bindings
            # Note that we invoke directly __init__ instead of using super, as
            # specified in pybind documentation
            Layer.__init__(self, params, name)
        except BaseException:
            self = None
            raise
